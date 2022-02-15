#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>
#include <sdm/utils/linear_algebra/hyperplane/balpha.hpp>
#include <sdm/utils/linear_algebra/hyperplane/oalpha.hpp>

#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_default.hpp>

#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/world/base/belief_mdp_interface.hpp>

namespace sdm
{

    double PWLCValueFunction::PRECISION = config::PRECISION_SDMS_VECTOR;

    PWLCValueFunction::PWLCValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                         const std::shared_ptr<Initializer> &initializer,
                                         const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                         int freq_pruning,
                                         MaxplanPruning::Type type_of_maxplan_prunning)
        : ValueFunctionInterface(world, initializer, action_selection),
          ValueFunction(world, initializer, action_selection_),
          PWLCValueFunctionInterface(world, initializer, action_selection_, freq_pruning),
          type_of_maxplan_prunning_(type_of_maxplan_prunning)
    {
        // Create all different structure in order to use the hyperplan value function.
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, HyperplanSet({}));
        this->all_state_updated_so_far = std::vector<std::unordered_set<std::shared_ptr<State>>>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, std::unordered_set<std::shared_ptr<State>>());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
        this->pomdp = std::dynamic_pointer_cast<POMDPInterface>(world->getUnderlyingProblem());
    }

    PWLCValueFunction::PWLCValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                         const std::shared_ptr<Initializer> &initializer,
                                         const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                         Config config)
        : PWLCValueFunction(world, initializer, action_selection, config.get("freq_pruning", -1))
    {
        auto opt_int = config.getOpt<int>("pruning_type");
        auto opt_str = config.getOpt<std::string>("pruning_type");
        if (opt_int.has_value())
        {
            this->type_of_maxplan_prunning_ = (MaxplanPruning::Type)opt_int.value();
        }
        else if (opt_str.has_value())
        {
            auto iter = MaxplanPruning::TYPE_MAP.find(opt_str.value());
            this->type_of_maxplan_prunning_ = (iter != MaxplanPruning::TYPE_MAP.end()) ? iter->second : MaxplanPruning::PAIRWISE;
        }
    }

    PWLCValueFunction::PWLCValueFunction(const PWLCValueFunction &copy)
        : ValueFunctionInterface(copy.world_, copy.initializer_, copy.action_selection_),
          ValueFunction(copy.world_, copy.initializer_, copy.action_selection_),
          PWLCValueFunctionInterface(copy.world_, copy.initializer_, copy.action_selection_, copy.freq_pruning),
          representation(copy.representation),
          default_values_per_horizon(copy.default_values_per_horizon),
          type_of_maxplan_prunning_(copy.type_of_maxplan_prunning_),
          all_state_updated_so_far(copy.all_state_updated_so_far)
    {
    }

    void PWLCValueFunction::initialize(double value, number t)
    {
        this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t] = value;
        auto initial_state = this->getWorld()->getInitialState();

        // If there are not element at time t, we have to create the default State
        if (this->representation[this->isInfiniteHorizon() ? 0 : t].size() == 0)
        {
            // Create the default state
            std::shared_ptr<AlphaVector> default_hyperplane;

            if (sdm::isInstanceOf<OccupancyStateInterface>(initial_state))
            {
                default_hyperplane = std::make_shared<oAlpha>(value);
            }
            else if (sdm::isInstanceOf<BeliefInterface>(initial_state))
            {
                // default_hyperplane = std::make_shared<BeliefAlphaVector>();
                default_hyperplane = std::make_shared<bAlpha>(value);
            }
            else
            {
                throw sdm::exception::TypeError("TypeError : state must derived from belief");
            }

            this->addHyperplaneAt(initial_state, default_hyperplane, t);
        }
    }

    void PWLCValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    double PWLCValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->evaluate(state, t).second;
    }

    Pair<std::shared_ptr<Hyperplane>, double> PWLCValueFunction::evaluate(const std::shared_ptr<State> &state, number t)
    {
        double current, max = -std::numeric_limits<double>::max();
        std::shared_ptr<AlphaVector> alpha_vector = nullptr;

        // Go over all hyperplan in the support
        for (const auto &plan : this->representation[this->isInfiniteHorizon() ? 0 : t])
        {
            // Determine the best hyperplan which give the best value for the current state
            if (max < (current = state->product(plan)))
            {
                max = current;
                alpha_vector = plan;
            }
        }
        return {alpha_vector, max};
    }

    void PWLCValueFunction::addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &new_hyperplan, number t)
    {
        // Add hyperplane in the hyperplane set
        this->representation[this->isInfiniteHorizon() ? 0 : t].insert(std::static_pointer_cast<AlphaVector>(new_hyperplan));

        // Add state to all state update so far, only if the prunning used is Bounded
        if (this->type_of_maxplan_prunning_ == MaxplanPruning::Type::BOUNDED)
            this->all_state_updated_so_far[this->isInfiniteHorizon() ? 0 : t].insert(state);
    }

    double PWLCValueFunction::getBeta(const std::shared_ptr<Hyperplane> &hyperplane, const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        auto alpha = std::static_pointer_cast<AlphaVector>(hyperplane);
        return alpha->getBetaValueAt(x, o, u, this->pomdp, t);
    }

    std::vector<std::shared_ptr<Hyperplane>> PWLCValueFunction::getHyperplanesAt(std::shared_ptr<State>, number t)
    {
        auto set = this->representation[this->isInfiniteHorizon() ? 0 : t];
        return std::vector<std::shared_ptr<Hyperplane>>(set.begin(), set.end());
    }

    PWLCValueFunction::HyperplanSet &PWLCValueFunction::getAlphaHyperplanesAt(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    std::shared_ptr<Hyperplane> PWLCValueFunction::getHyperplaneAt(std::shared_ptr<State> state, number t)
    {
        return this->evaluate(state, t).first;
    }

    std::vector<std::shared_ptr<State>> PWLCValueFunction::getSupport(number t)
    {
        throw sdm::exception::NotImplementedException();
        // return this->getHyperplanesAt(nullptr, t);
    }

    std::shared_ptr<ValueFunctionInterface> PWLCValueFunction::copy()
    {
        auto casted_value = std::dynamic_pointer_cast<PWLCValueFunction>(this->getptr());
        return std::make_shared<PWLCValueFunction>(*casted_value);
    }

    double PWLCValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[this->isInfiniteHorizon() ? 0 : t];
    }

    void PWLCValueFunction::prune(number t)
    {
        switch (this->type_of_maxplan_prunning_)
        {
        case MaxplanPruning::PAIRWISE:
            this->pairwise_prune(t);
            break;
        case MaxplanPruning::BOUNDED:
            this->bounded_prune(t);
        default:
            break;
        }
    }

    void PWLCValueFunction::pairwise_prune(number t)
    {
        // List of hyperplanes that are \eps-dominated
        std::vector<std::shared_ptr<AlphaVector>> hyperplan_to_delete;
        // List of hyperplanes that are not dominated
        std::vector<std::shared_ptr<AlphaVector>> hyperplan_to_keep;

        auto &all_hyperplanes = this->getAlphaHyperplanesAt(t);

        // Go over all current hyperplanes
        for (const auto &alpha : all_hyperplanes)
        {
            bool alpha_dominated = false;

            // Go over all hyperplanes in hyperplan_to_keep
            for (auto beta_iter = hyperplan_to_keep.begin(); beta_iter != hyperplan_to_keep.end(); beta_iter++)
            {
                // If beta dominate alpha, we add alpha to the set of hyperplanes to delete
                if (alpha->isDominated(**beta_iter))
                {
                    hyperplan_to_delete.push_back(alpha);
                    alpha_dominated = true;
                    break;
                }
            }
            // If alpha is dominated, we reject alpha and go to the next iteration of outer loop
            if (alpha_dominated)
                continue;

            // Go over all hyperplanes in hyperplan_to_keep
            for (auto beta_iter = hyperplan_to_keep.begin(); beta_iter != hyperplan_to_keep.end();)
            {
                // If alpha dominate a vector in hyperplan_to_keep, we deleted this vector
                if (alpha->dominate(**beta_iter))
                {
                    hyperplan_to_delete.push_back(*beta_iter);
                    beta_iter = hyperplan_to_keep.erase(beta_iter);
                }
                else
                {
                    beta_iter++;
                }
            }

            hyperplan_to_keep.push_back(alpha);
        }

        // Erase dominated hyperplanes
        for (const auto &to_delete : hyperplan_to_delete)
        {
            all_hyperplanes.erase(std::find(all_hyperplanes.begin(), all_hyperplanes.end(), to_delete));
        }
    }

    void PWLCValueFunction::bounded_prune(number t)
    {
        std::unordered_map<std::shared_ptr<Hyperplane>, number> refCount;
        auto &all_hyperplanes = this->getAlphaHyperplanesAt(t);

        // Initialize the count
        for (const auto &hyperplane : all_hyperplanes)
        {
            refCount[hyperplane] = 0;
        }

        // Update the count depending on visited beliefs
        std::shared_ptr<Hyperplane> max_alpha;
        double max_value = -std::numeric_limits<double>::max(), value;
        for (const auto &hyperplane : this->all_state_updated_so_far[t])
        {
            for (const auto &alpha : all_hyperplanes)
            {
                if (max_value < (value = (hyperplane->product(alpha))))
                {
                    max_value = value;
                    max_alpha = alpha;
                }
            }
            refCount.at(max_alpha)++;
        }

        // Delete hyperplanes with a count of 0
        for (auto hyperplane_iter = all_hyperplanes.begin(); hyperplane_iter != all_hyperplanes.end();)
        {
            if (refCount.at(*hyperplane_iter) == 0)
                hyperplane_iter = all_hyperplanes.erase(hyperplane_iter);
            else
                hyperplane_iter++;
        }
    }

    std::string PWLCValueFunction::str() const
    {
        std::ostringstream res;
        res << "<pwlc_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;

        for (number i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;
            for (const auto &plan : this->representation[i])
            {
                std::ostringstream hyperplan_str;
                hyperplan_str << plan->str();
                tools::indentedOutput(res, hyperplan_str.str().c_str(), 2);
            }
            res << "\t</value>" << std::endl;
        }

        res << "</pwlc_value_function>" << std::endl;
        return res.str();
    }

    size_t PWLCValueFunction::getSize(number t) const
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].size();
    }
} // namespace sdm