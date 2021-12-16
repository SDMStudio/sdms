#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

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
        this->default_values_per_horizon[t] = value;
        auto initial_state = this->getWorld()->getInitialState();

        // If there are not element at time t, we have to create the default State
        if (this->representation[t].size() == 0)
        {
            // Create the default state
            std::shared_ptr<BeliefInterface> default_hyperplane;

            if (sdm::isInstanceOf<OccupancyStateInterface>(initial_state))
            {
                default_hyperplane = std::make_shared<OccupancyState>();
            }
            else if (sdm::isInstanceOf<BeliefInterface>(initial_state))
            {
                default_hyperplane = std::make_shared<Belief>();
            }
            else
            {
                throw sdm::exception::TypeError("TypeError : state must derived from belief");
            }

            // Add default value of the default state
            default_hyperplane->setDefaultValue(value);

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

    Pair<std::shared_ptr<State>, double> PWLCValueFunction::evaluate(const std::shared_ptr<State> &state, number t)
    {
        try
        {
            double current, max = -std::numeric_limits<double>::max();
            std::shared_ptr<BeliefInterface> alpha_vector = nullptr;

            auto belief_state = state->toBelief();

            // Go over all hyperplan in the support
            for (const auto &plan : this->getHyperplanesAt(state, t))
            {
                auto belief_plan = plan->toBelief();

                // Determine the best hyperplan which give the best value for the current state
                if (max < (current = belief_state->operator^(belief_plan)))
                {
                    max = current;
                    alpha_vector = belief_plan;
                }
            }
            return {alpha_vector, max};
        }
        catch (const std::exception &e)
        {
            std::cerr << "PWLCValueFunction::evaluate error" << e.what() << '\n';
            exit(-1);
        }
    }

    void PWLCValueFunction::addHyperplaneAt(const std::shared_ptr<State> &state, const std::shared_ptr<State> &new_hyperplan, number t)
    {
        if (!this->exist(new_hyperplan->toBelief(), t))
        {
            this->representation[t].push_back(new_hyperplan);
        }

        // Add state to all state update so far, only if the prunning used is Bounded
        if (this->type_of_maxplan_prunning_ == MaxplanPruning::Type::BOUNDED)
            this->all_state_updated_so_far[t].insert(state);
    }

    double PWLCValueFunction::getNextAlphaValue(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t)
    {
        if (auto oalpha = sdm::isInstanceOf<OccupancyStateInterface>(alpha))
        {
            return getNextAlphaValueOccupancy(oalpha, state, history, action, next_state, observation, t);
        }
        else if (auto balpha = sdm::isInstanceOf<BeliefInterface>(alpha))
        {
            return getNextAlphaValueBelief(balpha, state, history, action, next_state, observation, t);
        }
        else
        {
            throw sdm::exception::TypeError("TypeError : state must derived from belief");
        }
    }

    double PWLCValueFunction::getNextAlphaValueBelief(const std::shared_ptr<BeliefInterface> &alpha, const std::shared_ptr<State> &, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &, number)
    {
        return alpha->getProbability(next_state);
    }

    double PWLCValueFunction::getNextAlphaValueOccupancy(const std::shared_ptr<OccupancyStateInterface> &alpha, const std::shared_ptr<State> &, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number)
    {
        return alpha->getProbability(std::dynamic_pointer_cast<JointHistoryInterface>(history->expand(observation)), next_state);
    }

    double PWLCValueFunction::getBeta(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t)
    {
        bool is_occupancy;
        std::shared_ptr<OccupancyStateInterface> oalpha;
        std::shared_ptr<BeliefInterface> balpha;
        if ((oalpha = sdm::isInstanceOf<OccupancyStateInterface>(alpha)))
            is_occupancy = true;
        else if ((balpha = sdm::isInstanceOf<BeliefInterface>(alpha)))
            is_occupancy = false;
        else
            throw sdm::exception::TypeError("TypeError : state must derived from belief");

        // Compute \beta_t(x,o,u) = R(x,u) + \gamma \sum_{y, z} p^{uz}_{xy} \alpha_{t+1}(y, (o,u,z))
        double next_expected_value = 0.0;

        // Go over all hidden state reachable next state
        for (const auto &next_state : this->pomdp->getReachableStates(state, action, t))
        {
            // Go over all observation reachable observation
            for (const auto &observation : this->pomdp->getReachableObservations(state, action, next_state, t))
            {
                // Get the next value of an hyperplane
                double alpha_;
                if (is_occupancy)
                    alpha_ = this->getNextAlphaValueOccupancy(oalpha, state, history, action, next_state, observation, t);
                else
                    alpha_ = this->getNextAlphaValueBelief(balpha, state, history, action, next_state, observation, t);

                // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
                next_expected_value += alpha_ * this->pomdp->getDynamics(state, action, next_state, observation, t);
            }
        }
        return this->pomdp->getReward(state, action, t) + this->getWorld()->getDiscount(t) * next_expected_value;
    }

    std::vector<std::shared_ptr<State>> PWLCValueFunction::getHyperplanesAt(const std::shared_ptr<State> &, number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t];
    }

    std::shared_ptr<State> PWLCValueFunction::getHyperplaneAt(const std::shared_ptr<State> &state, number t)
    {
        return this->evaluate(state, t).first;
    }

    std::vector<std::shared_ptr<State>> PWLCValueFunction::getSupport(number t)
    {
        return this->getHyperplanesAt(nullptr, t);
    }

    std::shared_ptr<ValueFunctionInterface> PWLCValueFunction::copy()
    {
        auto casted_value = std::dynamic_pointer_cast<PWLCValueFunction>(this->getptr());
        return std::make_shared<PWLCValueFunction>(*casted_value);
    }

    double PWLCValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[t];
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
        std::vector<std::shared_ptr<BeliefInterface>> hyperplan_to_delete;
        // List of hyperplanes that are not dominated
        std::vector<std::shared_ptr<BeliefInterface>> hyperplan_to_keep;

        // Go over all current hyperplanes
        for (const auto &alpha : this->getSupport(t))
        {
            bool alpha_dominated = false;

            // Go over all hyperplanes in hyperplan_to_keep
            for (auto beta_iter = hyperplan_to_keep.begin(); beta_iter != hyperplan_to_keep.end(); beta_iter++)
            {
                // If beta dominate alpha, we add alpha to the set of hyperplanes to delete
                if (alpha->toBelief()->operator<(*beta_iter))
                {
                    hyperplan_to_delete.push_back(alpha->toBelief());
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
                if ((*beta_iter)->operator<(alpha->toBelief()))
                {
                    hyperplan_to_delete.push_back(*beta_iter);
                    beta_iter = hyperplan_to_keep.erase(beta_iter);
                }
                else
                {
                    beta_iter++;
                }
            }

            hyperplan_to_keep.push_back(alpha->toBelief());
        }

        // Erase dominated hyperplanes
        for (const auto &to_delete : hyperplan_to_delete)
        {
            this->representation[t].erase(std::find(this->representation[t].begin(), this->representation[t].end(), to_delete));
        }
    }

    void PWLCValueFunction::bounded_prune(number t)
    {
        std::unordered_map<std::shared_ptr<State>, number> refCount;
        auto all_hyperplanes = this->representation[t];

        // Initialize the count
        for (const auto &hyperplane : all_hyperplanes)
        {
            refCount[hyperplane] = 0;
        }

        // Update the count depending on visited beliefs
        std::shared_ptr<State> max_alpha;
        double max_value = -std::numeric_limits<double>::max(), value;
        for (const auto &hyperplane : this->all_state_updated_so_far[t])
        {
            for (const auto &alpha : all_hyperplanes)
            {
                if (max_value < (value = (hyperplane->toBelief()->operator^(alpha->toBelief()))))
                {
                    max_value = value;
                    max_alpha = alpha;
                }
            }
            refCount.at(max_alpha)++;
        }

        // Delete hyperplanes with a count of 0
        for (auto hyperplane_iter = this->representation[t].begin(); hyperplane_iter != this->representation[t].end();)
        {
            if (refCount.at(*hyperplane_iter) == 0)
                hyperplane_iter = this->representation[t].erase(hyperplane_iter);
            else
                hyperplane_iter++;
        }
    }

    bool PWLCValueFunction::exist(const std::shared_ptr<BeliefInterface> &new_vector, number t, double)
    {
        // Go over all element in the Support
        for (const auto &hyperplane : this->representation[t])
        {
            // Test if the new vector is equal to the hyperplane
            if (new_vector->operator==(hyperplane->toBelief()))
            {
                return true;
            }
        }
        return false;
    }

    std::string PWLCValueFunction::str() const
    {
        std::ostringstream res;
        res << "<pwlc_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;

        for (number i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;
            for (auto plan : this->representation[i])
            {
                res << "\t\t<plan>" << std::endl;
                res << "\t\t\t" << plan->str() << std::endl;
                res << "\t\t</plan>" << std::endl;
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