#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_default.hpp>

#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>

namespace sdm
{

    double PWLCValueFunction::PRECISION = config::PRECISION_SDMS_VECTOR;

    PWLCValueFunction::PWLCValueFunction(number horizon,
                                         const std::shared_ptr<Initializer> &initializer,
                                         const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                         const std::shared_ptr<PWLCUpdateOperator> &update_operator,
                                         int freq_pruning,
                                         TypeOfMaxPlanPrunning type_of_maxplan_prunning)
        : PWLCValueFunctionInterface(horizon, initializer, action_selection, update_operator, freq_pruning),
          type_of_maxplan_prunning_(type_of_maxplan_prunning)
    {
        // Create all different structure in order to use the hyperplan value function.
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : horizon + 1, HyperplanSet({}));
        this->all_state_updated_so_far = std::vector<std::unordered_set<std::shared_ptr<State>>>(this->isInfiniteHorizon() ? 1 : horizon + 1, std::unordered_set<std::shared_ptr<State>>());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : horizon + 1, 0);
    }

    void PWLCValueFunction::initialize(double value, number t)
    {
        this->default_values_per_horizon[t] = value;
    }

    void PWLCValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    double PWLCValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        double value = this->evaluate(state, t).second;
        return value;
    }

    double PWLCValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        double value = this->evaluate(state, t).second;
        return value;
    }

    // Pair<std::shared_ptr<State>, double> PWLCValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    // {

    //     template <class TVector>
    //     static bool product_compare(TVector hyperplan1, TVector hyperplan2)
    //     {
    //         return ((belief ^ hyperplan1) < (belief ^ hyperplan2));
    //     }

    //     auto hyperplan_set = this->getSupport(t);
    //     return std::max_element(hyperplan_set.begin(), hyperplan_set.end(), product_compare);
    // }

    Pair<std::shared_ptr<State>, double> PWLCValueFunction::evaluate(const std::shared_ptr<State> &state, number t)
    {
        try
        {
            double current, max = -std::numeric_limits<double>::max();
            std::shared_ptr<BeliefInterface> alpha_vector;

            auto belief_state = state->toBelief();

            // Create Default State
            this->createDefault(state, t);

            // Go over all hyperplan in the support
            for (const auto &plan : this->getSupport(t))
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
            // std::cout<<"New pplan"<<std::endl;
            this->representation[t].push_back(new_hyperplan);
        }
    }

    double PWLCValueFunction::getQValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        // Cast the world into a MPOMDP 
        auto mpomdp = std::dynamic_pointer_cast<MPOMDPInterface>(this->getWorld()->getUnderlyingProblem());
        
        // Determine the reward for the hidden state and the action
        double factor = mpomdp->getReward(x, u, t);

        // Go over all reachable next hidden state
        for (const auto &y : mpomdp->getReachableStates(x, u, t))
        {
            // Go over all reachable observation
            for (const auto &z : mpomdp->getReachableObservations(x, u, y, t))
            {
                // Determine the next joint history conditionning to the observation
                auto o_ = o->expand(z->toObservation())->toJointHistory();
                factor += this->getWorld()->getDiscount(t) * this->tmp_representation->toOccupancyState()->getProbability(o_, y) * mpomdp->getDynamics(x, u, y, z, t);
            }
        }
        return factor;
    }

    // void PWLCValueFunction::updateValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    // {

    //     this->getUpdateOperator()->update(state, action, t);

    //     // Determine the new hyperplan
    //     const auto &new_hyperplan = this->template backup<std::shared_ptr<State>>(state, action, t)->toBelief();

    //     // If the hyperplan doesn't exit, we add it to representation at t
    //     if (!this->exist(new_hyperplan, t))
    //     {
    //         // std::cout<<"New pplan"<<std::endl;
    //         this->representation[t].push_back(new_hyperplan);

    //         // Add state to all state update so far, only if the prunning used is Bounded
    //         if (this->type_of_maxplan_prunning_ == TypeOfMaxPlanPrunning::BOUNDED)
    //             this->all_state_updated_so_far[t].insert(state);
    //     }
    // }

    // void PWLCValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    // {
    //     this->updateValueAt(state, this->getGreedyAction(state, t), t);
    // }

    std::vector<std::shared_ptr<State>> PWLCValueFunction::getSupport(number t)
    {
        return this->representation[t];
    }

    double PWLCValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[t];
    }

    void PWLCValueFunction::prune(number t)
    {
        switch (this->type_of_maxplan_prunning_)
        {
        case TypeOfMaxPlanPrunning::PAIRWISE:
            this->pairwise_prune(t);
            break;
        case TypeOfMaxPlanPrunning::BOUNDED:
            this->bounded_prune(t);

        default:
            break;
        }
    }

    void PWLCValueFunction::pairwise_prune(number t)
    {
        std::vector<std::shared_ptr<BeliefInterface>> hyperplan_not_to_be_deleted;
        std::vector<std::shared_ptr<BeliefInterface>> hyperplan_to_delete;

        // Go over all hyperplan
        for (const auto &alpha : this->getSupport(t))
        {
            bool alpha_dominated = false;

            // Go over all hyperplan in hyperplan_not_to_be_deleted
            for (const auto &beta : hyperplan_not_to_be_deleted)
            {
                // If beta dominate alpha, we had alpha to the hyperplan to delete
                if (alpha->toBelief()->operator<(beta))
                {
                    hyperplan_to_delete.push_back(alpha->toBelief());
                    alpha_dominated = true;
                    break;
                }
            }
            // If alpha is dominated, we go to the next hyperplan
            if (alpha_dominated)
            {
                continue;
            }

            // Go over all hyperplan in hyperplan_not_to_be_deleted
            std::vector<std::shared_ptr<BeliefInterface>> erase_tempo;

            for (const auto &beta : hyperplan_not_to_be_deleted)
            {
                // If alpha dominate a vector in hyperplan_not_to_be_deleted, we deleted this vector
                if (beta->operator<(alpha->toBelief()))
                {
                    erase_tempo.push_back(beta);
                    hyperplan_to_delete.push_back(beta);
                }
            }

            for (const auto &erase : erase_tempo)
            {
                auto it = std::find(hyperplan_not_to_be_deleted.begin(), hyperplan_not_to_be_deleted.end(), erase);
                hyperplan_not_to_be_deleted.erase(it);
            }
            hyperplan_not_to_be_deleted.push_back(alpha->toBelief());
        }

        for (const auto &to_delete : hyperplan_to_delete)
        {
            this->representation[t].erase(std::find(this->representation[t].begin(), this->representation[t].end(), to_delete));
        }
    }

    void PWLCValueFunction::bounded_prune(number t)
    {
        std::unordered_map<std::shared_ptr<State>, number> refCount;
        auto all_plan = this->getSupport(t);

        // Initialize the count
        for (const auto &element : all_plan)
        {
            refCount[element] = 0;
        }

        //<! update the count
        std::shared_ptr<State> max_alpha;
        double max_value = -std::numeric_limits<double>::max(), value;
        for (const auto &hyperplan : this->all_state_updated_so_far[t])
        {
            for (const auto &alpha : refCount)
            {
                if (max_value < (value = (hyperplan->toBelief()->operator^(alpha.first->toBelief()))))
                {
                    max_value = value;
                    max_alpha = alpha.first;
                }
            }
            refCount.at(max_alpha)++;
        }

        // Delete element with a count of 0
        for (const auto &element : all_plan)
        {
            if (refCount.at(element) == 0)
            {
                this->representation[t].erase(std::find(this->representation[t].begin(), this->representation[t].end(), element));
            }
        }
    }

    bool PWLCValueFunction::exist(const std::shared_ptr<BeliefInterface> &new_vector, number t, double)
    {
        // Go over all element in the Support
        for (const auto &element : this->representation[t])
        {
            // Test if the new vector is equal to the element
            if (new_vector->operator==(element->toBelief()))
            {
                return true;
            }
        }
        return false;
    }

    void PWLCValueFunction::createDefault(const std::shared_ptr<State> &state, number t)
    {
        // If there are not element at time t, we have to create the default State
        if (this->representation[t].size() == 0)
        {
            // Create the default state
            std::shared_ptr<BeliefInterface> default_state;

            switch (state->getTypeState())
            {
            case TypeState::BELIEF_STATE:
                default_state = std::make_shared<Belief>();
                break;
            case TypeState::OCCUPANCY_STATE:
                default_state = std::make_shared<OccupancyState>();
                break;
            case TypeState::SERIAL_OCCUPANCY_STATE:
                default_state = std::make_shared<SerialOccupancyState>();
                break;
            default:
                throw sdm::exception::Exception("The initializer used is not available for this formalism !");
                break;
            }

            // Add default value of the default state
            default_state->setDefaultValue(this->getDefaultValue(t));
            // default_state->finalize();
            this->representation[t].push_back(default_state);
        }
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