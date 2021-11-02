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
                                         const std::shared_ptr<PWLCUpdateOperator> &update_operator,
                                         int freq_pruning,
                                         MaxplanPruning::Type type_of_maxplan_prunning)
        : ValueFunctionInterface(world, initializer, action_selection),
          ValueFunction(world, initializer, action_selection_, update_operator),
          PWLCValueFunctionInterface(world, initializer, action_selection_, freq_pruning),
          type_of_maxplan_prunning_(type_of_maxplan_prunning)
    {
        // Create all different structure in order to use the hyperplan value function.
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, HyperplanSet({}));
        this->all_state_updated_so_far = std::vector<std::unordered_set<std::shared_ptr<State>>>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, std::unordered_set<std::shared_ptr<State>>());
        this->default_values_per_horizon = std::vector<double>(this->isInfiniteHorizon() ? 1 : world->getHorizon() + 1, 0);
    }

    void PWLCValueFunction::initialize(double value, number t)
    {
        this->default_values_per_horizon[t] = value;
        auto initial_state = this->getWorld()->getInitialState();

        // If there are not element at time t, we have to create the default State
        if (this->representation[t].size() == 0)
        {
            // Create the default state
            std::shared_ptr<BeliefInterface> default_state;

            if (sdm::isInstanceOf<OccupancyStateInterface>(initial_state))
            {
                default_state = std::make_shared<OccupancyState>();
            }
            else if (sdm::isInstanceOf<BeliefInterface>(initial_state))
            {
                default_state = std::make_shared<Belief>();
            }
            else
            {
                throw sdm::exception::TypeError("TypeError : state must derived from belief");
            }

            // Add default value of the default state
            default_state->setDefaultValue(value);

            this->representation[t].push_back(default_state);
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
    }

    double PWLCValueFunction::getNextAlphaValue(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation)
    {
        if (sdm::isInstanceOf<OccupancyStateInterface>(alpha))
        {
            return getNextAlphaValueOccupancy(alpha, state, history, action, next_state, observation);
        }
        else if (sdm::isInstanceOf<BeliefInterface>(alpha))
        {
            return getNextAlphaValueBelief(alpha, state, history, action, next_state, observation);
        }
        else
        {
            throw sdm::exception::TypeError("TypeError : state must derived from belief");
        }
    }

    double PWLCValueFunction::getNextAlphaValueBelief(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &)
    {
        return alpha->toBelief()->getVectorInferface()->getValueAt(next_state);
    }

    double PWLCValueFunction::getNextAlphaValueOccupancy(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation)
    {
        return alpha->toOccupancyState()->getProbability(history->expand(observation)->toJointHistory(), next_state);
    }

    double PWLCValueFunction::getBeta(const std::shared_ptr<State> &alpha, const std::shared_ptr<State> &state, const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, number t)
    {
        // Compute \beta_t(x,o,u) = R(x,u) + \gamma \sum_{y, z} p^{uz}_{xy} \alpha_{t+1}(y, (o,u,z))
        auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());

        double next_expected_value = 0.0;

        // Go over all hidden state reachable next state
        for (const auto &next_state : pomdp->getReachableStates(state, action, t))
        {
            // Go over all observation reachable observation
            for (const auto &observation : pomdp->getReachableObservations(state, action, next_state, t))
            {
                // Get the next value of an hyperplane
                double alpha_ = this->getNextAlphaValue(alpha, state, history, action, next_state, observation);

                // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
                next_expected_value += alpha_ * pomdp->getDynamics(state, action, next_state, observation, t);
            }
        }
        return pomdp->getReward(state, action, t) + this->getWorld()->getDiscount(t) * next_expected_value;
    }

    // std::vector<std::shared_ptr<State>> PWLCValueFunction::getBeta(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    // {

    //     std::shared_ptr<Space> obs_space = this->getWorld()->getObservationSpace(t);
    //     std::unordered_map<std::shared_ptr<Observation>, std::shared_ptr<State>> alpha;
    //     for (const auto &observation : *obs_space)
    //     {
    //         alpha[observation] = evaluate(getWorld()->getNextStateAndProba(state, action, observation, t).first, t + 1).first;
    //     }

    //     // Creation of a new beta hyperplane
    //     MappedVector<Tuple<std::shared_ptr<State>, std::shared_ptr<Action>>> beta_hyperplane;
    //     beta_hyperplane->setDefaultValue(value_function->getDefaultValue(t));

    //     // For each hidden state, we associate the value \beta(x, u) = r(x,u) + \gamma * \sum_{y, z} p^{u,z}_{x,y} * \alpha^{u,z}(y);
    //     for (const auto &state : belief_state->getStates())
    //     {
    //         for (const auto &action : pomdp->getActionSpace(t))
    //         {
    //             double next_expected_value = 0;
    //             // Go over all hidden state reachable next state
    //             for (const auto &next_state : pomdp->getReachableStates(state, action, t))
    //             {
    //                 // Go over all observation reachable observation
    //                 for (const auto &observation : pomdp->getReachableObservations(state, action, next_state, t))
    //                 {
    //                     // Get the next value of an hyperplane
    //                     double alpha_ = this->getNextAlphaValue(alpha[observation], state, nullptr, action, next_state, observation);

    //                     // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
    //                     next_expected_value += alpha[observation]->toBelief()->getValueAt(next_state) * pomdp->getDynamics(state, action, next_state, observation, t);
    //                 }
    //             }
    //             beta_hyperplane->setValueAt({state, action}, this->getBeta(alpha[observation], state, nullptr, action, t));//pomdp->getReward(state, action, t) + this->getWorld()->getDiscount(t) * next_expected_value);
    //         }
    //     }
    //     beta_hyperplane->finalize();
    //     return beta_hyperplane;
    // }


    // std::vector<std::shared_ptr<State>> PWLCValueFunction::getBeta(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    // {

    //     std::shared_ptr<Space> obs_space = this->getWorld()->getObservationSpace(t);
    //     std::unordered_map<std::shared_ptr<Observation>, std::shared_ptr<State>> alpha;
    //     for (const auto &observation : *obs_space)
    //     {
    //         alpha[observation] = evaluate(getWorld()->getNextStateAndProba(state, action, observation, t).first, t + 1).first;
    //     }

    //     // Creation of a new beta hyperplane
    //     MappedVector<Tuple<std::shared_ptr<State>, std::shared_ptr<HistoryInterface>, std::shared_ptr<Action>>> beta_hyperplane;
    //     beta_hyperplane->setDefaultValue(value_function->getDefaultValue(t));

    //     // For each hidden state, we associate the value \beta(x, o, u) = r(x,u) + \gamma * \sum_{y, z} p^{u,z}_{x,y} * \alpha^{u,z}(y, (o,u,z));
    //     for (const auto &state : belief_state->getStates())clear
    //     {
    //         for (const auto &action : pomdp->getActionSpace(t))
    //         {
    //             double next_expected_value = 0;
    //             // Go over all hidden state reachable next state
    //             for (const auto &next_state : pomdp->getReachableStates(state, action, t))
    //             {
    //                 // Go over all observation reachable observation
    //                 for (const auto &observation : pomdp->getReachableObservations(state, action, next_state, t))
    //                 {
    //                     // Get the next value of an hyperplane
    //                     double alpha_ = this->getNextAlphaValue(alpha[observation], state, nullptr, action, next_state, observation);

    //                     // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
    //                     next_expected_value += alpha[observation]->toBelief()->getValueAt(next_state) * pomdp->getDynamics(state, action, next_state, observation, t);
    //                 }
    //             }
    //             beta_hyperplane->setValueAt({state, action}, this->getBeta(alpha[observation], state, nullptr, action, t));//pomdp->getReward(state, action, t) + this->getWorld()->getDiscount(t) * next_expected_value);
    //         }
    //     }
    //     beta_hyperplane->finalize();
    //     return beta_hyperplane;
    // }

    std::vector<std::shared_ptr<State>> PWLCValueFunction::getHyperplanesAt(const std::shared_ptr<State>&, number t)
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
    //         if (this->type_of_maxplan_prunning_ == MaxplanPruning::Type::BOUNDED)
    //             this->all_state_updated_so_far[t].insert(state);
    //     }
    // }

    double PWLCValueFunction::getDefaultValue(number t)
    {
        return this->default_values_per_horizon[t];
    }

    void PWLCValueFunction::prune(number t)
    {
        switch (this->type_of_maxplan_prunning_)
        {
        case MaxplanPruning::Type::PAIRWISE:
            this->pairwise_prune(t);
            break;
        case MaxplanPruning::Type::BOUNDED:
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