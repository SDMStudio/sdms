
#include <sdm/utils/value_function/value_function.hpp>
namespace sdm
{

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction() {}

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, number horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer)
        : ValueFunction<TVector, TAction, TValue>(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, HyperplanSet({}));
        this->default_values_per_horizon = std::vector<TValue>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, 0);
    }

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunction<TVector, TAction, TValue>::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, number horizon, TValue default_value) : MaxPlanValueFunction(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value))
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::initialize(TValue value, number t)
    {
        TVector new_v(value);
        this->representation[t].push_back(new_v);
        this->default_values_per_horizon[t] = value;
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <typename TVector, typename TAction, typename TValue>
    std::pair<TValue, TVector> MaxPlanValueFunction<TVector, TAction, TValue>::getMaxAt(const TVector &state, number t)
    {
        TValue current, max = -std::numeric_limits<TValue>::max();
        TVector alpha_vector;

        for (const auto &plan : this->representation[t])
        {
            current = state ^ plan;

            if (max < current)
            {
                max = current;
                alpha_vector = plan;
            }
        }

        return {max, alpha_vector};
    }

    template <typename TVector, typename TAction, typename TValue>
    TValue MaxPlanValueFunction<TVector, TAction, TValue>::getValueAt(const TVector &state, number t)
    {
        return this->getMaxAt(state, t).first;
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
    {
        //std::cout<<"\n support ::::::"<<this->representation;
        auto new_hyperplan = this->backup_operator<TVector>(state, t);

        if (std::find(this->representation[t].begin(), this->representation[t].end(), new_hyperplan) == this->representation[t].end())
            this->representation[t].push_back(new_hyperplan);

        //this->prune(t);
    }

    template <typename TVector, typename TAction, typename TValue>
    std::vector<TVector> MaxPlanValueFunction<TVector, TAction, TValue>::getSupport(number t)
    {
        return this->representation[t];
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::prune(number t)
    {
        this->bounded_prune(t);
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunction<TVector, TAction, TValue>::bounded_prune(number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for bounded_prune().");
    }

    template <typename TVector, typename TAction, typename TValue>
    number MaxPlanValueFunction<TVector, TAction, TValue>::size()
    {
        return this->representation.size();
    }

    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &occupancy_state, number t)
    {
        auto oMDP = std::static_pointer_cast<OccupancyMDP<>>(this->getWorld());

        TVector v_max;
        double value_max = -std::numeric_limits<double>::max(), tmp;

        auto all_joint_decision_rules = oMDP->getActionSpaceAt(occupancy_state)->getAll();

        // Go over the hyperplanes of decision step t+1
        for (const auto &next_hyperplan : this->getSupport(t + 1))
        {
            // Go over all joint decision rules at occupancy occupancy_state
            for (const auto &joint_decision_rule : all_joint_decision_rules)
            {
                TVector v = this->getHyperplanAt<TVector>(occupancy_state, next_hyperplan, joint_decision_rule, t);
                if (value_max < (tmp = occupancy_state ^ v))
                {
                    value_max = tmp;
                    v_max = v;
                }
            }
        }
        return v_max;
    }

    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::getHyperplanAt(const TVector &occupancy_state, const TVector &next_hyperplan, const TAction &joint_decision_rule, number t)
    {
        TVector new_hyperplan(this->default_values_per_horizon[t]);

        // Go over all occupancy state
        for (const auto uncompressed_s_o : *occupancy_state.getFullyUncompressedOccupancy())
        {
            //Get information from uncompressed_s_o
            auto uncompressed_hidden_state = uncompressed_s_o.first.first;
            auto uncompressed_joint_history = uncompressed_s_o.first.second;
            auto compressed_joint_history = occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

            // Get the serial action from the serial_decision_rule
            auto action = joint_decision_rule.act(compressed_joint_history->getIndividualHistories());

            // Add the reward of the hyperplan
            new_hyperplan.addProbabilityAt(uncompressed_s_o.first, this->getWorld()->getUnderlyingProblem()->getReward()->getReward(uncompressed_hidden_state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action)));

            //Go ober all Reachable State
            for (auto next_hidden_state : this->getWorld()->getUnderlyingProblem()->getReachableStates(uncompressed_hidden_state, action))
            {
                //Go ober all Reachable Observation
                for (auto next_observation : this->getWorld()->getUnderlyingProblem()->getReachableObservations(uncompressed_hidden_state, action, next_hidden_state))
                {
                    auto next_joint_history = compressed_joint_history->expand(next_observation);
                    new_hyperplan.addProbabilityAt(uncompressed_s_o.first, this->getWorld()->getUnderlyingProblem()->getDiscount() * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(uncompressed_hidden_state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), this->getWorld()->getUnderlyingProblem()->getObsSpace()->joint2single(next_observation), next_hidden_state) * next_hyperplan.at({next_hidden_state, next_joint_history}));
                }
            }
        }
        return new_hyperplan;
    }

    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &occupancy_state, number t)
    {
        auto oMDP = std::static_pointer_cast<OccupancyMDP<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, JointDeterministicDecisionRule<HistoryTree_p<number>, number>>>(this->getWorld());

        TVector v_max;
        double value_max = -std::numeric_limits<double>::max(), tmp;

        auto all_joint_decision_rules = oMDP->getActionSpaceAt(occupancy_state)->getAll();

        // Go over the hyperplanes of decision step t+1
        for (const auto &next_hyperplan : this->getSupport(t + 1))
        {
            // Go over all joint decision rules at occupancy occupancy_state
            for (const auto &joint_decision_rule : all_joint_decision_rules)
            {
                TVector v = this->getHyperplanAt<TVector>(occupancy_state, next_hyperplan, joint_decision_rule, t);
                if (value_max < (tmp = occupancy_state ^ v))
                {
                    value_max = tmp;
                    v_max = v;
                }
            }
        }
        return v_max;
    }

    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::getHyperplanAt(const TVector &occupancy_state, const TVector &next_hyperplan, const TAction &joint_decision_rule, number t)
    {
        TVector new_hyperplan(this->default_values_per_horizon[t]);
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        // Go over all occupancy state
        for (const auto uncompressed_belief_history_proba : *occupancy_state.getFullyUncompressedOccupancy())
        {
            //Get information from uncompressed_belief_history_proba
            auto uncompressed_belief = uncompressed_belief_history_proba.first.first;
            auto uncompressed_joint_history = uncompressed_belief_history_proba.first.second;
            auto compressed_joint_history = occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

            // Get the serial action from the serial_decision_rule
            auto joint_action = joint_decision_rule.act(compressed_joint_history->getIndividualHistories());

            // Add the reward of the hyperplan
            new_hyperplan.addProbabilityAt(uncompressed_belief_history_proba.first, (uncompressed_belief->getData() ^ under_pb->getReward()->getReward(under_pb->getActionSpace()->joint2single(joint_action))));

            //Go over all Reachable Observation
            for (const auto &next_observation : under_pb->getObsSpace()->getAll())
            {
                auto next_joint_history = compressed_joint_history->expand(next_observation);
                auto next_belief = uncompressed_belief->expand(under_pb->getActionSpace()->joint2single(joint_action), under_pb->getObsSpace()->joint2single(next_observation));
                new_hyperplan.addProbabilityAt(uncompressed_belief_history_proba.first, under_pb->getDiscount() * uncompressed_belief->getProbability(under_pb->getActionSpace()->joint2single(joint_action), under_pb->getObsSpace()->joint2single(next_observation)) * next_hyperplan.at({next_belief, next_joint_history}));
            }
        }
        return new_hyperplan;
    }

    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &serial_occupancy_state, number t)
    {
        auto soMDP = std::static_pointer_cast<SerializedOccupancyMDP<TVector, TAction>>(this->getWorld());

        TVector v_max;
        double value_max = -std::numeric_limits<double>::max(), tmp;

        auto getAll_actionspace = soMDP->getActionSpaceAt(serial_occupancy_state)->getAll();

        // Go other the hyperplanes of decision step t+1
        for (const auto &next_hyperplan : this->getSupport(t + 1))
        {
            // Go over all joint decision rules at serial_occupancy_state
            for (const auto &serial_decision_rule : getAll_actionspace)
            {
                //Use the function getHyperplanAt
                TVector v = this->getHyperplanAt<TVector>(serial_occupancy_state, next_hyperplan, serial_decision_rule, t);

                // Choice the best hyperplan
                if (value_max < (tmp = serial_occupancy_state ^ v))
                {
                    value_max = tmp;
                    v_max = v;
                }
            }
        }
        return v_max;
    }

    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::getHyperplanAt(const TVector &serial_occupancy_state, const TVector &next_hyperplan, const TAction &serial_decision_rule, number t)
    {
        number ag_id = serial_occupancy_state.getCurrentAgentId();
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        TVector new_hyperplan(this->default_values_per_horizon[t]);
        new_hyperplan.setAgent(ag_id);

        // Go over all serial occupancy state
        for (const auto uncompressed_s_o : *serial_occupancy_state.getFullyUncompressedOccupancy())
        {
            //Get information from uncompressed_s_o
            auto uncompressed_hidden_serial_state = serial_occupancy_state.getState(uncompressed_s_o.first);
            auto uncompressed_joint_history = serial_occupancy_state.getHistory(uncompressed_s_o.first);
            auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

            // Get the serial action from the serial_decision_rule
            auto serial_action = serial_decision_rule.act(compressed_joint_history->getIndividualHistory(ag_id));

            // Add the reward of the hyperplan
            new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getReward(uncompressed_hidden_serial_state, serial_action));

            //Go ober all Reachable Serial State
            for (auto next_hidden_serial_state : under_pb->getReachableSerialStates(uncompressed_hidden_serial_state, serial_action))
            {
                //Go ober all Reachable Observation
                for (auto next_serial_observation : under_pb->getReachableObservations(uncompressed_hidden_serial_state, serial_action, next_hidden_serial_state))
                {
                    auto next_joint_history = compressed_joint_history->expand(next_serial_observation);
                    new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getDiscount(t) * under_pb->getDynamics(uncompressed_hidden_serial_state, serial_action, next_serial_observation, next_hidden_serial_state) * next_hyperplan.at({next_hidden_serial_state, next_joint_history}));
                }
            }
        }
        return new_hyperplan;
    }

    // For BeliefMDP
    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<BeliefState, T>, int>>
    TVector MaxPlanValueFunction<TVector, TAction, TValue>::backup_operator(const TVector &state, number t)
    {

        auto beliefMDP = std::static_pointer_cast<BeliefMDP<TVector, TAction, number>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        number n_obs = under_pb->getObsSpace()->getNumItems();
        number n_actions = under_pb->getActionSpace()->getNumItems();
        number n_states = under_pb->getStateSpace()->getNumItems();

        std::vector<std::vector<TVector>> beta_a_o(n_actions, std::vector<TVector>(n_obs, TVector()));
        std::vector<TVector> beta_a(n_actions, TVector());

        // beta_a_o = argmax_alpha ( alpha * belief_t+1)
        for (number a = 0; a < n_actions; a++)
        {
            for (number o = 0; o < n_obs; o++)
            {
                auto next_belief = beliefMDP->nextState(state, a, o);
                beta_a_o[a][o] = this->getMaxAt(next_belief, t + 1).second;
            }
        }

        // \beta_a = R(s,a) + \gamma * \sum_{o, s'} [ \beta_{a,o}(s') * O(s', a, o) * T(s,a,s') ]
        for (number a = 0; a < n_actions; a++)
        {
            for (number s = 0; s < n_states; s++)
            {
                double tmp = 0;
                for (number o = 0; o < n_obs; o++)
                {
                    for (number s_ = 0; s_ < n_states; s_++)
                    {
                        tmp += beta_a_o[a][o].at(s_) * under_pb->getObsDynamics()->getDynamics(s, a, o, s_);
                    }
                }
                beta_a[a][s] = under_pb->getReward(s, a) + under_pb->getDiscount() * tmp;
            }
        }

        number a_max;
        double current, max_v = -std::numeric_limits<double>::max();
        for (number a = 0; a < n_actions; a++)
        {
            current = state ^ beta_a[a];
            if (current > max_v)
            {
                max_v = current;
                a_max = a;
            }
        }
        auto new_plan = beta_a[a_max];

        return new_plan;
    }

} // namespace sdm