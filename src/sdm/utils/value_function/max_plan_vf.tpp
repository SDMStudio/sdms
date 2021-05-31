
#include <sdm/utils/value_function/value_function.hpp>
namespace sdm
{

    
    MaxPlanValueFunction::MaxPlanValueFunction() {}

    
    MaxPlanValueFunction::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, std::shared_ptr<Initializer> initializer)
        : ValueFunction<std::shared_ptr<BeliefState>, TAction, TValue>(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<HyperplanSet>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, HyperplanSet({}));
        this->default_values_per_horizon = std::vector<TValue>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, 0);
    }

    
    MaxPlanValueFunction::MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, TValue default_value) : MaxPlanValueFunction(problem, horizon, std::make_shared<ValueInitializer<std::shared_ptr<BeliefState>, TAction>>(default_value))
    {
    }

    void MaxPlanValueFunction::initialize(TValue value, number t)
    {
        std::shared_ptr<BeliefState> new_v(value);
        this->representation[t].push_back(new_v);
        this->default_values_per_horizon[t] = value;
    }

    
    void MaxPlanValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    
    std::pair<TValue, std::shared_ptr<BeliefState>> MaxPlanValueFunction::getMaxAt(const std::shared_ptr<BeliefState> &state, number t)
    {
        TValue current, max = -std::numeric_limits<TValue>::max();
        std::shared_ptr<BeliefState> alpha_vector;

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

    
    TValue MaxPlanValueFunction::getValueAt(const std::shared_ptr<BeliefState> &state, number t)
    {
        return this->getMaxAt(state, t).first;
    }

    
    void MaxPlanValueFunction::updateValueAt(const std::shared_ptr<BeliefState> &state, number t)
    {
        //std::cout<<"\n support ::::::"<<this->representation;
        const auto &new_hyperplan = this->backup_operator(state, t);

        if (std::find(this->representation[t].begin(), this->representation[t].end(), new_hyperplan) == this->representation[t].end())
            this->representation[t].push_back(new_hyperplan);

        if (this->last_prunning == this->freq_prune_)
        {
            for (number time = 0; time < this->getHorizon(); time++)
            {
                this->prune(time);
            }
            this->last_prunning = 0;
        }
        this->last_prunning++;
    }

    
    std::vector<std::shared_ptr<BeliefState>> MaxPlanValueFunction::getSupport(number t)
    {
        return this->representation[t];
    }

    
    void MaxPlanValueFunction::prune(number t)
    {
        this->bounded_prune(t);
    }

    
    void MaxPlanValueFunction::bounded_prune(number t)
    {
        std::unordered_map<std::shared_ptr<BeliefState>, number> refCount;
        auto all_plan = this->isInfiniteHorizon() ? this->representation[0] : this->representation[t];

        // Initialize ref count to 0 for each hyperplan
        for (auto iter = all_plan.begin(); iter != all_plan.end(); iter++)
        {
            refCount.emplace(*iter, 0);
        }

        //<! update the count
        std::shared_ptr<BeliefState> max_alpha;
        TValue max_value = -std::numeric_limits<TValue>::max(), value;
        for (const auto &hyperplan : all_plan)
        {
            for (const auto &alpha : refCount)
            {
                if (max_value < (value = (hyperplan) ^ (alpha.first)))
                {
                    max_value = value;
                    max_alpha = alpha.first;
                }
            }

            if (refCount.find(max_alpha) != refCount.end())
            {
                refCount.at(max_alpha)++;
            }
        }

        for (auto iter = all_plan.begin(); iter != all_plan.end(); iter++)
        {
            if (refCount.at(*iter) == 0)
            {
                this->representation[t].erase(std::find(this->representation[t].begin(), this->representation[t].end(), *iter));
            }
        }
    }

    
    number MaxPlanValueFunction::size()
    {
        return this->representation.size();
    }

    
    std::shared_ptr<BeliefState> MaxPlanValueFunction::backup_operator(const std::shared_ptr<BeliefState> &occupancy_state, number t)
    {
        std::shared_ptr<BeliefState> v_max;
        double value_max = -std::numeric_limits<double>::max(), tmp;

        // Go over the hyperplanes of decision step t+1
        for (const auto &next_hyperplan : this->getSupport(t + 1))
        {
            // Go over all joint decision rules at occupancy occupancy_state
            for (const auto &joint_decision_rule : this->getWorld()->getActionSpaceAt(occupancy_state)->getAll())
            {
                const std::shared_ptr<BeliefState> &new_hyperplan = this->getHyperplanAt<std::shared_ptr<BeliefState>>(occupancy_state, next_hyperplan, joint_decision_rule, t);
                if (value_max < (tmp = occupancy_state ^ new_hyperplan))
                {
                    value_max = tmp;
                    v_max = new_hyperplan;
                }
            }
        }
        return v_max;
    }

    // ---------------------------------------------------------------
    // ----- DEFINITION FOR OccupancyMDP with Belief FORMALISM -------
    // ---------------------------------------------------------------

    
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &occupancy_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &joint_decision_rule, number t)
    {
        std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        // Go over all occupancy state
        for (const auto &uncompressed_belief_history_proba : *occupancy_state.getFullyUncompressedOccupancy())
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

    // ---------------------------------------------------------------
    // --------- DEFINITION FOR OccupancyMDP FORMALISM ---------------
    // ---------------------------------------------------------------

    
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &occupancy_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &joint_decision_rule, number t)
    {
        std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        // Go over all occupancy state
        for (const auto &uncompressed_s_o : *occupancy_state.getFullyUncompressedOccupancy())
        {
            //Get information from uncompressed_s_o
            auto uncompressed_hidden_state = uncompressed_s_o.first.first;
            auto uncompressed_joint_history = uncompressed_s_o.first.second;
            auto compressed_joint_history = occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

            // Get the serial action from the serial_decision_rule
            auto action = joint_decision_rule.act(compressed_joint_history->getIndividualHistories());

            // Add the reward of the hyperplan
            new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getReward()->getReward(uncompressed_hidden_state, under_pb->getActionSpace()->joint2single(action)));

            //Go ober all Reachable State
            for (const auto &next_hidden_state : under_pb->getReachableStates(uncompressed_hidden_state, action))
            {
                //Go ober all Reachable Observation
                for (const auto &next_observation : under_pb->getReachableObservations(uncompressed_hidden_state, action, next_hidden_state))
                {
                    auto next_joint_history = compressed_joint_history->expand(next_observation);
                    new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getDiscount() * under_pb->getObsDynamics()->getDynamics(uncompressed_hidden_state, under_pb->getActionSpace()->joint2single(action), under_pb->getObsSpace()->joint2single(next_observation), next_hidden_state) * next_hyperplan.at({next_hidden_state, next_joint_history}));
                }
            }
        }
        return new_hyperplan;
    }

    // ---------------------------------------------------------------
    // ------- DEFINITION FOR SerializedOccupancyMDP FORMALISM -------
    // ---------------------------------------------------------------

    
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &serial_occupancy_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &serial_decision_rule, number t)
    {
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
        number ag_id = serial_occupancy_state.getCurrentAgentId();
        new_hyperplan.setAgent(ag_id);

        // Go over all serial occupancy state
        for (const auto &uncompressed_s_o : *serial_occupancy_state.getFullyUncompressedOccupancy())
        {
            //Get information from uncompressed_s_o
            auto uncompressed_hidden_serial_state = serial_occupancy_state.getState(uncompressed_s_o.first);
            auto uncompressed_joint_history = serial_occupancy_state.getHistory(uncompressed_s_o.first);
            auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

            // Get the serial action from the serial_decision_rule
            auto serial_action = serial_decision_rule.act(compressed_joint_history->getIndividualHistory(ag_id));

            // Add the reward of the hyperplan
            new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getReward(uncompressed_hidden_serial_state, serial_action));

            // Go over all Reachable Serial State
            for (const auto &next_hidden_serial_state : under_pb->getReachableSerialStates(uncompressed_hidden_serial_state, serial_action))
            {
                // Go over all Reachable Observation
                for (const auto &next_serial_observation : under_pb->getReachableObservations(uncompressed_hidden_serial_state, serial_action, next_hidden_serial_state))
                {
                    auto next_joint_history = compressed_joint_history->expand(next_serial_observation);
                    new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getDiscount(t) * under_pb->getDynamics(uncompressed_hidden_serial_state, serial_action, next_serial_observation, next_hidden_serial_state) * next_hyperplan.at({next_hidden_serial_state, next_joint_history}));
                }
            }
        }
        return new_hyperplan;
    }

    // ---------------------------------------------------------------
    // --------- DEFINITION FOR BeliefMDP FORMALISM ------------------
    // ---------------------------------------------------------------

    // 
    // template <typename T, std::enable_if_t<std::is_same_v<BeliefState, T>, int>>
    // std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &belief_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &action, number t)
    // {
    //     std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
    //     auto underlying_problem = this->getWorld()->getUnderlyingProblem();

    //     // Go over all occupancy state
    //     for (const auto &pair_state_proba : belief_state)
    //     {
    //         //Get information from pair_state_proba
    //         auto state = pair_state_proba.first;

    //         // Add the reward of the hyperplan
    //         new_hyperplan.addProbabilityAt(pair_state_proba.first, underlying_problem->getReward()->getReward(state, action));

    //         //Go over all Reachable State
    //         for (const auto &next_state : underlying_problem->getReachableStates(state, action))
    //         {
    //             //Go over all Reachable Observation
    //             for (auto next_observation : underlying_problem->getReachableObservations(state, action, next_state))
    //             {
    //                 new_hyperplan.addProbabilityAt(pair_state_proba.first, underlying_problem->getDiscount() * underlying_problem->getObsDynamics()->getDynamics(state, action, next_observation, next_state) * next_hyperplan.at(next_state));
    //             }
    //         }
    //     }
    //     return new_hyperplan;
    // }

    template <>
    BeliefState<> MaxPlanValueFunction<BeliefState<>, number, double>::backup_operator(const BeliefState<> &state, number t)
    {
        std::cout << "in backup"<<std::endl;
        auto beliefMDP = std::static_pointer_cast<BeliefMDP<BeliefState<>, number, number>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        number n_obs = under_pb->getObsSpace()->getNumItems();
        number n_actions = under_pb->getActionSpace()->getNumItems();
        number n_states = under_pb->getStateSpace()->getNumItems();

        std::vector<std::vector<BeliefState<>>> beta_a_o(n_actions, std::vector<BeliefState<>>(n_obs, BeliefState<>(n_states)));
        std::vector<BeliefState<>> beta_a(n_actions, BeliefState<>(n_states));

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
            current = state ^ beta_a.at(a);
            if (current > max_v)
            {
                max_v = current;
                a_max = a;
            }
        }
        auto new_plan = beta_a[a_max];

        return new_plan;
    }

    // ---------------------------------------------------------------
    // --------- DEFINITION FOR BeliefMDP FORMALISM ------------------
    // ---------------------------------------------------------------

    template <>
    std::pair<double, SerializedBeliefState> MaxPlanValueFunction<SerializedBeliefState, number, double>::getMaxAt(const SerializedBeliefState &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    
    template <typename T, std::enable_if_t<std::is_same_v<SerializedBeliefState, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    // ---------------------------------------------------------------
    // --------- DEFINITION FOR MDP FORMALISM ------------------------
    // ---------------------------------------------------------------

    template <>
    std::pair<double, number> MaxPlanValueFunction<number, number, double>::getMaxAt(const number &, number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
    }

    template <>
    std::pair<double, SerializedState> MaxPlanValueFunction<SerializedState, number, double>::getMaxAt(const SerializedState &, number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
    }

    template <>
    SerializedState MaxPlanValueFunction<SerializedState, number, double>::backup_operator(const SerializedState &, number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
    }

    
    template <typename T, std::enable_if_t<std::is_same_v<SerializedState, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
    }

    
    template <typename T, std::enable_if_t<std::is_same_v<BeliefStateGraph_p<number, number>, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = BeliefStateGraph_p<number, number>.");
    }

    
    template <typename T, std::enable_if_t<std::is_same_v<number, T>, int>>
    std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = number.");
    }

    template <>
    void MaxPlanValueFunction<SerializedState, number, double>::bounded_prune(number t)
    {
        throw sdm::exception::Exception("MaxPlanVF cannot be used for State = SerializedState.");
    }

} // namespace sdm