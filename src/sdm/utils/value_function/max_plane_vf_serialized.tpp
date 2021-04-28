#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::MaxPlanValueFunctionSerialized() {}

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer)
        : MaxPlanValueFunction<TVector, TAction, TValue>(problem, horizon,initializer)
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value) : MaxPlanValueFunctionSerialized(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value))
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    double MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::getValueAt(const TVector &serialized_occupancy_state,const TVector& hyperplan,const number private_action ,const typename TVector::jhistory_type::element_type::ihistory_type& private_history,number t)
    {
        double value =0;

        for (const auto &pair_hidden_state_history_proba : serialized_occupancy_state.getPrivateOccupancyState(serialized_occupancy_state.getCurrentAgentId(),private_history)) 
        {
            auto pair_hidden_state_history = pair_hidden_state_history_proba.first;
            auto serial_hidden_state = serialized_occupancy_state.getState(pair_hidden_state_history);
            auto joint_history = serialized_occupancy_state.getHistory(pair_hidden_state_history);
            auto proba = pair_hidden_state_history_proba.second;

            value += under_pb->getReward(serial_hidden_state,private_action)*proba; 

            for(const auto &serialized_underlying_next_state: under_pb->getReacheableStates(serial_hidden_state, private_action)) 
            {
                for(const auto &serial_observation : under_pb->getReacheableObservations(private_action, serialized_underlying_next_state)) 
                {
                    auto joint_history_next = joint_history->expand(serial_observation);
                    value += proba*under_pb->getDiscount(t)*hyperplan.at(std::make_pair(serialized_underlying_next_state,history_2_next))*under_pb->getObsDynamics(serial_hidden_state,private_action,serial_observation,serialized_underlying_next_state);
                }
            }
        }
        return value
    }
    
    template <typename TVector, typename TAction, typename TValue>
    Pair<TAction,double> MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::greedyMaxPlane(const TVector &serialized_occupancy_state,number t)
    {
        auto soMDP = std::static_pointer_cast<SerializedOccupancyMDP<TVector,TAction>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        double argmax_global = -std::numeric_limits<double>::max();  
        TAction decision_max;

        for(const auto &hyperplan : this->getSupport(t+1))
        {
            double decision_rule_value = 0;  
            TAction decision_;

            for (const auto &pair_s_o_p : serialized_occupancy_state)
            {
                double argmax_local = -std::numeric_limits<double>::max(); 

                auto private_history = serialized_occupancy_state.getHistory(pair_s_o_p.first)->getIndividualHistory(serialized_occupancy_state.getCurrentAgentId());

                for (const auto &private_action : under_pb->getActionSpaceAt(serialized_occupancy_state.getCurrentAgentId())->getAll())
                {
                    double action_value = this->computeGreedyMaxPlane(serialized_occupancy_state,hyperplan,private_action,private_history,t);

                    if(argmax_local< action_value)
                    {
                        argmax_local = action_value;
                        decision_.container_[private_history] = private_action;
                    }
                }
                decision_rule_value += argmax_local;
            }
            if(argmax_global<decision_rule_value)
            {
                argmax_global = decision_rule_value;
                decision_max = decision_;
            }
        }
        return std::make_pair(decision_max,argmax_global);
    }   

    // For SerializedOccupancyMDP (i.e. SerializedOccupancyState as vector type)
    template <typename TVector, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    TVector MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::backup_operator(const TVector &serialized_occupancy_state, number t)
    {
        auto pair_action_value = this->greedyMaxPlane(serialized_occupancy_state,t);

        TVector hyperplan(this->default_value_[t]);

        auto soMDP = std::static_pointer_cast<SerializedOccupancyMDP<TVector,TAction>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        for(const auto &plan : this->getSupport(t+1))
        {
            for (const auto &pair_s_o_p : serialized_occupancy_state)
            {
                auto pair_s_o = pair_s_o_p.first;
                auto serialized_state = serialized_occupancy_state.getState(pair_s_o);
                auto history = serialized_occupancy_state.getHistory(pair_s_o);

                auto private_action = pair_action_value.first.act(history->getIndividualHistory(serialized_occupancy_state.getCurrentAgentId()));

                hyperplan[pair_s_o] = under_pb->getReward(serialized_state,private_action);

                for(const auto &serialized_underlying_next_state: under_pb->getReacheableStates(serialized_state, private_action))
                {
                    for(const auto &serial_observation :under_pb->getReacheableObservations(private_action, serialized_state))
                    {
                        auto history_next = history->expand(serial_observation);
                        hyperplan[pair_s_o] += under_pb->getDiscount(t)*plan.at(std::make_pair(serialized_underlying_next_state,history_next))*under_pb->getObsDynamics(s_state,private_action,serial_observation,serialized_underlying_next_state);
                    }
                }
            }
        }
        return hyperplan;
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
    {
        auto new_hyperplan = this->backup_operator<TVector>(state, t);

        if(std::find(this->representation[t].begin(), this->representation[t].end(), new_hyperplan) == this->representation[t].end())
            this->representation[t].push_back(new_hyperplan);
    }

} // namespace sdm