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
    double MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::getMaxPlanValueAt(const TVector &serial_occupancy_state,const TVector& hyperplan,const number private_action ,const typename TVector::jhistory_type::element_type::ihistory_type& private_history,number t)
    {
        double value =0;
        auto under_pb = this->getWorld()->getUnderlyingProblem();
        
        for (const auto &pair_hidden_state_history_proba : *serial_occupancy_state.getPrivateOccupancyState(serial_occupancy_state.getCurrentAgentId(),private_history)) 
        {
            auto pair_hidden_state_history = pair_hidden_state_history_proba.first;
            auto serial_hidden_serial_state = serial_occupancy_state.getState(pair_hidden_state_history);

            auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_hidden_state_history);
            auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

            auto proba = pair_hidden_state_history_proba.second;

            value += under_pb->getReward(serial_hidden_serial_state,private_action)*proba; 

            for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(serial_hidden_serial_state, private_action)) 
            {
                for(const auto &serial_observation : under_pb->getReachableObservations(serial_hidden_serial_state,private_action, serialized_underlying_next_state)) 
                {
                    auto joint_history_next = compressed_joint_history->expand(serial_observation);
                    value += proba*under_pb->getDiscount(t)*hyperplan.at(std::make_pair(serialized_underlying_next_state,joint_history_next))*under_pb->getDynamics(serial_hidden_serial_state,private_action,serial_observation,serialized_underlying_next_state);
                }
            }
        }
        return value;
    }
    
    template <typename TVector, typename TAction, typename TValue>
    Pair<TAction,double> MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::greedyMaxPlaneSerial(const TVector &serial_occupancy_state,number t)
    {
        auto soMDP = std::static_pointer_cast<SerializedOccupancyMDP<TVector,TAction>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        double argmax_global = -std::numeric_limits<double>::max();  
        TAction decision_max;

        auto getActionSpace = under_pb->getActionSpace(serial_occupancy_state.getCurrentAgentId())->getAll();

        for(const auto &hyperplan : this->getSupport(t+1))
        {
            double decision_rule_value = 0;  
            TAction decision_;

            for (const auto &pair_s_o_p : *serial_occupancy_state.getFullyUncompressedOccupancy())
            {
                double argmax_local = -std::numeric_limits<double>::max(); 
                auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_s_o_p.first);
                auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

                auto private_compressedd_history = compressed_joint_history->getIndividualHistory(serial_occupancy_state.getCurrentAgentId());

                for (const auto &private_action : getActionSpace)
                {
                    double action_value = this->getMaxPlanValueAt(serial_occupancy_state,hyperplan,private_action,private_compressedd_history,t);
                    if(argmax_local< action_value)
                    {
                        argmax_local = action_value;
                        decision_[private_compressedd_history] = private_action;
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
    TVector MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::backup_operator(const TVector &serial_occupancy_state, number t)
    {
        auto pair_action_value = this->greedyMaxPlaneSerial(serial_occupancy_state,t);

        TVector new_hyperplan(this->default_values_per_horizon[t]);
        new_hyperplan.setAgent(serial_occupancy_state.getCurrentAgentId());

        auto soMDP = std::static_pointer_cast<SerializedOccupancyMDP<TVector,TAction>>(this->getWorld());
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : this->getSupport(t+1))
        {
            for (const auto &uncompressed_s_o : *serial_occupancy_state.getFullyUncompressedOccupancy())
            {
                auto pair_s_o = uncompressed_s_o.first;
                auto uncompressed_hidden_serial_state = serial_occupancy_state.getState(pair_s_o);
                auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_s_o);

                auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

                auto serial_action = pair_action_value.first.act(compressed_joint_history->getIndividualHistory(serial_occupancy_state.getCurrentAgentId()));

                new_hyperplan.addProbabilityAt( pair_s_o,under_pb->getReward(uncompressed_hidden_serial_state,serial_action));

                for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(uncompressed_hidden_serial_state, serial_action))
                {
                    for(const auto &serial_observation :under_pb->getReachableObservations(uncompressed_hidden_serial_state,serial_action, serialized_underlying_next_state))
                    {
                        auto history_next = compressed_joint_history->expand(serial_observation);
                        new_hyperplan.addProbabilityAt(pair_s_o,under_pb->getDiscount(t)*hyperplan.at(std::make_pair(serialized_underlying_next_state,history_next))*under_pb->getDynamics(uncompressed_hidden_serial_state,serial_action,serial_observation,serialized_underlying_next_state));
                    }
                }
            }
        }
        std::cout<<"\n new_hyperplan "<<new_hyperplan.str_hyperplan();
        return new_hyperplan;
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
    {
        std::cout<<"\n size hyperplan t "<<this->representation[t].size();
        auto new_hyperplan = this->backup_operator<TVector>(state, t);

        if(std::find(this->representation[t].begin(), this->representation[t].end(), new_hyperplan) == this->representation[t].end())
            this->representation[t].push_back(new_hyperplan);

        std::cout<<"\n size hyperplan t "<<this->representation[t].size();
    }

} // namespace sdm