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
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        double value = 0;
        double tmp; //Used for optimization
        
        // GO over all private Occupancy State
        for (const auto &pair_hidden_state_history_proba : *serial_occupancy_state.getPrivateOccupancyState(serial_occupancy_state.getCurrentAgentId(),private_history)) 
        {
            // Get information from private occupancy state
            auto pair_hidden_state_history = pair_hidden_state_history_proba.first;
            auto serial_hidden_serial_state = serial_occupancy_state.getState(pair_hidden_state_history);
            auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_hidden_state_history);
            auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

            // Add the reward of the hyperplan
            value += under_pb->getReward(serial_hidden_serial_state,private_action); 

            // Compute the value for the precise private occupancy state
            tmp =0;

            // Go over all Reachable Serial State
            for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(serial_hidden_serial_state, private_action)) 
            {
                // Go over all Reachable Observation
                for(const auto &serial_observation : under_pb->getReachableObservations(serial_hidden_serial_state,private_action, serialized_underlying_next_state)) 
                {
                    auto joint_history_next = compressed_joint_history->expand(serial_observation);
                    tmp += hyperplan.at(std::make_pair(serialized_underlying_next_state,joint_history_next))*under_pb->getDynamics(serial_hidden_serial_state,private_action,serial_observation,serialized_underlying_next_state);
                }
            }
            value +=tmp*under_pb->getDiscount(t)*pair_hidden_state_history_proba.second;
        }
        return value;
    }
    
    template <typename TVector, typename TAction, typename TValue>
    Pair<TAction,double> MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::greedyMaxPlaneSerial(const TVector &serial_occupancy_state,number t)
    {
        TAction decision_max;

        auto under_pb = this->getWorld()->getUnderlyingProblem();
        auto agent = serial_occupancy_state.getCurrentAgentId();

        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max();  
        double decision_rule_value;
        double argmax_local;
        double action_value;

        // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : this->getSupport(t+1))
        {
            decision_rule_value = 0;  
            TAction decision_;

            // Go over all Individual Histories in Serial Occupancy State
            for (const auto &private_history : serial_occupancy_state.getIndividualHistories(agent))
            {
                argmax_local = -std::numeric_limits<double>::max(); 
                
                // Go over all action possible for the current agent
                for (const auto &private_action : under_pb->getActionSpace(agent)->getAll())
                {
                    action_value = this->getMaxPlanValueAt(serial_occupancy_state, hyperplan, private_action, private_history, t);
                    
                    // Take the best deterministic decision rule "decision_" for a precise hyperplan
                    if(argmax_local < action_value)
                    {
                        argmax_local = action_value;
                        decision_[private_history] = private_action;
                    }
                }

                double prob = 0;
                /**
                 * @brief compute the probability of private_history given serial_occupancy_state
                 * @warning this probability should be stored in the attribute of any (serial_) occupancy_state
                 */
                for (const auto &pair_hidden_state_history_proba : *serial_occupancy_state.getPrivateOccupancyState(agent, private_history)) 
                {
                    prob += pair_hidden_state_history_proba.second;    
                }     
                std::cout<<"\n proba private history "<<prob;
                std::cout<<"\n Proba over Indiv History"<<serial_occupancy_state.getProbabilityOverIndividualHistories(agent,private_history);

                decision_rule_value += prob*argmax_local;
            }
            // Take the best deterministic decision rule
            if(argmax_global < decision_rule_value)
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
        auto under_pb = this->getWorld()->getUnderlyingProblem();

        double max = -std::numeric_limits<double>::max(), value; 
        auto pair_action_value = this->greedyMaxPlaneSerial(serial_occupancy_state,t);

        TVector max_hyperplan(this->default_values_per_horizon[t]);
        max_hyperplan.setAgent(serial_occupancy_state.getCurrentAgentId());

        // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : this->getSupport(t+1))
        {
            std::cout << "--------------------- new hyperplan -----------------" << std::endl;
            TVector new_hyperplan(this->default_values_per_horizon[t]);
            new_hyperplan.setAgent(serial_occupancy_state.getCurrentAgentId());

            for (const auto &uncompressed_s_o : *serial_occupancy_state.getFullyUncompressedOccupancy())
            {
                auto pair_s_o = uncompressed_s_o.first;
                auto uncompressed_hidden_serial_state = serial_occupancy_state.getState(pair_s_o);
                auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_s_o);

                auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

                auto serial_action = pair_action_value.first.act(compressed_joint_history->getIndividualHistory(serial_occupancy_state.getCurrentAgentId()));

                double immediate_reward = under_pb->getReward(uncompressed_hidden_serial_state, serial_action);
                new_hyperplan.setProbabilityAt(pair_s_o, immediate_reward);

                double next_value = 0;
                for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(uncompressed_hidden_serial_state, serial_action))
                {
                    for(const auto &serial_observation :under_pb->getReachableObservations(uncompressed_hidden_serial_state, serial_action, serialized_underlying_next_state))
                    {
                        auto history_next = compressed_joint_history->expand(serial_observation);
                        next_value += under_pb->getDiscount(t) * hyperplan.at(std::make_pair(serialized_underlying_next_state,history_next)) * under_pb->getDynamics(uncompressed_hidden_serial_state,serial_action,serial_observation,serialized_underlying_next_state);
                    }
                }
                new_hyperplan.addProbabilityAt(pair_s_o, next_value);

                std::cout << "\tpair_s_o : " << pair_s_o << " immediate-reward=" << immediate_reward << " next-expected-value=" << next_value << std::endl;
            }

            if(max < (value = serial_occupancy_state^new_hyperplan))
            {
                max = value;
                max_hyperplan = new_hyperplan;
            }

            std::cout << new_hyperplan.str_hyperplan() << std::endl;
            std::cout << "max : " << max << " value=" << value << std::endl;
        }

        std::cout << "--------------------- max hyperplan -----------------" << std::endl;
        std::cout << max_hyperplan.str_hyperplan() << std::endl;

        // debugging 
        if( std::abs(pair_action_value.second - (value = serial_occupancy_state^max_hyperplan)) > 0.01 )
        {
            throw sdm::exception::Exception("backed up value " + std::to_string(pair_action_value.second) + " differs from computed value " +std::to_string(value) + " max=" + std::to_string(max));
        }

        return max_hyperplan;
    }

    template <typename TVector, typename TAction, typename TValue>
    void MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
    {
        auto new_hyperplan = this->backup_operator<TVector>(state, t);

        if(std::find(this->representation[t].begin(), this->representation[t].end(), new_hyperplan) == this->representation[t].end())
            this->representation[t].push_back(new_hyperplan);
    }

} // namespace sdm