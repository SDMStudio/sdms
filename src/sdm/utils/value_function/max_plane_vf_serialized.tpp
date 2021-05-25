#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::MaxPlanValueFunctionSerialized() {}

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer, int freq_prunning)
        : MaxPlanValueFunction<TVector, TAction, TValue>(problem, horizon,initializer,freq_prunning)
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value, int freq_pruning) : MaxPlanValueFunctionSerialized(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value),freq_pruning)
    {
    }

    template <typename TVector, typename TAction, typename TValue>
    double MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::getMaxPlanValueAt(const TVector &serial_occupancy_state,const TVector& hyperplan,const number private_action ,const typename TVector::jhistory_type::element_type::ihistory_type& private_history,number t)
    {
        double value = 0;
        auto agent = serial_occupancy_state.getCurrentAgentId();
        auto under_pb = this->getWorld()->getUnderlyingProblem();
        
        for (const auto &pair_hidden_state_history_proba : *serial_occupancy_state.getPrivateOccupancyState(agent,private_history)) 
        {
            // Get information from private occupancy state
            auto pair_hidden_state_history = pair_hidden_state_history_proba.first;
            auto serial_hidden_serial_state = serial_occupancy_state.getState(pair_hidden_state_history);
            auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_hidden_state_history);
            auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

            double proba = pair_hidden_state_history_proba.second;

            // Determine the reward 
            double immediate_reward = under_pb->getReward(serial_hidden_serial_state, private_action);

            // Compute the next value
            double next_value = 0;

            // Go over all Reachable Serial State
            for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(serial_hidden_serial_state, private_action)) 
            {
                // Go over all Reachable Observation
                for(const auto &serial_observation : under_pb->getReachableObservations(serial_hidden_serial_state,private_action, serialized_underlying_next_state)) 
                {
                    auto joint_history_next = compressed_joint_history->expand(serial_observation);
                    next_value += hyperplan.at(std::make_pair(serialized_underlying_next_state,joint_history_next))*under_pb->getDynamics(serial_hidden_serial_state,private_action,serial_observation,serialized_underlying_next_state);
                }
            }
            value += proba * (immediate_reward + under_pb->getDiscount(t) *next_value);
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

        // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : this->getSupport(t+1))
        {
            decision_rule_value = 0;  
            TAction decision_;

            // Go over all Individual Histories in Serial Occupancy State
            for (const auto &private_history : serial_occupancy_state.getIndividualHistories(agent))
            {
                double argmax_local = -std::numeric_limits<double>::max(); 
                
                // Go over all action possible for the current agent
                for (const auto &private_action : under_pb->getActionSpace(agent)->getAll())
                {
                    double action_value = this->getMaxPlanValueAt(serial_occupancy_state, hyperplan, private_action, private_history, t);
                    
                    // Take the best deterministic decision rule "decision_" for a precise hyperplan
                    if(argmax_local < action_value)
                    {
                        argmax_local = action_value;
                        decision_[private_history] = private_action;
                    }
                }
                decision_rule_value += argmax_local;
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


    template <typename TVector, typename TAction, typename TValue>
    TVector MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::setHyperplan(const TVector &serial_occupancy_state, const TAction& indiv_dr, const TVector& next_step_hyperplan, number t)
    {
        TVector new_hyperplan(this->default_values_per_horizon[t]);
        new_hyperplan.setAgent(serial_occupancy_state.getCurrentAgentId());

        auto under_pb = this->getWorld()->getUnderlyingProblem();

        // Go over all uncompressed serial occupancy state 
        for (const auto &uncompressed_s_o : *serial_occupancy_state.getFullyUncompressedOccupancy())
        {
            // Get information uncompressed serial occupancy state 
            auto pair_s_o = uncompressed_s_o.first;
            auto uncompressed_hidden_serial_state = serial_occupancy_state.getState(pair_s_o);
            auto uncompressed_joint_history = serial_occupancy_state.getHistory(pair_s_o);
            auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 

            // Determine serial action
            auto serial_action = indiv_dr.act(compressed_joint_history->getIndividualHistory(serial_occupancy_state.getCurrentAgentId()));

            // Determine the reward 
            double immediate_reward = under_pb->getReward(uncompressed_hidden_serial_state, serial_action);

            // Compute the next value
            double next_value = 0;

            // Go over all Reachable Serial State
            for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(uncompressed_hidden_serial_state, serial_action))
            {
                // Go over all Reachable Observation
                for(const auto &serial_observation :under_pb->getReachableObservations(uncompressed_hidden_serial_state, serial_action, serialized_underlying_next_state))
                {
                    auto history_next = compressed_joint_history->expand(serial_observation);
                    next_value += next_step_hyperplan.at(std::make_pair(serialized_underlying_next_state, history_next)) * under_pb->getDynamics(uncompressed_hidden_serial_state,serial_action,serial_observation,serialized_underlying_next_state);
                }
            }
            new_hyperplan.setProbabilityAt(pair_s_o,immediate_reward +under_pb->getDiscount(t)*next_value);
        }
        return new_hyperplan;
    }


    template <typename TVector, typename TAction, typename TValue>
    double MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::getMaxPlanValueAt(const TVector &serial_occupancy_state, const TAction& indiv_dr, const TVector& next_step_hyperplan, number t)
    {
        double value = 0;
        auto under_pb = this->getWorld()->getUnderlyingProblem();
        number agent = serial_occupancy_state.getCurrentAgentId();

        // Go over all compressed serial occupancy state 
        for (const auto &compressed_s_o_prob : serial_occupancy_state)
        {
            // Get information uncompressed serial occupancy state 
            auto prob = compressed_s_o_prob.second;
            auto pair_s_o = compressed_s_o_prob.first;
            auto compressed_hidden_serial_state = serial_occupancy_state.getState(pair_s_o);
            auto compressed_joint_history = serial_occupancy_state.getHistory(pair_s_o);
            auto serial_action = indiv_dr.act(compressed_joint_history->getIndividualHistory(agent));
            
            // Determine the reward 
            double immediate_reward = under_pb->getReward(compressed_hidden_serial_state, serial_action);

            // Compute the next value
            double next_value = 0;

            // Go over all Reachable Serial State
            for(const auto &serialized_underlying_next_state: under_pb->getReachableSerialStates(compressed_hidden_serial_state, serial_action))
            {
                // Go over all Reachable Observation
                for(const auto &serial_observation :under_pb->getReachableObservations(compressed_hidden_serial_state, serial_action, serialized_underlying_next_state))
                {
                    auto history_next = compressed_joint_history->expand(serial_observation);
                    next_value += prob*under_pb->getDiscount(t) * next_step_hyperplan.at(std::make_pair(serialized_underlying_next_state, history_next)) * under_pb->getDynamics(compressed_hidden_serial_state,serial_action,serial_observation,serialized_underlying_next_state);
                }
            }
            value += prob * (immediate_reward+ prob * under_pb->getDiscount(t) * next_value);
        }
        return value;
    }

  

    // For SerializedOccupancyMDP (i.e. SerializedOccupancyState as vector type)
    template <typename TVector, typename TAction, typename TValue>
    TVector MaxPlanValueFunctionSerialized<TVector, TAction, TValue>::backup_operator(const TVector &serial_occupancy_state, number t)
    {
        double max = -std::numeric_limits<double>::max(), value; 

        //Determine the determinitic decision rule
        auto pair_action_value = this->greedyMaxPlaneSerial(serial_occupancy_state,t);

        TVector max_next_step_hyperplan;

        // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : this->getSupport(t+1))
        {
            // Determine the max next hyperplan
            if(max < (value = this->getMaxPlanValueAt(serial_occupancy_state, pair_action_value.first, hyperplan, t)))
            {
                max = value;
                max_next_step_hyperplan = hyperplan;
            }
        }
        return this->setHyperplan(serial_occupancy_state, pair_action_value.first, max_next_step_hyperplan, t);
    }

} // namespace sdm