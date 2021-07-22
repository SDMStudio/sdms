#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
#include <sdm/utils/linear_programming/variable_naming.hpp>

// #include "../../toulbar2/src/toulbar2lib.hpp"
#include "toulbar2lib.hpp"

#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    class ActionVFSawtoothWCSP : public ActionVFBase, public VarNaming
    {
    public:
        using TData = std::shared_ptr<State>;
        
        ActionVFSawtoothWCSP();
        ActionVFSawtoothWCSP(const std::shared_ptr<SolvableByHSVI>& world);
        
        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        std::shared_ptr<Action> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
    
        // Fonction temporaire le temps de bien comprendre 
        Pair<std::shared_ptr<Action>,double>  createWCSPProblem(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
    
    protected :

        /**
         * @brief defines the maximum value in the domain of the payoff function
         * 
         */
        double max;

        /**
         * @brief factor used to convert real values into integer costs
         * 
         */
        long offset = 1000000000000;

        /**
         * @brief Returns a cost value
         * 
         * @param double defines a real value
         */
        long getCost(double);

        double getValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface>& occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action,const std::shared_ptr<State>& next_hyperplan, number t);

        std::shared_ptr<MappedVector<std::shared_ptr<State>,double>> representation;

        std::unordered_map<std::shared_ptr<State>,std::shared_ptr<Action>> state_linked_to_decision_rule;

        void determineMaxValue(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State>& state, number t);

    };
}
