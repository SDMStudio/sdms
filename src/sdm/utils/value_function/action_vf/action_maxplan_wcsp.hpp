#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
#include "../../toulbar2/src/toulbar2lib.hpp"
#include <sdm/utils/value_function/variable_naming.hpp>


namespace sdm
{
    class ActionVFMaxplanWCSP : public ActionVFBase, public VarNaming
    {
    public:
        using TData = std::shared_ptr<State>;
        
        ActionVFMaxplanWCSP();
        ActionVFMaxplanWCSP(const std::shared_ptr<SolvableByHSVI>& world);
        
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
        void createWCSPProblem(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
    
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
        int getCost(double);
    };
}
