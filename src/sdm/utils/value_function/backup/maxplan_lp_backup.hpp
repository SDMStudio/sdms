#pragma once

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/decentralized_constraints_with_lp.hpp>

namespace sdm
{
    class MaxPlanLPBackup : public MaxPlanBackup, DecentralizedConstraintsLP
    {
    public:
        using TData = std::shared_ptr<State>;

        MaxPlanLPBackup();
        MaxPlanLPBackup(const std::shared_ptr<SolvableByHSVI>& );

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

    protected : 
        Pair<std::shared_ptr<State>,std::shared_ptr<Action>> getBestActionAndMaxHyperplan(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        
        /**
        * @brief Returns the greedy decision rule for the current occupancy state and the prescribed hyperplan
        * Maximize \sum_{x,o,u} A(u|o) s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \alpha(x_,o_)  ]
        * Subject to:
        *       A(u|o) <= A_i(u_i|o_i)
        *       A(u|o) >= \sum_i A_i(u_i|o_i) + 1 - n
        *       \sum_{u_i} A_i(u_i|o_i) = 1
        * 
        * @param const std::shared_ptr<OccupancyStateInterface>& : current occupancy state
        * @param const std::shared_ptr<OccupancyStateInterface>& : hyperplan to use
        * @param number : time step
        * 
        * @return Pair<std::shared_ptr<Action>,double> : action and the value associated 
        */
        Pair<std::shared_ptr<Action>,double> getGreedy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        /**
         * @brief Set the variable used in greedyMaxPlane
         * 
         * @param const TVector&
         * @param IloEnv& 
         * @param IloNumVarArray&
         */
        void setGreedyVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t);

        /**
         * @brief Set coefficient of variable a(u|o)  for all u and o i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \alpha_i(x_,o_)  ] 
         * 
         * @param const std::shared_ptr<OccupancyStateInterface>& : current occupancy state
         * @param IloNumVarArray&
         * @param IloObjective&
         * @param const std::shared_ptr<OccupancyStateInterface>& : hyperplan to use
         * @param number : time steps
         * 
         */
        void setGreedyObjective(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

    };
}