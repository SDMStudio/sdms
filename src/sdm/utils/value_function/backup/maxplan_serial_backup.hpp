
#pragma once

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>

namespace sdm
{
    class MaxPlanSerialBackup : public MaxPlanBackup
    {
    public:
        using TData = std::shared_ptr<State>;

        MaxPlanSerialBackup();
        MaxPlanSerialBackup(const std::shared_ptr<SolvableByHSVI>& );

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        // virtual std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);


        // /**
        //  * @brief Used in the function greedyMaxPlane in order to compute \sum_{x_\tau,o^{-i}} s(x, o) \cdot [r_{x,u} + \gamma \sum_{x_,z_} p_{x,x_,u,z_} \cdot \alpha^{(\kappa)}_{\tau+1}(x_{\tau+1},\langle o_\tau, u_\tau, z_{\tau+1} \rangle)
        //  * 
        //  * @param const TVector& : serialized_occupancy_state
        //  * @param const TVector& : hyperplan
        //  * @param const number : action of an agent 
        //  * @param typename TVector::jhistory_type& : joint history 
        //  * @param number : time step 
        //  * @return double 
        //  */
        // double getMaxPlanValueAt(const TVector&, const TVector& ,const number ,const typename TVector::jhistory_type::element_type::ihistory_type& ,number = 0);

        // /**
        //  * @brief Greedy will determine the best pair <decision rule, value> for a precice serialized occupancy state
        //  * 
        //  * @param state 
        //  * @param t 
        //  * @return Pair<TAction,double> 
        //  */
        // Pair<TAction,double> greedyMaxPlaneSerial(const TVector &,number =0);

        // /**
        //  * @brief backup operator for the serial occupancy state -- type of the state -- 
        //  * @param const TVector & serial occupancy state 
        //  * @param number horizon
        //  * @tparam T 
        //  * @return TVector 
        //  */
        // template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        // TVector backup_operator(const TVector &, number  = 0);

        // /**
        //  * @brief Get the Value At occupancy state, decision rule, and next-step hyperplane
        //  * 
        //  * @param serial_occupancy_state 
        //  * @param indiv_dr 
        //  * @param next_step_hyperplan 
        //  * @param t 
        //  * @return double 
        //  */
        // double getMaxPlanValueAt(const TVector &, const TAction& , const TVector& , number);

        // /**
        //  * @brief Set the Hyperplan occupancy state, decision rule, and next-step hyperplane
        //  * 
        //  * @param serial_occupancy_state 
        //  * @param indiv_dr 
        //  * @param next_step_hyperplan 
        //  * @param t 
        //  * @return TVector 
        //  */
        // TVector setHyperplan(const TVector &, const TAction& , const TVector& , number );
    };
}