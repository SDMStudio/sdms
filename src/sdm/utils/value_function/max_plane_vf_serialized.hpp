/**
 * @file max_plan_vf.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 18/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>

#include <sdm/utils/value_function/max_plan_vf.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TVector type of hyperplan representation. Must implement sdm::VectorImpl interface.
     * @tparam TValue value type (default : double)
     */
    template <typename TVector, typename TAction, typename TValue = double>
    class MaxPlanValueFunctionSerialized : public MaxPlanValueFunction<TVector, TAction, TValue>
    {
    protected:
    public:
        MaxPlanValueFunctionSerialized();
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<TVector, TAction>> , int , std::shared_ptr<Initializer<TVector, TAction>> );
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<TVector, TAction>> , int = 0, TValue = 0.);

        std::string str()
        {
            std::ostringstream res;
            res << "<maxplan_value_function_serialized horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
            for (number i = 0; i < this->representation.size(); i++)
            {
                res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << ">" << std::endl;
                for (auto plan : this->representation[i])
                {
                    res << "\t\t<plan>" << std::endl;
                    res << "\t\t\t" << plan << std::endl;
                    res << "\t\t</plan>" << std::endl;
                }
                res << "\t</value>" << std::endl;
            }

            res << "</maxplan_value_function_serialized>" << std::endl;
            return res.str();
        }

        /**
         * @brief Used in the function greedyMaxPlane in order to compute \sum_{x_\tau,o^{-i}} s(x, o) \cdot [r_{x,u} + \gamma \sum_{x_,z_} p_{x,x_,u,z_} \cdot \alpha^{(\kappa)}_{\tau+1}(x_{\tau+1},\langle o_\tau, u_\tau, z_{\tau+1} \rangle)
         * 
         * @param const TVector& : serialized_occupancy_state
         * @param const TVector& : hyperplan
         * @param const number : action of an agent 
         * @param typename TVector::jhistory_type& : joint history 
         * @param number : time step 
         * @return double 
         */
        double getMaxPlanValueAt(const TVector&, const TVector& ,const number ,const typename TVector::jhistory_type::element_type::ihistory_type& ,number = 0);

        /**
         * @brief Greedy will determine the best pair <decision rule, value> for a precice serialized occupancy state
         * 
         * @param state 
         * @param t 
         * @return Pair<TAction,double> 
         */
        Pair<TAction,double> greedyMaxPlaneSerial(const TVector &,number =0);

        /**
         * @brief backup operator for the serial occupancy state -- type of the state -- 
         * @param const TVector & serial occupancy state 
         * @param number horizon
         * @tparam T 
         * @return TVector 
         */
        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        TVector backup_operator(const TVector &, number  = 0);

        /**
         * @brief Get the Value At occupancy state, decision rule, and next-step hyperplane
         * 
         * @param serial_occupancy_state 
         * @param indiv_dr 
         * @param next_step_hyperplan 
         * @param t 
         * @return double 
         */
        double getMaxPlanValueAt(const TVector &, const TAction& , const TVector& , number);

        /**
         * @brief Set the Hyperplan occupancy state, decision rule, and next-step hyperplane
         * 
         * @param serial_occupancy_state 
         * @param indiv_dr 
         * @param next_step_hyperplan 
         * @param t 
         * @return TVector 
         */
        TVector setHyperplan(const TVector &, const TAction& , const TVector& , number );


        void updateValueAt(const TVector &, number );

    };

    template <class TAction, class TValue>
    class MaxPlanValueFunctionSerialized<number, TAction, TValue> : public ValueFunction<number, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<number, TAction>> , int , std::shared_ptr<Initializer<number, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = number.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = number.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = number.");
        }

        TValue getValueAt(const number &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = number.");
        }

        void updateValueAt(const number &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = number.");
        }

        std::vector<number> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = number.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };

    template <class TAction, class TValue>
    class MaxPlanValueFunctionSerialized<SerializedState, TAction, TValue> : public ValueFunction<SerializedState, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>> , int , std::shared_ptr<Initializer<SerializedState, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedState.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedState.");
        }

        TValue getValueAt(const SerializedState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedState.");
        }

        void updateValueAt(const SerializedState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedState.");
        }

        std::vector<SerializedState> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedState.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };


    template <class TAction, class TValue>
    class MaxPlanValueFunctionSerialized<BeliefState, TAction, TValue> : public MaxPlanValueFunction<BeliefState, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<BeliefState, TAction>> , int , std::shared_ptr<Initializer<BeliefState, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        TValue getValueAt(const BeliefState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        void updateValueAt(const BeliefState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        std::vector<BeliefState> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };

    template <class TAction, class TValue>
    class MaxPlanValueFunctionSerialized<SerializedBeliefState, TAction, TValue> : public MaxPlanValueFunction<SerializedBeliefState, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<SerializedBeliefState, TAction>> , int , std::shared_ptr<Initializer<SerializedBeliefState, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedBeliefState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedBeliefState.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedBeliefState.");
        }

        TValue getValueAt(const SerializedBeliefState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedBeliefState.");
        }

        void updateValueAt(const SerializedBeliefState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedBeliefState.");
        }

        std::vector<SerializedBeliefState> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = SerializedBeliefState.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };


    template <class TAction, class TValue>
    class MaxPlanValueFunctionSerialized<OccupancyState<>, TAction, TValue> : public MaxPlanValueFunction<OccupancyState<>, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<OccupancyState<>, TAction>> , int , std::shared_ptr<Initializer<OccupancyState<>, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<>.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<>.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<>.");
        }

        TValue getValueAt(const OccupancyState<> &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<>.");
        }

        void updateValueAt(const OccupancyState<> &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<>.");
        }

        std::vector<OccupancyState<>> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<>.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };

    template <class TAction, class TValue>
    class MaxPlanValueFunctionSerialized<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, TAction, TValue> : public MaxPlanValueFunction<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionSerialized(std::shared_ptr<SolvableByHSVI<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, TAction>> , int , std::shared_ptr<Initializer<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>.");
        }

        TValue getValueAt(const OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>> &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>.");
        }

        void updateValueAt(const OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>> &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>.");
        }

        std::vector<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };

} // namespace sdm
#include <sdm/utils/value_function/max_plane_vf_serialized.tpp>
