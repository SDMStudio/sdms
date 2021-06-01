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

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam std::shared_ptr<BeliefState> type of hyperplan representation. Must implement sdm::VectorImpl interface.
     * @tparam double value type (default : double)
     */
    class MaxPlanValueFunction : public ValueFunction
    {
    public:
        MaxPlanValueFunction();
        MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI>, number, std::shared_ptr<Initializer>);
        MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI>, number = 0, double = 0.);

        void initialize();
        void initialize(double, number = 0);

        /**
         * @brief Get the Value at state x.
         * 
         * @param state the state 
         * @return double 
         */
        double getValueAt(const std::shared_ptr<BeliefState> &, number = 0);

        /**
         * @brief Update the max plan representation by adding a new hyperplan
         */
        void updateValueAt(const std::shared_ptr<BeliefState> &, number = 0);

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::vector<std::shared_ptr<BeliefState>> getSupport(number);

        /**
         * @brief Get the maximum value and hyperplan at a specific state
         * 
         * @param state a specific state
         * @return the maximum value and hyperplan at a specific state (std::pair<double, std::shared_ptr<BeliefState>>) 
         */
        std::pair<double, std::shared_ptr<BeliefState>> getMaxAt(const std::shared_ptr<BeliefState> &, number);

        /**
         * @brief Prune unecessary vectors
         * 
         */
        void prune(number = 0);

        /*!
         * @brief this method prunes dominated alpha-vectors, known as Lark's pruning.
         * This approach goes other all vectors until all of them have been treated. For each arbitrary vector,
         * it performs a linear programming providing a gap delta, and a frequency f. If the gap is over a certain
         * threshold epsilon, that means one can preserve the selected vector, otherwise one should discard it.
         */
        void lark_prune(number = 0);

        /*!
         * @brief this method prunes dominated points, known as bounded pruning by Trey Smith.
         * This approach stores the number of frequency states, among those already visited, that are maximal at a hyperplan.
         * And prune hyperplan with a number of maximal frequency states zero.
         */
        void bounded_prune(number = 0);

        /**
         * @brief backup operator for the occupancy state with belief representation -- type of the state -- 
         * @param const std::shared_ptr<BeliefState> & occupancy state 
         * @param number horizon
         * @tparam T 
         * @return std::shared_ptr<BeliefState> 
         */
        std::shared_ptr<BeliefState> backup_operator(const std::shared_ptr<BeliefState> &, number = 0);

        /**
         * @brief Get the number of hyperplans 
         * 
         * @return number 
         */
        number size();

        std::string str()
        {
            std::ostringstream res;
            res << "<maxplan_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;

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

            res << "</maxplan_value_function>" << std::endl;
            return res.str();
        }

    protected:
        using HyperplanSet = std::vector<std::shared_ptr<BeliefState>>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::vector<HyperplanSet> representation;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer> initializer_;

        /**
         * @brief the default values, one for each decision epoch.
         */
        std::vector<double> default_values_per_horizon;

        /**
         * @brief Frequency before prunning
         * 
         */
        number freq_prune_;

        /**
         * @brief The last time the prunning took place
         * 
         */
        number last_prunning = 0;

        // template <typename T, std::enable_if_t<std::is_same_v<number, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        // template <typename T, std::enable_if_t<std::is_same_v<SerializedState, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        // template <typename T, std::enable_if_t<std::is_same_v<BeliefState<>, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        // template <typename T, std::enable_if_t<std::is_same_v<BeliefStateGraph_p<number, number>, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        // template <typename T, std::enable_if_t<std::is_same_v<SerializedBeliefState, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        /**
         * @brief Compute the next hyperplan for a precise occupancy_state, hyperplan and a joint decision rule
         * 
         * @tparam T 
         * @tparam std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> 
         * 
         * @param const std::shared_ptr<BeliefState>& : occupancy state
         * @param const std::shared_ptr<BeliefState>& : hyperplan
         * @param const TAction& : joint decision rule
         * @param number : time step
         * 
         * 
         * @return std::shared_ptr<BeliefState> 
         */
        // template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        /**
         * @brief Compute the next hyperplan for a precise occupancy_state, hyperplan and a joint decision rule
         * 
         * @tparam T 
         * @tparam std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> 
         * 
         * @param const std::shared_ptr<BeliefState>& : occupancy state
         * @param const std::shared_ptr<BeliefState>& : hyperplan
         * @param const TAction& : joint decision rule
         * @param number : time step
         * 
         * 
         * @return std::shared_ptr<BeliefState> 
         */
        // template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);

        /**
         * @brief Compute the next hyperplan for a precise serialized_occupancy_state, hyperplan and a joint decision rule
         * 
         * @tparam T 
         * @tparam std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> 
         * 
         * @param const std::shared_ptr<BeliefState>& : serialized_occupancy_state
         * @param const std::shared_ptr<BeliefState>& : hyperplan
         * @param const TAction& : joint decision rule
         * @param number : time step
         * 
         * 
         * @return std::shared_ptr<BeliefState> 
         */
        // template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        // std::shared_ptr<BeliefState> getHyperplanAt(const std::shared_ptr<BeliefState> &, const std::shared_ptr<BeliefState> &, const TAction &, number = 0);
    };

} // namespace sdm
#include <sdm/utils/value_function/max_plan_vf.tpp>
