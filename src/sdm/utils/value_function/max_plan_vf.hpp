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
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/initializer.hpp>

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
    class MaxPlanValueFunction : public ValueFunction<TVector, TAction, TValue>
    {
    protected:
        using HyperplanSet = std::set<TVector>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::vector<HyperplanSet> representation;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer<TVector, TAction>> initializer_;

        // std::vector<TValue> default_value_;

    public:
        MaxPlanValueFunction();
        MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer);
        MaxPlanValueFunction(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon = 0, TValue default_value = 0.);

        void initialize();
        void initialize(TValue default_value, int t = 0);

        /**
         * @brief Update the value function by setting the value of a specific state and updating the curves in consequence. 
         * 
         * @param state the state
         */
        void updateValueAt(const TVector &state);

        /**
         * @brief Get the Value at state x 
         * 
         * @param state the state 
         * @return TValue 
         */
        TValue getValueAt(const TVector &state, int t = 0);

        /**
         * @brief Get the maximum value and hyperplan at a specific state
         * 
         * @param state a specific state
         * @return the maximum value and hyperplan at a specific state (std::pair<TValue, TVector>) 
         */
        std::pair<TValue, TVector> getMaxAt(const TVector &state, int t);

        /**
         * @brief Get the number of hyperplans 
         * 
         * @return number 
         */
        number size();

        /**
         * @brief Prune unecessary vectors
         * 
         */
        void prune(int t = 0);

        /*!
         * \fn lark_pruning
         * \brief this method prunes dominated alpha-vectors, known as Lark's pruning.
         * This approach goes other all vectors until all of them have been treated. For each arbitrary vector,
         * it performs a linear programming providing a gap delta, and a frequency f. If the gap is over a certain
         * threshold epsilon, that means one can preserve the selected vector, otherwise one should discard it.
         */
        void lark_prune(int t = 0);

        /*!
         * \fn bounded_pruning
         * \brief this method prunes dominated points, known as bounded pruning by Trey Smith.
         * This approach stores the number of frequency states, among those already visited, that are maximal at a hyperplan.
         * And prune hyperplan with a number of maximal frequency states zero.
         */
        void bounded_prune(int t = 0);

        TVector backup_bellman_operator(std::shared_ptr<POSG> world, TVector belief);

        TAction getBestAction(const TVector &state, int t = 0);
        TValue getQValueAt(const TVector &state, const TAction &action, int t = 0);
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(const TVector &state, int t = 0);
        TValue operator()(const TVector &state);

        void updateValueAt(const TVector &state, int t = 0);

        std::string str() {
            return "MaxPlanVF";
        }
    };
} // namespace sdm
#include <sdm/utils/value_function/max_plan_vf.tpp>
