/**
 * @file abstract_bound.hpp
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
#include <sdm/core/bounds/abstract_bound.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <typename TState>
    class LowerBound : public AbstractBound
    {
    protected:
        // std::set<Vector> vector_set;
        /**
         * @brief Vector set represented as a standard mapping from State to Hyperplan
         * 
         */
        RecursiveMap<TState, Vector> vector_set;

    public:
        LowerBound(std::shared_ptr<POSG> problem);

        /**
         * @brief Set the Value at a specific state
         * 
         * @param state the state
         * @param value the value of this state
         */
        void setValueAt(TState state, Vector value);

        /**
         * @brief Update the value function by setting the value of a specific state and updating the curves in consequence. 
         * 
         * @param state the state
         */
        void updateValueAt(TState &);

        /**
         * @brief Get the Value at state x 
         * 
         * @param x 
         * @return double 
         */
        double getValueAt(TState x);

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
        void prune();

        /*!
         * \fn lark_pruning
         * \brief this method prunes dominated alpha-vectors, known as Lark's pruning.
         * This approach goes other all vectors until all of them have been treated. For each arbitrary vector,
         * it performs a linear programming providing a gap delta, and a frequency f. If the gap is over a certain
         * threshold epsilon, that means one can preserve the selected vector, otherwise one should discard it.
         */
        void lark_prune();

        void initialize(double value);
    };
} // namespace sdm