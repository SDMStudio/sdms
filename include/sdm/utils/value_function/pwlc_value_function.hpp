/**
 * @file pwlc_value_function.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Defines the piece-wise linear convex value function.
 * @version 0.1
 * @date 16/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <map>
#include <sdm/utils/struct/recursive_map.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

    enum PruningType
    {
        CLASSIC,
        LARK,
        BOUNDED
    };

    enum BoundType
    {
        UPPER,
        LOWER
    };

    /**
     * @class PWLCValueFunction 
     *
     * @brief 
     * 
     */
    template <typename TState, typename TAction>
    struct PWLCValueFunction : ValueFunction<TState, double>
    {
    protected:
        PruningType pruning_type_ = PruningType::CLASSIC;
        BoundType bound_type_;

        std::vector<UpperBound<TState, TAction>> upper_bound;
        std::vector<LowerBound<TState, TAction>> lower_bound; 

        /**
         * @brief Set the Value at a specific state
         * 
         * @param state the state
         * @param value the value of this state
         */
        void setValueAt(TState state, double value);

        void prune();

        void classic_pruning();
        void bounded_pruning();
        void lark_pruning();

    public:
        PWLCValueFunction(BoundType bound_type, PruningType pruning_type = PruningType::CLASSIC);

        /**
         * @brief initialize the value function
         * 
         */
        void initialize();

        number size() const;

        /*!
         * @fn getGreedyAt
         * @brief this method selects the greedy belief decision rule at a given belief occupancy state.
         * @param belief describes a belief occupancy state.
         * @param dr is a belief greedy decision rule at a given belief occupancy state.
         * @param value is the vector value associated with the current belief occupancy state.
         * @return double
         */
        double getGreedyValueAt(TState &state, TAction &dr, std::shared_ptr<Vector> &value) const;

        double getValueAt(TState state) const;


        /**
         * @brief Update the value function by setting the value of a specific state and updating the curves in consequence. 
         * 
         * @param state the state
         */
        void updateValueAt(TState &);


        std::shared_ptr<Vector> getMinimumAlphaVector(const std::shared_ptr<Vector> &, const std::shared_ptr<Vector> &);

        std::pair<std::shared_ptr<Vector>, double> checkDominance(const std::shared_ptr<Vector> &, const std::unordered_set<std::shared_ptr<Vector>> &);

    };
} // namespace sdm