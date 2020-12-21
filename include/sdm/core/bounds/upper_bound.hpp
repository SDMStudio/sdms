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

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

    template <typename TX, typename TY>
    struct Point : std::pair<TX, TY>
    {
        TX x;
        TY y;

        Point(TX x, TY y) : std::pair<TX, TY>(x, y) {}
    };

    template <typename TState>
    class UpperBound : public AbstractBound
    {
    protected:
        // std::set<Point<TState, double>> point_set;
        std::unordered_map<TState, double> point_set;

    public:
        void addPoint(TState x, double y);
        void addPoint(const Point<TState, double> &point);

        void deletePoint(TState x, double y);
        void deletePoint(const Point<TState, double> &point);

        double getValueAt(TState x);
        number size();

        void prune();
        void initialize(double value);
    };
} // namespace sdm