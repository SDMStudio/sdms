/**
 * @file state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for state class
 * @version 0.1
 * @date 11/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/observation/observation.hpp>

namespace sdm
{
    class BeliefInterface;
    class OccupancyStateInterface;
    class HistoryInterface;
    class JointHistoryInterface;
    class BaseSerialInterface;
    class SerialOccupancyInterface;

    /**
     * @brief A public interface for states. 
     * 
     * Any class inheriting from this interface will be considered as generic state for algorithms.
     * Consider sections [ Theoritical Background](/tutorials/theory.html) and [ Algorithms](/tutorials/algorithms/) for more information.   
     *
     */
    class State : public Observation
    {
    public:
        virtual ~State() {}

        /** @brief Cast the state into a belief */
        virtual std::shared_ptr<BeliefInterface> toBelief();

        /** @brief Cast the state into an occupancy state */
        virtual std::shared_ptr<OccupancyStateInterface> toOccupancyState();

        /** @brief Cast the state into a history */
        virtual std::shared_ptr<HistoryInterface> toHistory();

        /** @brief Cast the state into a serial interface */
        virtual std::shared_ptr<BaseSerialInterface> toSerial();

        /**
         * @brief Get the hash of the state. 
         * 
         * The hash is used in tabular value functions in order to compare efficiently two states. 
         * This function must be reimplemented in inherited classes. 
         * 
         * @return size_t the hash code
         */
        virtual size_t hash(double precision = -1) const;

        /**
         * @brief Check equality between two states. 
         * 
         * This function must be implemented in inherited classes. 
         * 
         * @param other the state to be compared to current state
         * @return true if states are equal
         * @return false if they are different
         */
        virtual bool isEqual(const std::shared_ptr<State> &other, double precision = -1) const;

        virtual std::string str() const = 0;

        virtual TypeState getTypeState() const;
    };

    template <typename TItem_1, typename TItem_2, class SuperClass = Item>
    class ItemPair : public SuperClass, public Pair<TItem_1, TItem_2>
    {
    public:
        ItemPair(const TItem_1 &first_item, const TItem_2 &second_item) : Pair<TItem_1, TItem_2>(first_item, second_item) {}
        ItemPair(const Pair<TItem_1, TItem_2> &copy) : Pair<TItem_1, TItem_2>(copy) {}

        std::string str() const
        {
            std::ostringstream res;
            res << "{" << this->first->str() << "," << this->second->str() << "}";
            return res.str();
        }
    };

    template <typename TState_1, typename TState_2>
    using StatePair = ItemPair<TState_1, TState_2, State>;

    template <typename TAction_1, typename TAction_2>
    using ActionPair = ItemPair<TAction_1, TAction_2, Action>;

    using JointHistoryStatePair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<State>>;
    using JointHistoryBeliefPair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>;

    using JointHistoryJointActionPair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<Action>>;

    using PrivateHierarchicalOccupancyStateJointHistoryPair = StatePair<std::shared_ptr<OccupancyStateInterface>, std::shared_ptr<JointHistoryInterface>>;

    // using PrivateHierarchicalOccupancyStateJointHistoryJointActionPair = StatePair<std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryPair>, std::shared_ptr<Action>>;

    // template <typename TItem, typename SuperClass = Item>
    // class JointItem : public SuperClass, public Joint<TItem>
    // {

    // };

    // template <typename TState>
    // using JointState = ItemPair<TState, State>;

    // template <typename TAction>
    // using JointAction = ItemPair<TAction, Action>;

    // template <typename TObservation>
    // using JointObservation = ItemPair<TObservation, Observation>;

} // namespace sdm