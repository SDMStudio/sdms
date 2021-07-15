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

namespace sdm
{   
    // Observation from the P.O.V. of the central agent.
    class Observation : public Item
    {
    public:
        virtual ~Observation() {}
        virtual std::string str() const = 0;
    };

    const std::shared_ptr<Observation> DEFAULT_OBSERVATION = nullptr;

    class BeliefInterface;
    class OccupancyStateInterface;
    class HistoryInterface;
    class JointHistoryInterface;
    class SerialInterface;
    class SerialOccupancyInterface;

    class State : public Observation
    {
    public:
        virtual ~State() {}

        /**
         * @brief Transform the State in a BeliefInterface
         * 
         * @return std::shared_ptr<BeliefInterface> 
         */
        virtual std::shared_ptr<BeliefInterface> toBelief();

        /**
         * @brief Transform the State in a OccupancyStateInterface
         * 
         * @return std::shared_ptr<OccupancyStateInterface> 
         */
        virtual std::shared_ptr<OccupancyStateInterface> toOccupancyState();

        /**
         * @brief Transform the State in a HistoryInterface
         * 
         * @return std::shared_ptr<HistoryInterface> 
         */
        virtual std::shared_ptr<HistoryInterface> toHistory();

        /**
         * @brief Transform the State in a Serial Interface
         * 
         * @return std::shared_ptr<SerialInterface> 
         */
        virtual std::shared_ptr<SerialInterface> toSerial();

        /**
         * @brief Transform the State in a Serial Occupancy Interface
         * 
         * @return std::shared_ptr<SerialInterface> 
         */
        virtual std::shared_ptr<SerialOccupancyInterface> toSerialOccupancyState();

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

    using JointHistoryBeliefPair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>;

    using JointHistoryJointActionPair = StatePair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<Action>>;
    
    using PrivateHierarchicalOccupancyStateJointHistoryPair = StatePair<std::shared_ptr<OccupancyStateInterface>, std::shared_ptr<JointHistoryInterface>>;

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