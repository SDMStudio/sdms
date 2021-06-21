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

namespace sdm
{
    class Observation : public Item
    {
    public:
        virtual ~Observation() {}
        virtual std::string str() const = 0;
    };

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
        virtual std::shared_ptr<HistoryInterface> toHistoryTree();

        /**
         * @brief Transform the State in a JointHistoryInterface
         * 
         * @return std::shared_ptr<JointHistoryInterface> 
         */
        virtual std::shared_ptr<JointHistoryInterface> toJointHistoryTree();

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
} // namespace sdm