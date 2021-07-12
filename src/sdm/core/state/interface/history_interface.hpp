#pragma once

#include <sdm/core/state/state.hpp>

namespace sdm
{
    class JointHistoryInterface;

    class HistoryInterface : public State
    {

    public:
        // virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Item> &data, bool backup = true) = 0;
        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, bool backup = true) = 0;

        virtual std::shared_ptr<HistoryInterface> getPreviousHistory() = 0;

        virtual std::shared_ptr<Observation> getObservation() = 0;

        // virtual std::shared_ptr<Action> getLastAction() = 0;
        // virtual std::shared_ptr<Observation> getObservation() = 0;

        // virtual std::shared_ptr<Action> getActionAt(number t) = 0;
        // virtual std::shared_ptr<Observation> getObservationAt(number t) = 0;
        virtual std::string str() const = 0;
        virtual std::string short_str() const = 0;

        virtual TypeState getTypeState() const { return TypeState::STATE; }


        /**
         * @brief Transform the HistoryInterface in a JointHistoryInterface
         * 
         * @return std::shared_ptr<JointHistoryInterface> 
         */
        std::shared_ptr<JointHistoryInterface> toJointHistory();
    };
}