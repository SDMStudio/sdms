#pragma once

#include <sdm/core/state/state.hpp>

namespace sdm
{
    class HistoryInterface : public State
    {

    public:
        // virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Item> &data, bool backup = true) = 0;
        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true) = 0;
        
        virtual std::shared_ptr<HistoryInterface> getPreviousHistory() = 0;

        // virtual std::shared_ptr<Action> getLastAction() = 0;
        // virtual std::shared_ptr<Observation> getLastObservation() = 0;

        // virtual std::shared_ptr<Action> getActionAt(number t) = 0;
        // virtual std::shared_ptr<Observation> getObservationAt(number t) = 0;

        virtual std::string str() const = 0;
        virtual TypeState getTypeState() const = 0;
    };
}