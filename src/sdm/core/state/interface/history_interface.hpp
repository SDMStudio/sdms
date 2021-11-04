#pragma once

#include <sdm/core/state/state.hpp>

namespace sdm
{
    class JointHistoryInterface;

    /**
     * @brief A common interface for objects that represent a history.
     * 
     */
    class HistoryInterface : public State
    {

    public:
        /**
         * @brief Expands the history. 
         * 
         * @param observation the observation of the expanded node
         * @return the expanded history
         * 
         * We consider that histories can expand next history from themselves and the next observation. 
         * 
         */
        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true) = 0;

        /**
         * @brief Get the previous history. 
         * 
         * @return the one step behind history.
         */
        virtual std::shared_ptr<HistoryInterface> getPreviousHistory() = 0;

        /**
         * @brief Get the last observation. 
         * 
         * @return the last observation 
         */
        virtual std::shared_ptr<Observation> getLastObservation() = 0;

        /**
         * @brief Get the horizon of the history.
         * 
         * @return number the history
         */
        virtual number getHorizon() const = 0;

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