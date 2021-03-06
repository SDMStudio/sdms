#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    /**
     * @brief A common interface for objects that represent a joint history.
     *
     */
    class JointHistoryInterface : virtual public HistoryInterface    {
    public:
        virtual number getNumAgents() const = 0;

        virtual std::shared_ptr<HistoryInterface> getIndividualHistory(number agent_id) const = 0;

        virtual Joint<std::shared_ptr<HistoryInterface>> getIndividualHistories() const = 0;

        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true) = 0;

        virtual std::shared_ptr<JointHistoryInterface> expand(const std::shared_ptr<JointObservation> &joint_observation, const std::shared_ptr<JointAction> &joint_action = nullptr, bool = true) = 0;

        virtual void addIndividualHistory(std::shared_ptr<HistoryInterface> ihist) = 0;

        virtual std::shared_ptr<JointObservation> getDefaultObs() = 0;

        virtual std::string str() const = 0;
    };
}