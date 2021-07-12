#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    class JointHistoryInterface : virtual public HistoryInterface
    {

    public:
        virtual std::string str() const = 0;

        virtual std::shared_ptr<HistoryInterface> getIndividualHistory(number agent_id) const = 0;
        virtual Joint<std::shared_ptr<HistoryInterface>> getIndividualHistories() const = 0;

        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &obs, bool backup = true) = 0;    
        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Joint<std::shared_ptr<Observation>>> &, bool = true) = 0;

        virtual void addIndividualHistory(std::shared_ptr<HistoryInterface> ihist) = 0;
        // virtual number getNumAgents() const;
    };
}