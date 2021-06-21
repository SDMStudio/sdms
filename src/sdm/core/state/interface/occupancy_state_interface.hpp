#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/jhistory_tree_interface.hpp>
#include <sdm/core/state/base_state.hpp>

namespace sdm
{
    class OccupancyStateInterface : public BeliefInterface
    {
    public:

        virtual const std::vector<std::set<std::shared_ptr<HistoryTreeInterface>>> &getAllIndividualHistories() const =0;
        virtual const std::set<std::shared_ptr<State>> &getStatesAt(const std::shared_ptr<JointHistoryTreeInterface> &jhistory) const =0;
        virtual const std::set<std::shared_ptr<HistoryTreeInterface>> &getIndividualHistories(number ag_id) const = 0;
        virtual std::shared_ptr<State> getHiddenState(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>> &) const = 0;
        virtual std::shared_ptr<JointHistoryTreeInterface> getHistory(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>> &) const =0;
        virtual void finalize() = 0;
        virtual std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy() const = 0;
        virtual void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;
        virtual std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy() const = 0;
        virtual void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;
        virtual std::vector<std::shared_ptr<HistoryTreeInterface>> getJointLabels(const std::vector<std::shared_ptr<HistoryTreeInterface>> &) const =0;
        virtual const std::set<std::shared_ptr<JointHistoryTreeInterface>> &getJointHistories() const = 0;
        virtual const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryTreeInterface>&) const = 0 ;
        virtual void updateJointLabels(const std::vector<std::shared_ptr<HistoryTreeInterface>> &, const std::vector<std::shared_ptr<HistoryTreeInterface>> &) =0;
        virtual std::shared_ptr<JointHistoryTreeInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryTreeInterface> &) const = 0;
        virtual const double &getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryTreeInterface>&) const = 0;
        virtual std::shared_ptr<OccupancyStateInterface> compress() = 0;

        virtual std::shared_ptr<State> HiddenStateAndJointHistoryToState(const std::shared_ptr<State>&, const std::shared_ptr<JointHistoryTreeInterface>&)const =0;
        virtual const std::shared_ptr<BeliefInterface> createBelief(const std::shared_ptr<JointHistoryTreeInterface> &joint_history) const =0;
        virtual const std::shared_ptr<BeliefInterface> createBeliefWeighted(const std::shared_ptr<JointHistoryTreeInterface> &joint_history) const = 0;

        // virtual void setAllIndividualHistories() = 0;
        // virtual void setJointHistories() =0 ;
        // virtual void setStates()=0;
        // virtual const std::unordered_set<std::shared_ptr<JointHistoryTreeInterface>> &getJointHistoryOverIndividualHistories(number, std::shared_ptr<HistoryTreeInterface>) const = 0;
        // virtual void setJointHistoryOverIndividualHistories() = 0;
        // virtual void setProbabilityOverJointHistory() = 0;
        // virtual bool areIndividualHistoryLPE(const std::shared_ptr<HistoryTreeInterface> &, const std::shared_ptr<HistoryTreeInterface> &, number) =0;
        // virtual const Joint<RecursiveMap<std::shared_ptr<HistoryTreeInterface>, std::shared_ptr<PrivateOccupancyState>>> &getPrivateOccupancyStates() const = 0;
        // virtual const std::shared_ptr<PrivateOccupancyState> &getPrivateOccupancyState(const number &agent_id, const HistoryTreeInterface &ihistory) const;
        // virtual void setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;
        // virtual std::shared_ptr<OccupancyStateInterface> getCompressedOccupancy() const = 0;
        // virtual std::shared_ptr<HistoryTreeInterface> getLabel(const std::shared_ptr<HistoryTreeInterface> &ihistory, number agent_id) const = 0;
        // virtual void updateLabel(number, const std::shared_ptr<HistoryTreeInterface> &, const std::shared_ptr<HistoryTreeInterface> &) =0 ;
        // virtual void setProbabilityOverIndividualHistories();
    };
}