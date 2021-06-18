#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/belief_interface.hpp>
#include <sdm/core/state/jhistory_tree_interface.hpp>
#include <sdm/core/state/base_state.hpp>

namespace sdm
{
    class OccupancyStateInterface : public BeliefInterface
    {
    public:

        // Ancienne fonction de Belief Interface 
        // Faudrait que Belief INterface possède des templates, et comme cela, on peut avoir setProbability(const Pair<State*,Obs*>&) / etc
        // Ou alors faire comme pour JOint<State*>, c'est à dire faire une spéciafication de Pair avec Pair qui hérite de state si (State,Obs) 

        virtual std::vector<std::shared_ptr<State>> getStates() const = 0;

        virtual size_t size() const = 0;

        virtual double getProbability(const std::shared_ptr<State> &state) const = 0;

        virtual void setProbability(const std::shared_ptr<State> &state, double proba) = 0;

        virtual void addProbability(const std::shared_ptr<State> &, double proba) = 0;

        virtual bool operator==(const std::shared_ptr<BeliefInterface> &other) const = 0;

        virtual std::string str() const = 0;

        virtual TypeState getTypeState() const =0;

        // Nouvelle fonction 

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