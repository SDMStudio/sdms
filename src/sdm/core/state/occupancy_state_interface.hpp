#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/belief_interface.hpp>
#include <sdm/core/state/jhistory_tree_interface.hpp>

namespace sdm
{
    class OccupancyInterface : virtual public BeliefInterface
    {
    public:

        // Ancienne fonction de Belief Interface 
        // Faudrait que Belief INterface possède des templates, et comme cela, on peut avoir setProbability(const Pair<State*,Obs*>&) / etc 

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
        // virtual void setAllIndividualHistories() = 0;
        // virtual const std::set<std::shared_ptr<JointHistoryTreeInterface>> &getJointHistories() const = 0;
        // virtual void setJointHistories() =0 ;
        virtual const std::set<std::shared_ptr<State>> &getStatesAt(const std::shared_ptr<JointHistoryTreeInterface> &jhistory) const =0;
        // virtual void setStates()=0;
        virtual const std::set<std::shared_ptr<HistoryTreeInterface>> &getIndividualHistories(number ag_id) const = 0;
        virtual std::shared_ptr<State> getHiddenState(const Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>> &pair_state_hist) const = 0;


    };
}