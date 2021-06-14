#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/history_tree_interface.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    class JointHistoryTreeInterface : public HistoryTreeInterface
    {

    public : 
        virtual std::string str() const = 0;
        virtual TypeState getTypeState() const =0;

        virtual std::shared_ptr<HistoryTreeInterface> getIndividualHistory(number agent_id) const =0;
        virtual void expand(const std::shared_ptr<Joint<std::shared_ptr<Observation>>>&, const std::shared_ptr<Joint<std::shared_ptr<Action>>>&) = 0;

        // virtual std::shared_ptr<HistoryTree<T>> getParent() const = 0;
        // virtual std::shared_ptr<HistoryTree<T>> getOrigin() = 0;
        // virtual std::vector<std::shared_ptr<HistoryTree<T>>> getChildren() const = 0;
        // virtual std::shared_ptr<HistoryTree<T>> getChild(const T &child_item) const = 0;
    };
}