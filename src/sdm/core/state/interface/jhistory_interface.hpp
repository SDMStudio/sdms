#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    class JointHistoryInterface : public HistoryInterface
    {

    public : 
        virtual std::string str() const = 0;
        virtual TypeState getTypeState() const =0;

        virtual std::shared_ptr<HistoryInterface> getIndividualHistory(number agent_id) const =0;
        virtual Joint<std::shared_ptr<HistoryInterface>> getIndividualHistories() const =0;

        virtual Joint<std::shared_ptr<State>> JointHistoryTreeToJointState(const Joint<std::shared_ptr<HistoryInterface>>&) =0;

        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Joint<std::shared_ptr<Observation>>>&, const std::shared_ptr<Joint<std::shared_ptr<Action>>>& = nullptr, bool = true) = 0;

        // virtual std::shared_ptr<HistoryTree<T>> getParent() const = 0;
        // virtual std::shared_ptr<HistoryTree<T>> getOrigin() = 0;
        // virtual std::vector<std::shared_ptr<HistoryTree<T>>> getChildren() const = 0;
        // virtual std::shared_ptr<HistoryTree<T>> getChild(const T &child_item) const = 0;
    };
}