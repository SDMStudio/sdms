#pragma once

#include <sdm/core/state/state.hpp>

namespace sdm
{
    class HistoryTreeInterface : public State
    {

    public : 
        // virtual std::shared_ptr<HistoryTreeInterface> expand(const std::shared_ptr<Item> &data, bool backup = true) = 0;
        virtual void expand(const std::shared_ptr<Observation>&, const std::shared_ptr<Action>&) = 0;

        virtual std::string str() const = 0;
        virtual TypeState getTypeState() const =0;

        // virtual std::shared_ptr<HistoryTreeInterface> getParent() const = 0;
        // virtual std::shared_ptr<HistoryTreeInterface> getOrigin() = 0;
        // virtual std::vector<std::shared_ptr<HistoryTreeInterface>> getChildren() const = 0;
        // virtual std::shared_ptr<HistoryTreeInterface> getChild(const T &child_item) const = 0;
    };
}