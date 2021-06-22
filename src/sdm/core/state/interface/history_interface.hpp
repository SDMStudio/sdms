#pragma once

#include <sdm/core/state/state.hpp>

namespace sdm
{
    class HistoryInterface : public State
    {

    public : 
        // virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Item> &data, bool backup = true) = 0;
        virtual std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation>&, const std::shared_ptr<Action>& = nullptr, bool = true) = 0;

        virtual std::string str() const = 0;
        virtual std::string short_str() const = 0;
        virtual TypeState getTypeState() const =0;

        // virtual std::shared_ptr<HistoryInterface> getParent() const = 0;
        // virtual std::shared_ptr<HistoryInterface> getOrigin() = 0;
        // virtual std::vector<std::shared_ptr<HistoryInterface>> getChildren() const = 0;
        // virtual std::shared_ptr<HistoryInterface> getChild(const T &child_item) const = 0;
    };
}