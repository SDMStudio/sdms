#pragma once

#include <string>
#include <sdm/types.hpp>

namespace sdm
{
    class State;
    class Action;
    class Observation;

    class Item : public std::inheritable_enable_shared_from_this<Item>
    {
    public:
        virtual std::string str() const = 0;

        virtual std::shared_ptr<Item> toItem();
        virtual std::shared_ptr<State> toState();
        virtual std::shared_ptr<Action> toAction();
        virtual std::shared_ptr<Observation> toObservation();

        virtual std::shared_ptr<Item> getPointer();

        template <typename T>
        std::shared_ptr<T> to()
        {
            return std::static_pointer_cast<T>(this->shared_from_this());
        }

        friend std::ostream &
        operator<<(std::ostream &os, const Item &item)
        {
            os << item.str();
            return os;
        }
    };
} // namespace sdm
