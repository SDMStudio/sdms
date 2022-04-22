#pragma once

#include <string>
#include <sdm/types.hpp>

namespace sdm
{
    class State;
    class Action;
    class Observation;

    /**
     * @brief A public interface for actions, states and observations.
     */
    class Item : public std::enable_shared_from_this<Item>
    {
    public:
        using base = Item;

        /** @brief Cast the item into a pointer on item. */
        virtual std::shared_ptr<Item> toItem();
        
        /** @brief Cast the item into a pointer on state. */
        virtual std::shared_ptr<State> toState();
        
        /** @brief Cast the item into a pointer on action. */
        virtual std::shared_ptr<Action> toAction();

        /** @brief Cast the item into a pointer on observation. */
        virtual std::shared_ptr<Observation> toObservation();

        /**
         * @brief Get a string describing the current item 
         * 
         * @return std::string the description of the item 
         */
        virtual std::string str() const = 0;

        /**
         * @brief Get the shared_ptr on current (this) item
         * 
         * @return std::shared_ptr<Item> 
         */
        virtual std::shared_ptr<Item> getPointer();

        template <typename T>
        std::shared_ptr<T> to()
        {
            return std::dynamic_pointer_cast<T>(this->shared_from_this());
        }

        friend std::ostream &
        operator<<(std::ostream &os, const Item &item)
        {
            os << item.str();
            return os;
        }

        virtual bool isBaseItem(){return false;}
    };
} // namespace sdm
