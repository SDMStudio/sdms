#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{

    /**
     * @brief A common interface for objects that are serialized.
     * 
     */
    class SerialInterface
    {
    public:
        /**
         * @brief Get the current Agent Id of the object
         * 
         * @return number the current agent id
         */
        virtual number getCurrentAgentId() const = 0;
    };

    /**
     * @brief A common interface for serial states. 
     * 
     */
    class BaseSerialInterface : public SerialInterface, virtual public State
    {
    public:
        /**
         * @brief Get the Hidden State of the serial object
         * 
         * @return std::shared_ptr<State> 
         */
        virtual std::shared_ptr<State> getHiddenState() const = 0;

        /**
         * @brief Get the hidden vector of action that were already decided
         * p
         * @return std::vector<std::shared_ptr<Action>> 
         */
        virtual Joint<std::shared_ptr<Action>> getAction() const = 0;

        /**
         * @brief Set the Agent Id of the object
         * 
         */
        virtual void setAgentId(number) = 0;
    };

}