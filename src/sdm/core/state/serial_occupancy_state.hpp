#pragma once

#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

namespace sdm
{

    /**
     * @brief A serial occupancy state refers to the complete knowledge the central planner have access 
     * to take decisions in serial decentralized POMDPs.
     *
     * Serial occupancy states (also known as sequential occupancy states) are firstly introduced by ... in [...](...).
     * This class implement the serial occupancy state as a posterior distribution over serial states and histories, 
     * given a complete information state (i.e. \$\\xi_t (x_{t, serial}, o_{t} ) = p(x_{t, serial}, o_t \\mid i_{t})\$ ) .
     */
    class SerialOccupancyState : public OccupancyState, public SerialInterface
    {
    public:
        SerialOccupancyState();
        SerialOccupancyState(number num_agents, number h);
        SerialOccupancyState(number num_agents, number h, StateType stateType);
        SerialOccupancyState(const SerialOccupancyState &copy);

        std::shared_ptr<OccupancyState> make(number h);
        std::shared_ptr<OccupancyState> copy();

        virtual std::shared_ptr<Action> applyDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const;

        /**
         * @brief Get the current Agent Id of the object
         *
         * @return number
         */
        number getCurrentAgentId() const;
        number getNumAgents() const;

        std::string str() const;
    };
}

namespace std
{
    template <>
    struct hash<sdm::SerialOccupancyState>
    {
        typedef sdm::SerialOccupancyState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::OccupancyState>()(in);
        }
    };
}