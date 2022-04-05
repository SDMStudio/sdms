#pragma once

#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

namespace sdm
{

    /**
     * @brief A hierarchical occupancy state.
     *
     * This kind of state refered to an occupancy state for problems
     * involving a hierarchical structure.
     */
    class HierarchicalOccupancyState : public OccupancyState
    {
    public:
        HierarchicalOccupancyState();
        HierarchicalOccupancyState(number num_agents, number h);
        HierarchicalOccupancyState(number num_agents, number h, StateType stateType);
        HierarchicalOccupancyState(const HierarchicalOccupancyState &copy);

        std::shared_ptr<OccupancyState> make(number h);
        std::shared_ptr<OccupancyState> copy();

        number getLowLevelAgentID();
        bool checkCompatibility(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Observation> &observation);

        std::string str() const;

    protected:
        /** @brief the identifier of the agent at the last level of the hierarchy */
        number low_level_agent_id_ = 0;
    };
}

namespace std
{
    template <>
    struct hash<sdm::HierarchicalOccupancyState>
    {
        typedef sdm::HierarchicalOccupancyState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::OccupancyState>()(in);
        }
    };
}