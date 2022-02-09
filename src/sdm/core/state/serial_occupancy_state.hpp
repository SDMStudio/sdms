#pragma once

#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

namespace sdm
{
    class SerialOccupancyState : public OccupancyState, public SerialInterface
    {
    public:
        SerialOccupancyState();
        SerialOccupancyState(number num_agents);
        SerialOccupancyState(number num_agents, StateType stateType);
        SerialOccupancyState(const SerialOccupancyState &copy);

        std::shared_ptr<OccupancyState> make();
        std::shared_ptr<OccupancyState> copy();

        // Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);
        // double getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t);
        std::shared_ptr<Action> applyDR(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history, number t);

        /**
         * @brief Get the current Agent Id of the object
         *
         * @return number
         */
        number getCurrentAgentId() const;
        TypeState getTypeState() const;
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