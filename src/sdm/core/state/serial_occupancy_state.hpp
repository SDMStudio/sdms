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
        SerialOccupancyState(const SerialOccupancyState &copy);
        SerialOccupancyState(const OccupancyState &copy);

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