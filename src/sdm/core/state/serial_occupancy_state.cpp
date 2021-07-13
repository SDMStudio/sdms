#include <sdm/core/state/serial_occupancy_state.hpp>
#include <iomanip>

namespace sdm
{
    SerialOccupancyState::SerialOccupancyState() : OccupancyState() {}
    SerialOccupancyState::SerialOccupancyState(number num_agents) :OccupancyState(num_agents) {}
    SerialOccupancyState::SerialOccupancyState(const SerialOccupancyState &copy) : OccupancyState(copy) 
    {
        this->setAgentId(copy.agentID_);
    }

    number SerialOccupancyState::getCurrentAgentId() const
    {
        return this->agentID_;
    }

    void SerialOccupancyState::setAgentId(number agent_id)
    {
        this->agentID_ = agent_id;
    }

    TypeState SerialOccupancyState::getTypeState() const
    {
        return TypeState::SERIAL_OCCUPANCY_STATE;
    }

    std::string SerialOccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<serial-occupancy-state agent_id=\""<<this->getCurrentAgentId()<<"\t size=\"" << MappedVector<std::shared_ptr<State>>::size() << "\">\n";
        for (const auto &history_as_state : this->getIndexes())
        {
            auto joint_history = history_as_state->toHistory()->toJointHistory();
            res << "\t<probability";
            res << " joint_history=" << joint_history->short_str() << "";
            res << " belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</serial-occupancy-state>";
        return res.str();
    }
}