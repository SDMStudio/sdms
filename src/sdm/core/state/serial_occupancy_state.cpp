#include <iomanip>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

namespace sdm
{
    SerialOccupancyState::SerialOccupancyState() : OccupancyState() {}
    SerialOccupancyState::SerialOccupancyState(number num_agents, number h) : OccupancyState(num_agents, h) {}
    SerialOccupancyState::SerialOccupancyState(number num_agents, number h, StateType stateType) : OccupancyState(num_agents, h, stateType) {}
    SerialOccupancyState::SerialOccupancyState(const SerialOccupancyState &copy) : OccupancyState(copy)
    {
    }

    std::shared_ptr<OccupancyState> SerialOccupancyState::make(number h)
    {
        return std::make_shared<SerialOccupancyState>(this->num_agents_, h, this->state_type);
    }

    std::shared_ptr<OccupancyState> SerialOccupancyState::copy()
    {
        return std::make_shared<SerialOccupancyState>(*this);
    }

    std::shared_ptr<Action> SerialOccupancyState::applyDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return dr->act(joint_history->getIndividualHistory(this->getCurrentAgentId()));
    }

    number SerialOccupancyState::getCurrentAgentId() const
    {
        return (this->h % this->num_agents_);
    }

    number SerialOccupancyState::getNumAgents() const
    {
        return this->num_agents_;
    }

    std::string SerialOccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<serial-occupancy-state agent_id=\"" << this->getCurrentAgentId() << "\t size=\"" << this->size() << "\">\n";
        for (const auto &history_as_state : this->getStates())
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