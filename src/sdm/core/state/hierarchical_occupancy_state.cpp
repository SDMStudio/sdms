#include <iomanip>
#include <sdm/core/state/hierarchical_occupancy_state.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

namespace sdm
{
    HierarchicalOccupancyState::HierarchicalOccupancyState() : OccupancyState() {}
    HierarchicalOccupancyState::HierarchicalOccupancyState(number num_agents, number h) : OccupancyState(num_agents, h){}
    HierarchicalOccupancyState::HierarchicalOccupancyState(number num_agents, number h, StateType stateType) : OccupancyState(num_agents, h, stateType){}
    HierarchicalOccupancyState::HierarchicalOccupancyState(const HierarchicalOccupancyState &copy) : OccupancyState(copy){}

    std::shared_ptr<OccupancyState> HierarchicalOccupancyState::make(number h)
    {
        return std::make_shared<HierarchicalOccupancyState>(this->num_agents_, h, this->state_type);
    }

    std::shared_ptr<OccupancyState> HierarchicalOccupancyState::copy()
    {
        return std::make_shared<HierarchicalOccupancyState>(*this);
    }

    bool HierarchicalOccupancyState::checkCompatibility(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Observation> &observation)
    {
        return (std::static_pointer_cast<JointObservation>(joint_observation)->get(this->getLowLevelAgentID()) == observation);
    }

    number HierarchicalOccupancyState::getLowLevelAgentID()
    {
        return this->low_level_agent_id_;
    }

    std::string HierarchicalOccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<hierarchical-occupancy-state size=\"" << this->size() << "\">\n";
        for (const auto &history_as_state : this->getStates())
        {
            auto joint_history = history_as_state->toHistory()->toJointHistory();
            res << "\t<probability";
            res << " joint_history=" << joint_history->short_str() << "";
            res << " belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</hierarchical-occupancy-state>";
        return res.str();
    }

}