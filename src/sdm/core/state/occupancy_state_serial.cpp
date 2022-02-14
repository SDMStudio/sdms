#include <iomanip>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/core/state/occupancy_state_serial.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{
    OccupancyStateSerial::OccupancyStateSerial() {}
    OccupancyStateSerial::OccupancyStateSerial(number num_agents, number h, StateType stateType, Joint<std::shared_ptr<DecisionRule>> actions)
        : OccupancyState(num_agents, h, stateType), actions(actions) {}

    OccupancyStateSerial::OccupancyStateSerial(const OccupancyStateSerial &copy) : OccupancyState(copy)
    {
    }

    OccupancyStateSerial::~OccupancyStateSerial() {}

    std::shared_ptr<OccupancyState> OccupancyStateSerial::make(number h)
    {
        return std::make_shared<OccupancyStateSerial>(this->num_agents_, h, this->state_type);
    }

    std::shared_ptr<OccupancyState> OccupancyStateSerial::copy()
    {
        return std::make_shared<OccupancyStateSerial>(*this);
    }

    number OccupancyStateSerial::getCurrentAgentId() const
    {
        return this->actions.size();
    }

    number OccupancyStateSerial::getNumAgents() const
    {
        return this->num_agents_;
    }

    std::shared_ptr<Action> OccupancyStateSerial::applyDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return OccupancyState::applyDR(dr, joint_history);
        // return dr->act(joint_history->getIndividualHistory(this->getCurrentAgentId()));
    }

    std::shared_ptr<DecisionRule> OccupancyStateSerial::getFullDecisionRule(const std::shared_ptr<MDPInterface> &mdp, Joint<std::shared_ptr<DecisionRule>> previous_dr, const std::shared_ptr<DecisionRule> &last_dr, number t)
    {
        previous_dr.push_back(last_dr);
        return std::make_shared<JointDeterministicDecisionRule>(previous_dr, mdp->getActionSpace(t));
    }

    Pair<std::shared_ptr<State>, double> OccupancyStateSerial::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        if (!this->isLastAgent(t))
        {
            auto next_ostate = std::static_pointer_cast<OccupancyStateSerial>(this->copy());
            next_ostate->h++;
            next_ostate->actions.push_back(action->toDecisionRule());
            next_ostate->finalize();
            return std::make_pair(next_ostate, 1.);
        }
        else
        {
            auto res = OccupancyState::next(mdp, this->getFullDecisionRule(mdp, this->actions, action->toDecisionRule(), t), observation, t);
            return res;
        }
    }

    double OccupancyStateSerial::getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t)
    {
        if (!this->isLastAgent(t))
        {
            return 0.;
        }
        else
        {
            auto full = this->getFullDecisionRule(mdp, this->actions, action->toDecisionRule(), t);
            return OccupancyState::getReward(mdp, full, t);
        }
    }

    std::string OccupancyStateSerial::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;
        auto copy = this;
        res << "<serial-occupancy-state addr=" << copy << " t="<< this->h<<" agent_id=\"" << this->getCurrentAgentId() << "\">\n";
        for (const auto &joint_history : this->getJointHistories())
        {
            res << "\t<probability";
            res << " joint_history=" << joint_history->short_str() << "";
            res << " serial_actions=(";
            for (number i = 0; i < this->actions.size(); i++)
            {
                res << this->actions.at(i)->act(joint_history->getIndividualHistory(i))->str();
            }
            res << ") belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</serial-occupancy-state>";
        return res.str();
    }

}