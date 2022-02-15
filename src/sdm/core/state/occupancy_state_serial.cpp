#include <iomanip>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/core/state/occupancy_state_serial.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{
    OccupancyStateSerial::OccupancyStateSerial() {}
    OccupancyStateSerial::OccupancyStateSerial(number num_agents, number h, StateType stateType, Joint<std::shared_ptr<DecisionRule>> decision_rules)
        : OccupancyState(num_agents, h, stateType), decision_rules(decision_rules)
    {
        for (const auto &jhist : this->getJointHistories())
        {
            for (const auto &idr : decision_rules)
            {
                this->actions[jhist].push_back(this->applyIndivDR(idr, jhist));
            }
        }
    }

    OccupancyStateSerial::OccupancyStateSerial(const OccupancyStateSerial &copy) : OccupancyState(copy),
                                                                                   decision_rules(copy.decision_rules),
                                                                                   actions(copy.actions)
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
        return this->decision_rules.size();
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

    std::shared_ptr<Action> OccupancyStateSerial::applyIndivDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        // return OccupancyState::applyDR(dr, joint_history);
        return dr->act(joint_history->getIndividualHistory(this->getCurrentAgentId()));
    }

    std::shared_ptr<DecisionRule> OccupancyStateSerial::getFullDecisionRule(const std::shared_ptr<MDPInterface> &mdp, Joint<std::shared_ptr<DecisionRule>> previous_dr, const std::shared_ptr<DecisionRule> &last_dr, number t)
    {
        previous_dr.push_back(last_dr);
        return std::make_shared<JointDeterministicDecisionRule>(previous_dr, mdp->getActionSpace(t));
    }

    Pair<std::shared_ptr<State>, double> OccupancyStateSerial::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto idr = action->toDecisionRule();
        number simul_time = (t / this->getNumAgents());
        if (!this->isLastAgent(t))
        {
            auto next_ostate = std::static_pointer_cast<OccupancyStateSerial>(this->copy());
            next_ostate->h++;
            next_ostate->decision_rules.push_back(idr);
            for (const auto &jhist : this->getJointHistories())
            {
                next_ostate->actions[jhist].push_back(this->applyIndivDR(idr, jhist));
            }
            return std::make_pair(next_ostate, 1.);
        }
        else
        {
            auto res = OccupancyState::next(mdp, this->getFullDecisionRule(mdp, this->decision_rules, idr, simul_time), observation, simul_time);
            return res;
        }
    }

    double OccupancyStateSerial::getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t)
    {
        number simul_time = (t / this->getNumAgents());
        if (!this->isLastAgent(t))
        {
            return 0.;
        }
        else
        {
            auto full = this->getFullDecisionRule(mdp, this->decision_rules, action->toDecisionRule(), simul_time);
            return OccupancyState::getReward(mdp, full, simul_time);
        }
    }

    size_t OccupancyStateSerial::hash(double precision) const
    {
        size_t seed = OccupancyState::hash(precision);
        // sdm::hash_combine(seed, this->h);

        return seed;
    }


    bool OccupancyStateSerial::operator==(const OccupancyStateSerial &other) const
    {
        return this->isEqual(other, OccupancyState::PRECISION);
    }

    bool OccupancyStateSerial::isEqual(const OccupancyStateSerial &other, double precision) const
    {

        // if (this->h != other.h)
        // {
        //     return false;
        // }
        
        if (this->actions != other.actions)
        {
            return false;
        }
        else
        {
            return OccupancyState::isEqual(other, precision);
        }
    }

    bool OccupancyStateSerial::isEqual(const std::shared_ptr<State> &other, double precision) const
    {
        return this->isEqual(*std::dynamic_pointer_cast<OccupancyStateSerial>(other), precision);
    }

    std::string OccupancyStateSerial::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;
        auto copy = this;
        res << "<serial-occupancy-state addr=" << copy << " t=" << this->h << " agent_id=\"" << this->getCurrentAgentId() << "\">\n";
        for (const auto &joint_history : this->getJointHistories())
        {
            res << "\t<probability";
            res << " joint_history=" << joint_history->short_str() << "";
            res << " serial_actions=(";
            for (number i = 0; i < this->decision_rules.size(); i++)
            {
                res << this->decision_rules.at(i)->act(joint_history->getIndividualHistory(i))->str();
            }
            res << ") belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</serial-occupancy-state>";
        return res.str();
    }

}