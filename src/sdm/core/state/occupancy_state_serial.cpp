#include <iomanip>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/core/state/occupancy_state_serial.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>

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

    double OccupancyStateSerial::product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action)
    {
        double product = 0.0;
        auto decision_rule = action->toDecisionRule();
        for (auto history : this->getJointHistories())
        {
            auto action = this->applyIndivDR(decision_rule, history);
            double proba_a = 1;

            for (auto state : this->getBeliefAt(history)->getStates())
            {
                product += this->getProbability(history, state) * proba_a * beta->getValueAt(state, history, action);
            }
        }
        return product;
    }

    std::shared_ptr<Action> OccupancyStateSerial::applyIndivDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return dr->act(joint_history->getIndividualHistory(this->getCurrentAgentId()));
    }

    std::shared_ptr<DecisionRule> OccupancyStateSerial::getFullDecisionRule(const std::shared_ptr<MDPInterface> &mdp, Joint<std::shared_ptr<DecisionRule>> previous_dr, const std::shared_ptr<DecisionRule> &last_dr, number t)
    {
        previous_dr.push_back(last_dr);
        return std::make_shared<JointDeterministicDecisionRule>(previous_dr, mdp->getActionSpace(t));
    }

    std::shared_ptr<JointAction> OccupancyStateSerial::getFullAction(const std::shared_ptr<MDPInterface> &mdp, Joint<std::shared_ptr<Action>> previous_action, const std::shared_ptr<Action> &last_action, number t)
    {
        previous_action.push_back(last_action);
        return mdp->getActionSpace()->toDiscreteSpace()->getItemAddress(previous_action)->toAction()->toJointAction();
    }


    Pair<std::shared_ptr<State>, double> OccupancyStateSerial::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto idr = action->toDecisionRule();
        number simul_time = (t / this->getNumAgents());
        if (!this->isLastAgent(t))
        {
            auto next_ostate = std::static_pointer_cast<OccupancyStateSerial>(this->copy());
            next_ostate->h = t+1;
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
            auto ostate = std::dynamic_pointer_cast<OccupancyStateSerial>(res.first);
            ostate->h = t+1;
            return {ostate, res.second};
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
        return  OccupancyState::hash(precision);
    }


    bool OccupancyStateSerial::operator==(const OccupancyStateSerial &other) const
    {
        auto eq = this->isEqual(other, OccupancyState::PRECISION);
        return eq;
    }

    bool OccupancyStateSerial::isEqual(const OccupancyStateSerial &other, double precision) const
    {
            return OccupancyState::isEqual(other, precision);
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
                res << this->actions.at(joint_history);
            }
            res << ") belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</serial-occupancy-state>";
        return res.str();
    }

}