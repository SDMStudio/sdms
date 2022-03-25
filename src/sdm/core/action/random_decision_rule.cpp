#include <sdm/core/action/random_decision_rule.hpp>
#include <sdm/core/space/discrete_space.hpp>

namespace sdm
{

    RandomDecisionRule::RandomDecisionRule(const std::shared_ptr<Space> &action_space) : action_space(action_space) {}

    std::shared_ptr<Action> RandomDecisionRule::act(const std::shared_ptr<HistoryInterface> &) const
    {
        return this->action_space->sample()->toAction();
    }

    double RandomDecisionRule::getProbability(const std::shared_ptr<HistoryInterface> &state, const std::shared_ptr<Action> &action) const
    {
        return (this->action_space->isContinuous()) ? 0 : 1. / this->action_space->toDiscreteSpace()->getNumItems();
    }

    void RandomDecisionRule::setProbability(const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &, double)
    {
        throw sdm::exception::Exception("Cannot change probabilities of RandomDecisionRules");
    }

    std::string RandomDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<random-decision-rule type=\"stochastic\">" << std::endl;
        res << "\t<decision state=\"*\" action=\"" << *this->action_space << "\"/>" << std::endl;
        res << "<random-decision-rule/>";
        return res.str();
    }
}