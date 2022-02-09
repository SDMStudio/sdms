#include <iomanip>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

namespace sdm
{
    SerialOccupancyState::SerialOccupancyState() : OccupancyState() {}
    SerialOccupancyState::SerialOccupancyState(number num_agents) : OccupancyState(num_agents) {}
    SerialOccupancyState::SerialOccupancyState(number num_agents, StateType stateType) : OccupancyState(num_agents, stateType) {}
    SerialOccupancyState::SerialOccupancyState(const SerialOccupancyState &copy) : OccupancyState(copy)
    {
    }

    std::shared_ptr<OccupancyState> SerialOccupancyState::make()
    {
        return std::make_shared<SerialOccupancyState>(this->num_agents_, this->state_type);
    }

    std::shared_ptr<OccupancyState> SerialOccupancyState::copy()
    {
        return std::make_shared<SerialOccupancyState>(*this);
    }

    // Pair<std::shared_ptr<State>, double> SerialOccupancyState::next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    // {
    //     // The new one step left occupancy state
    //     auto next_one_step_left_compressed_occupancy_state = this->make();
    //     auto decision_rule = action->toDecisionRule();
    //     auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(mdp);

    //     // For each joint history in the support of the fully uncompressed occupancy state
    //     for (const auto &compressed_joint_history : this->getJointHistories())
    //     {
    //         // Get p(o_t)
    //         double proba_history = this->getProbability(compressed_joint_history);

    //         // Get the corresponding belief
    //         auto belief = this->getBeliefAt(compressed_joint_history);

    //         // Apply decision rule and get action
    //         auto jaction = decision_rule->act(compressed_joint_history);

    //         // For each action that is likely to be taken
    //         for (const auto &joint_action : {jaction}) // decision_rule->getDistribution(compressed_joint_history)->getSupport())
    //         {
    //             // Get p(u_t | o_t)
    //             double proba_action = 1; // decision_rule->getProbability(compressed_joint_history, joint_action);

    //             // For each observation in the space of joint observation
    //             for (auto jobs : *pomdp->getObservationSpace(t))
    //             {
    //                 auto joint_observation = jobs->toObservation();
    //                 if (this->checkCompatibility(joint_observation, observation))
    //                 {
    //                     // Get the next belief and p(z_{t+1} | b_t, u_t)
    //                     auto [next_belief, proba_observation] = belief->next(mdp, jaction, joint_observation, t);

    //                     double next_joint_history_probability = proba_history * proba_action * proba_observation;

    //                     // If the next history probability is not zero
    //                     if (next_joint_history_probability > 0)
    //                     {
    //                         // Update new one step uncompressed occupancy state
    //                         std::shared_ptr<JointHistoryInterface> next_compressed_joint_history = compressed_joint_history->expand(joint_observation /*, joint_action*/)->toJointHistory();
    //                         this->updateOccupancyStateProba(next_one_step_left_compressed_occupancy_state, next_compressed_joint_history, next_belief->toBelief(), next_joint_history_probability);
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     return this->finalizeNextState(next_one_step_left_compressed_occupancy_state, t);
    // }

    // double SerialOccupancyState::getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t)
    // {
    //     double reward = 0.;
    //     auto decision_rule = action->toDecisionRule();
    //     auto seq_mmdp = std::dynamic_pointer_cast<SerialMMDPInterface>(mdp);
    //     // For all histories in the occupancy state
    //     for (const auto &jhist : this->getJointHistories())
    //     {
    //         // Get the belief corresponding to this history
    //         auto belief = this->getBeliefAt(jhist);

    //         // Get the action from decision rule
    //         auto iaction = decision_rule->act(jhist->getIndividualHistory(seq_mmdp->getAgentId(t)));

    //         // Update the expected reward
    //         reward += this->getProbability(jhist) * belief->getReward(seq_mmdp, iaction, t);
    //     }
    //     return reward;
    // }

    std::shared_ptr<Action> SerialOccupancyState::applyDR(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history, number t)
    {
        auto seq_mmdp = std::dynamic_pointer_cast<SerialMMDPInterface>(mdp);
        return dr->act(joint_history->getIndividualHistory(seq_mmdp->getAgentId(t)));
    }

    number SerialOccupancyState::getCurrentAgentId() const
    {
        return std::dynamic_pointer_cast<SerialState>(*this->getBeliefAt(*this->getJointHistories().begin())->getStates().begin())->getCurrentAgentId();
    }

    TypeState SerialOccupancyState::getTypeState() const
    {
        return TypeState::SERIAL_OCCUPANCY_STATE;
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