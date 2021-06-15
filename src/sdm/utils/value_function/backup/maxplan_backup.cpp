#include <sdm/utils/value_function/backup/maxplan_backup.hpp>

namespace sdm
{
    std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        // TValue current, max = -std::numeric_limits<TValue>::max();
        // std::shared_ptr<BeliefState> alpha_vector;

        // for (const auto &plan : this->representation[t])
        // {
        //     current = state ^ plan;

        //     if (max < current)
        //     {
        //         max = current;
        //         alpha_vector = plan;
        //     }
        // }

        // return {max, alpha_vector};
    }

    std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {

    }

    std::shared_ptr<State> getBackup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {

    }




    // std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &occupancy_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &joint_decision_rule, number t)
    // {
    //     std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
    //     auto under_pb = this->getWorld()->getUnderlyingProblem();

    //     // Go over all occupancy state
    //     for (const auto &uncompressed_belief_history_proba : *occupancy_state.getFullyUncompressedOccupancy())
    //     {
    //         //Get information from uncompressed_belief_history_proba
    //         auto uncompressed_belief = uncompressed_belief_history_proba.first.first;
    //         auto uncompressed_joint_history = uncompressed_belief_history_proba.first.second;
    //         auto compressed_joint_history = occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

    //         // Get the serial action from the serial_decision_rule
    //         auto joint_action = joint_decision_rule.act(compressed_joint_history->getIndividualHistories());

    //         // Add the reward of the hyperplan
    //         new_hyperplan.addProbabilityAt(uncompressed_belief_history_proba.first, (uncompressed_belief->getData() ^ under_pb->getReward()->getReward(under_pb->getActionSpace()->joint2single(joint_action))));

    //         //Go over all Reachable Observation
    //         for (const auto &next_observation : under_pb->getObsSpace()->getAll())
    //         {
    //             auto next_joint_history = compressed_joint_history->expand(next_observation);
    //             auto next_belief = uncompressed_belief->expand(under_pb->getActionSpace()->joint2single(joint_action), under_pb->getObsSpace()->joint2single(next_observation));
    //             new_hyperplan.addProbabilityAt(uncompressed_belief_history_proba.first, under_pb->getDiscount() * uncompressed_belief->getProbability(under_pb->getActionSpace()->joint2single(joint_action), under_pb->getObsSpace()->joint2single(next_observation)) * next_hyperplan.at({next_belief, next_joint_history}));
    //         }
    //     }
    //     return new_hyperplan;
    // }

    // // ---------------------------------------------------------------
    // // --------- DEFINITION FOR OccupancyMDP FORMALISM ---------------
    // // ---------------------------------------------------------------

    // std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &occupancy_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &joint_decision_rule, number t)
    // {
    //     std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
    //     auto under_pb = this->getWorld()->getUnderlyingProblem();

    //     // Go over all occupancy state
    //     for (const auto &uncompressed_s_o : *occupancy_state.getFullyUncompressedOccupancy())
    //     {
    //         //Get information from uncompressed_s_o
    //         auto uncompressed_hidden_state = uncompressed_s_o.first.first;
    //         auto uncompressed_joint_history = uncompressed_s_o.first.second;
    //         auto compressed_joint_history = occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

    //         // Get the serial action from the serial_decision_rule
    //         auto action = joint_decision_rule.act(compressed_joint_history->getIndividualHistories());

    //         // Add the reward of the hyperplan
    //         new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getReward()->getReward(uncompressed_hidden_state, under_pb->getActionSpace()->joint2single(action)));

    //         //Go ober all Reachable State
    //         for (const auto &next_hidden_state : under_pb->getReachableStates(uncompressed_hidden_state, action))
    //         {
    //             //Go ober all Reachable Observation
    //             for (const auto &next_observation : under_pb->getReachableObservations(uncompressed_hidden_state, action, next_hidden_state))
    //             {
    //                 auto next_joint_history = compressed_joint_history->expand(next_observation);
    //                 new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getDiscount() * under_pb->getObsDynamics()->getDynamics(uncompressed_hidden_state, under_pb->getActionSpace()->joint2single(action), under_pb->getObsSpace()->joint2single(next_observation), next_hidden_state) * next_hyperplan.at({next_hidden_state, next_joint_history}));
    //             }
    //         }
    //     }
    //     return new_hyperplan;
    // }

    // // ---------------------------------------------------------------
    // // ------- DEFINITION FOR SerializedOccupancyMDP FORMALISM -------
    // // ---------------------------------------------------------------

    // std::shared_ptr<BeliefState> MaxPlanValueFunction::getHyperplanAt(const std::shared_ptr<BeliefState> &serial_occupancy_state, const std::shared_ptr<BeliefState> &next_hyperplan, const TAction &serial_decision_rule, number t)
    // {
    //     auto under_pb = this->getWorld()->getUnderlyingProblem();

    //     std::shared_ptr<BeliefState> new_hyperplan(this->default_values_per_horizon[t]);
    //     number ag_id = serial_occupancy_state.getCurrentAgentId();
    //     new_hyperplan.setAgent(ag_id);

    //     // Go over all serial occupancy state
    //     for (const auto &uncompressed_s_o : *serial_occupancy_state.getFullyUncompressedOccupancy())
    //     {
    //         //Get information from uncompressed_s_o
    //         auto uncompressed_hidden_serial_state = serial_occupancy_state.getState(uncompressed_s_o.first);
    //         auto uncompressed_joint_history = serial_occupancy_state.getHistory(uncompressed_s_o.first);
    //         auto compressed_joint_history = serial_occupancy_state.getCompressedJointHistory(uncompressed_joint_history);

    //         // Get the serial action from the serial_decision_rule
    //         auto serial_action = serial_decision_rule.act(compressed_joint_history->getIndividualHistory(ag_id));

    //         // Add the reward of the hyperplan
    //         new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getReward(uncompressed_hidden_serial_state, serial_action));

    //         // Go over all Reachable Serial State
    //         for (const auto &next_hidden_serial_state : under_pb->getReachableSerialStates(uncompressed_hidden_serial_state, serial_action))
    //         {
    //             // Go over all Reachable Observation
    //             for (const auto &next_serial_observation : under_pb->getReachableObservations(uncompressed_hidden_serial_state, serial_action, next_hidden_serial_state))
    //             {
    //                 auto next_joint_history = compressed_joint_history->expand(next_serial_observation);
    //                 new_hyperplan.addProbabilityAt(uncompressed_s_o.first, under_pb->getDiscount(t) * under_pb->getDynamics(uncompressed_hidden_serial_state, serial_action, next_serial_observation, next_hidden_serial_state) * next_hyperplan.at({next_hidden_serial_state, next_joint_history}));
    //             }
    //         }
    //     }
    //     return new_hyperplan;
    // }

    // BeliefState<> MaxPlanValueFunction<BeliefState<>, number, double>::backup_operator(const BeliefState<> &state, number t)
    // {
    //     std::cout << "in backup"<<std::endl;
    //     auto beliefMDP = std::static_pointer_cast<BeliefMDP<BeliefState<>, number, number>>(this->getWorld());
    //     auto under_pb = this->getWorld()->getUnderlyingProblem();

    //     number n_obs = under_pb->getObsSpace()->getNumItems();
    //     number n_actions = under_pb->getActionSpace()->getNumItems();
    //     number n_states = under_pb->getStateSpace()->getNumItems();

    //     std::vector<std::vector<BeliefState<>>> beta_a_o(n_actions, std::vector<BeliefState<>>(n_obs, BeliefState<>(n_states)));
    //     std::vector<BeliefState<>> beta_a(n_actions, BeliefState<>(n_states));

    //     // beta_a_o = argmax_alpha ( alpha * belief_t+1)
    //     for (number a = 0; a < n_actions; a++)
    //     {
    //         for (number o = 0; o < n_obs; o++)
    //         {
    //             auto next_belief = beliefMDP->nextState(state, a, o);
    //             beta_a_o[a][o] = this->getMaxAt(next_belief, t + 1).second;
    //         }
    //     }

    //     // \beta_a = R(s,a) + \gamma * \sum_{o, s'} [ \beta_{a,o}(s') * O(s', a, o) * T(s,a,s') ]
    //     for (number a = 0; a < n_actions; a++)
    //     {
    //         for (number s = 0; s < n_states; s++)
    //         {
    //             double tmp = 0;
    //             for (number o = 0; o < n_obs; o++)
    //             {
    //                 for (number s_ = 0; s_ < n_states; s_++)
    //                 {
    //                     tmp += beta_a_o[a][o].at(s_) * under_pb->getObsDynamics()->getDynamics(s, a, o, s_);
    //                 }
    //             }
    //             beta_a[a][s] = under_pb->getReward(s, a) + under_pb->getDiscount() * tmp;
    //         }
    //     }

    //     number a_max;
    //     double current, max_v = -std::numeric_limits<double>::max();
    //     for (number a = 0; a < n_actions; a++)
    //     {
    //         current = state ^ beta_a.at(a);
    //         if (current > max_v)
    //         {
    //             max_v = current;
    //             a_max = a;
    //         }
    //     }
    //     auto new_plan = beta_a[a_max];

    //     return new_plan;
    // }
}

