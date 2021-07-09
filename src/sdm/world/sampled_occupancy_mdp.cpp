#include <sdm/world/sampled_occupancy_mdp.hpp>

namespace sdm
{

    SampledOccupancyMDP::SampledOccupancyMDP()
    {

    }

    SampledOccupancyMDP::SampledOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, int batch_size, bool store_action_spaces)
        : OccupancyMDP(underlying_dpomdp, memory, store_action_spaces), batch_size_(batch_size)
    {
    }

    Pair<std::shared_ptr<State>, double> SampledOccupancyMDP::computeNextStateAndProba(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t)
    {
        try
        {
            // std::cout << "start" << std::endl;
            auto occupancy_state = ostate->toOccupancyState();
            auto decision_rule = action->toDecisionRule();

            // The new compressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_fully_uncompressed_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<OccupancyStateInterface> new_one_step_left_compressed_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

            // Bag for next history (w) and next state (y) and counters for the two.
            std::unordered_map<std::shared_ptr<HistoryInterface>, Pair<double, std::unordered_map<std::shared_ptr<State>, double>>> w_y_bag;

            // This map is for keeping track of the joint history and next observation for each next joint history.
            std::unordered_map<std::shared_ptr<HistoryInterface>, Pair<std::shared_ptr<HistoryInterface>, std::shared_ptr<Observation>>> w_o_z_map;

            // Record the true state of the system before doing and sampling.
            auto true_state = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->getInternalState();

            for (int k = 0; k < this->batch_size_; k++)
            {
                // Sample joint history and belief.
                auto joint_history_belief = occupancy_state->sampleJointHistoryBelief();
                auto joint_history = joint_history_belief.first;
                auto belief = joint_history_belief.second;
                // Sample state.
                auto state = belief->sampleState();
                // Get joint action.
                auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, action, t);
                // Set state.
                std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->setInternalState(state);
                // Sample next observation.
                auto feedback = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(joint_action, false);
                auto next_observation = std::get<0>(feedback);
                // Sample next state.
                auto next_state = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->getInternalState();
                // Sample next joint history.
                auto next_joint_history = joint_history->expand(next_observation);
                // If next_joint_history is seen for the first time:
                if (w_y_bag.find(next_joint_history) == w_y_bag.end())
                {
                    // Set next_joint_history's counter to 1.
                    w_y_bag[next_joint_history].first = 1;
                    // No need to check if (next_joint_history, next_state) was seen before, the answer is no.
                    // Set (next_joint_history, next_state)'s counter to 1.
                    w_y_bag[next_joint_history].second[next_state] = 1;
                    // These two will be needed later.
                    w_o_z_map[next_joint_history] = std::make_pair(joint_history, next_observation);
                }
                // If next_joint_history was seen before:
                else
                {
                    // Incriment next_joint_history's counter.
                    w_y_bag[next_joint_history].first++;
                    // If (next_joint_history, next_state) is seen for the first time:
                    if (w_y_bag[next_joint_history].second.find(next_state) == w_y_bag[next_joint_history].second.end())
                    {
                        // Set (next_joint_history, next_state)'s counter to 1.
                        w_y_bag[next_joint_history].second[next_state] = 1;
                    }
                    // If (next_joint_history, next_state) was seen before:
                    else
                    {
                        // Incriment (next_joint_history, next_state)'s counter.
                        w_y_bag[next_joint_history].second[next_state]++;
                    }
                }
            }
            
            // Now that sampling is done, recover the true state of the system.
            std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->setInternalState(true_state);
            // std::cout << "MIDDLE" << std::endl;
            // Iterate through the (next_joint_history, next_state) pairs.
            for (auto const i : w_y_bag)
            {
                auto next_joint_history = i.first;
                auto next_joint_history_count = i.second.first;
                auto y_bag = i.second.second;
                // this->belief_mdp_->
                std::shared_ptr<BeliefInterface> next_belief = std::make_shared<Belief>();
                for (auto const j : y_bag)
                {
                    auto next_state = j.first;
                    auto next_state_count = j.second;
                    next_belief->setProbability(next_state, next_state_count / next_joint_history_count);
                }
                new_fully_uncompressed_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief, next_joint_history_count / this->batch_size_);
                auto joint_history = w_o_z_map[next_joint_history].first;
                auto next_observation = w_o_z_map[next_joint_history].second;
                auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history->toJointHistory());
                auto next_compressed_joint_history = compressed_joint_history->expand(next_observation->toObservation());
                new_one_step_left_compressed_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_count / this->batch_size_);
                new_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
            }

            // Finalize the one step left compressed occupancy state
            new_one_step_left_compressed_occupancy_state->finalize();
            new_fully_uncompressed_occupancy_state->finalize();

            if (this->compression_)
            {
                // Compress the occupancy state
                new_compressed_occupancy_state = new_one_step_left_compressed_occupancy_state->compress();
                new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
                new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);
                return std::make_pair(new_compressed_occupancy_state, 1);
            }

            new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
            new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);
            return std::make_pair(new_one_step_left_compressed_occupancy_state, 1);
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SampledOccupancyMDP::nextState(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }
} // namespace sdm
