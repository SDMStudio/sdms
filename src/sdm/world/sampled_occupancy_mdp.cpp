#include <sdm/world/sampled_occupancy_mdp.hpp>

namespace sdm
{

    SampledOccupancyMDP::SampledOccupancyMDP()
    {

    }

    SampledOccupancyMDP::SampledOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool store_action_spaces, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, store_action_spaces), batch_size_(batch_size)
    {

    }

    Pair<std::shared_ptr<State>, double> SampledOccupancyMDP::computeNextStateAndProba(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t)
    {
        try
        {   
            auto occupancy_state = ostate->toOccupancyState();
            auto decision_rule = action->toDecisionRule();

            // The new compressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_fully_uncompressed_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<OccupancyStateInterface> new_one_step_left_compressed_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

            auto new_occupancy_state = std::make_shared<OccupancyState>();

            for (int sample_hist_belief = 0; sample_hist_belief < this->batch_size_; sample_hist_belief++)
            {
                // Sample pair history / belief
                auto [joint_history, belief] = ostate->toOccupancyState()->sample();

                // Apply decision rule
                auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, decision_rule, t);

                auto next_belief = std::make_shared<Belief>();

                for (int n_sample_state = 0; n_sample_state < this->batch_size_; n_sample_state++)
                {
                    // Sample hidden state
                    auto x = belief->sample();

                    // Set internal state
                    this->getUnderlyingProblem()->setInternalState(x);

                    // Do step to get next observation
                    auto [observation, rewards, is_done] = this->getUnderlyingProblem()->step(joint_action);

                    auto y = this->getUnderlyingProblem()->getInternalState();

                    // Compute next sampled history
                    auto next_history = current_jhistory->expand(observation);

                    next_belief->addProbability(y, 1. / this->batch_size_);
                }

                new_occupancy_state->addProbability(jhistory, next_belief, 1. / this->batch_size_);

                // for (const auto &pair_history_pair_state_count : map_count)
                // {
                //     // Get the history
                //     auto jhistory = pair_history_pair_state_count.fisrt;

                //     // Build the sampled belief
                //     auto belief = std::make_shared<Belief>();

                //     // Get the number of sampled states in this history
                //     int num_sampled_states = pair_history_pair_state_count.second.size();

                //     for (const auto &pair_state_count : pair_history_pair_state_count.second)
                //     {
                //         belief->addProbability(pair_state_count.first, 1. / num_sampled_states);
                //     }

                //     new_occupancy_state->addProbability(jhistory, belief, num_sampled_states / this->num_samples_);
                // }
            }

            return new_occupancy_state;

            for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
            {
                auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, decision_rule, t);

                for (const auto &belief : occupancy_state->getFullyUncompressedOccupancy()->getBeliefsAt(joint_history))
                {
                    for (auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
                    {
                        auto next_joint_history = joint_history->expand(joint_observation->toObservation());
                        auto next_belief = this->getUnderlyingBeliefMDP()->nextBelief(belief, joint_action, joint_observation->toObservation(), t);

                        // p(o') = p(o) * p(z | b, a)
                        double proba_next_history = occupancy_state->getFullyUncompressedOccupancy()->getProbability(joint_history, belief) * this->getUnderlyingBeliefMDP()->getObservationProbability(belief, joint_action, next_belief->toBelief(), joint_observation->toObservation(), t);

                        if (proba_next_history > 0)
                        {
                            // Build fully uncompressed occupancy state*
                            new_fully_uncompressed_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief->toBelief(), proba_next_history);

                            // Build one step left uncompressed occupancy state
                            auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history);

                            auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
                            new_one_step_left_compressed_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), proba_next_history);

                            // Update next history labels
                            new_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                        }
                    }
                }
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
