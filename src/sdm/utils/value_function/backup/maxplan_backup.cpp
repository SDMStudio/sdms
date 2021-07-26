#include <sdm/utils/value_function/backup/maxplan_backup.hpp>

#include <sdm/world/base/pomdp_interface.hpp>

#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/utils/value_function/hyperplan_value_function.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/serial_occupancy_state.hpp>
#include <sdm/core/state/belief_state.hpp>

// #include <sdm/core/action/decision_rule.hpp>
// #include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{

    MaxPlanBackup::MaxPlanBackup() {}

    MaxPlanBackup::MaxPlanBackup(const std::shared_ptr<SolvableByHSVI> &world) : BackupBase<std::shared_ptr<State>>(world) {}

    std::shared_ptr<State> MaxPlanBackup::backup(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {

        //Specialisation of backup for different state
        switch (state->getTypeState())
        {
        case TypeState::BELIEF_STATE:
            return this->setHyperplanBelief(vf, state, action, t);
            break;
        case TypeState::OCCUPANCY_STATE:
            return this->setHyperplanOccupancy(vf, state, action, t);
            break;
        default:
            throw sdm::exception::Exception("The state is not a compatible with maxplan backup !");
            break;
        }
    }

    std::shared_ptr<State> MaxPlanBackup::setHyperplanBelief(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &belief_state, const std::shared_ptr<Action> &action, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
        auto belief_mdp = std::static_pointer_cast<BeliefMDP>(this->world_);
        auto belief = belief_state->toState()->toBelief();

        //Creation of a new belief
        auto new_hyperplan = std::make_shared<Belief>();
        new_hyperplan->setDefaultValue(std::static_pointer_cast<HyperplanValueFunction>(vf)->getDefaultValue(t));

        // Go over all state in the current belief
        for (const auto &hidden_state : belief->getStates())
        {
            double tmp = 0.0;

            // Go over all hidden state reachable
            for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state->toState(), action->toAction(), t))
            {
                // Go over all observation reachable
                for (const auto &observation : under_pb->getReachableObservations(hidden_state->toState(), action->toAction(), next_hidden_state->toState(), t))
                {
                    //Determine the next belief conditionning to the action and observation
                    auto tempo_belief = belief_mdp->nextBelief(belief, action->toAction(), observation->toObservation(), t);

                    // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
                    tmp += vf->evaluate(tempo_belief, t + 1).first->toBelief()->getProbability(next_hidden_state) * under_pb->getDynamics(hidden_state->toState(), action->toAction(), next_hidden_state->toState(), observation->toObservation(), t);
                }
            }
            // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
            new_hyperplan->setProbability(hidden_state->toState(), under_pb->getReward(hidden_state->toState(), action->toAction(), t) + this->world_->getDiscount(t) * tmp);
        }
        new_hyperplan->finalize();
        return new_hyperplan;
    }

    std::shared_ptr<State> MaxPlanBackup::setHyperplanOccupancy(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &belief_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        try
        {
            auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
            auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(this->world_);

            auto occupancy = belief_state->toState()->toOccupancyState();

            // Create the new hyperplan
            auto new_hyperplan = std::make_shared<OccupancyState>(under_pb->getNumAgents());
            new_hyperplan->setDefaultValue(std::static_pointer_cast<HyperplanValueFunction>(vf)->getDefaultValue(t));

            // Get the next occupancy state associated to the decision rule
            auto next_occupancy_state = occupancy_mdp->nextOccupancyState(occupancy, decision_rule, nullptr, t);
            // Determine the best next hyperplan associated to the next occupancy state
            auto best_evaluate_occupancy_state = vf->evaluate(next_occupancy_state, t + 1).first->toOccupancyState();

            // Go over all joint history for the occupancy state
            for (const auto &jhistory : occupancy->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // Select the joint action
                auto action = occupancy_mdp->applyDecisionRule(occupancy->toOccupancyState(), occupancy->toOccupancyState()->getCompressedJointHistory(jhistory), decision_rule, t);

                // Create new belief
                auto new_belief = std::make_shared<Belief>();
                for (const auto &hidden_state : occupancy->getFullyUncompressedOccupancy()->getBeliefAt(jhistory)->getStates())
                {
                    double tmp = 0.0;
                    // Go over all hidden state reachable
                    for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state, action, t))
                    {
                        //Go over all observation reachable
                        for (const auto &observation : under_pb->getReachableObservations(hidden_state, action, next_hidden_state, t))
                        {
                            //Expand the current joint history
                            auto next_jhistory = jhistory->expand(observation->toObservation())->toJointHistory();

                            tmp += best_evaluate_occupancy_state->getProbability(next_jhistory, next_hidden_state) * under_pb->getDynamics(hidden_state, action, next_hidden_state, observation, t);
                        }
                    }
                    // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
                    new_belief->setProbability(hidden_state, under_pb->getReward(hidden_state, action, t) + this->world_->getDiscount(t) * tmp);
                }
                new_belief->finalize();
                new_hyperplan->setProbability(jhistory, new_belief, 1);
            }
            new_hyperplan->finalize();
            return new_hyperplan;
        }
        catch (const std::exception &e)
        {
            std::cerr << "MaxPlanBackup::setHyperplanOccupancy error " << e.what() << '\n';
            exit(-1);
        }
    }
}
