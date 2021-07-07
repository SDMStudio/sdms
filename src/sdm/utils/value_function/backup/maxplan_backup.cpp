#include <sdm/utils/value_function/backup/maxplan_backup.hpp>

#include <sdm/world/base/pomdp_interface.hpp>

#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/utils/value_function/hyperplan_value_function.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/belief_state.hpp>

// #include <sdm/core/action/decision_rule.hpp>
// #include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{

    MaxPlanBackup::MaxPlanBackup() {}

    MaxPlanBackup::MaxPlanBackup(const std::shared_ptr<SolvableByHSVI> &world) : BackupBase<std::shared_ptr<State>>(world) {}

    std::shared_ptr<State> MaxPlanBackup::backup(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        switch (state->getTypeState())
        {
        case TypeState::BELIEF_STATE:
            return this->setHyperplanBelief(vf, state, action, t);
            break;
        case TypeState::OCCUPANCY_STATE:
            return this->setHyperplanOccupancy(vf, state, action, t);
            break;
        case TypeState::SERIAL_OCCUPANCY_STATE:
            return this->setHyperplanOccupancy(vf, state, action, t);
            break;
        default:
            break;
        }
    }

    std::shared_ptr<State> MaxPlanBackup::setHyperplanBelief(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &belief_state, const std::shared_ptr<Action> &action, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
        auto belief_mdp = std::static_pointer_cast<BeliefMDP>(this->world_);
        auto belief = belief_state->toState()->toBelief();

        auto next_belief = std::make_shared<Belief>();
        next_belief->setDefaultValue(std::static_pointer_cast<HyperplanValueFunction>(vf)->getDefaultValue(t));

        for (const auto &state : belief->getStates())
        {
            double tmp = 0;

            for (const auto &next_state : under_pb->getReachableStates(state->toState(), action->toAction(), t))
            {
                for (const auto &observation : under_pb->getReachableObservations(state->toState(), action->toAction(), next_state->toState(), t))
                {
                    auto tempo_belief = belief_mdp->nextBelief(belief, action->toAction(), observation->toObservation(), t);
                    tmp += vf->evaluate(tempo_belief, t + 1).first->toBelief()->getProbability(next_state) * under_pb->getDynamics(state->toState(), action->toAction(), next_state->toState(), observation->toObservation(), t);
                }
            }
            next_belief->setProbability(state->toState(), under_pb->getReward(state->toState(), action->toAction(), t) + this->world_->getDiscount(t) * tmp);
        }
        return next_belief;
    }

    std::shared_ptr<State> MaxPlanBackup::setHyperplanOccupancy(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &belief_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(this->world_);

        auto occupancy = belief_state->toState()->toOccupancyState();
        
        auto next_occupancy = std::make_shared<OccupancyState>(under_pb->getNumAgents());
        next_occupancy->setDefaultValue(std::static_pointer_cast<HyperplanValueFunction>(vf)->getDefaultValue(t));

        for (const auto &jhistory : occupancy->getFullyUncompressedOccupancy()->getJointHistories())
        {
            // Get selected joint action
            auto action = occupancy_mdp->applyDecisionRule(occupancy->toOccupancyState(), jhistory, decision_rule, t);
            //Create next occupancy state
            auto next_occupancy_state = occupancy_mdp->nextOccupancyState(occupancy, decision_rule, nullptr, t);

            for (const auto &belief : occupancy->getFullyUncompressedOccupancy()->getBeliefsAt(jhistory))
            {
                double tmp = 0;

                for (const auto &observation : *under_pb->getObservationSpace(t))
                {
                    auto next_belief = occupancy_mdp->nextBelief(belief, action, observation->toObservation(), t);
                    auto next_jhistory = jhistory->expand(observation->toObservation())->toJointHistory();

                    tmp += vf->evaluate(next_occupancy_state, t + 1).first->toOccupancyState()->getProbability(next_jhistory, next_belief->toBelief()) * occupancy_mdp->getUnderlyingBeliefMDP()->getObservationProbability(belief, action, next_belief->toBelief(), observation->toObservation(), t);
                }
                next_occupancy->setProbability(jhistory, belief, occupancy_mdp->getRewardBelief(belief, action, t) + this->world_->getDiscount(t) * tmp);
            }
        }
        return next_occupancy;
    }
}
