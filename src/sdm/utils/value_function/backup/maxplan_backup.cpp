#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/core/action/decision_rule.hpp>

#include <sdm/exception.hpp>

namespace sdm
{

    MaxPlanBackup::MaxPlanBackup(){}

    MaxPlanBackup::MaxPlanBackup(const std::shared_ptr<SolvableByHSVI>& world) : BackupBase<std::shared_ptr<State>>(world) {}

    std::shared_ptr<State> MaxPlanBackup::backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state,const std::shared_ptr<Action>&action, number t)
    {
        switch (state->getTypeState())
        {
        case TypeState::BELIEF_STATE :
            return this->backupBeliefState(vf,state,action,t);
            break;
        case TypeState ::OCCUPANCY_STATE :
            return this->backupOccupancyState(vf,state,action,t);
        default:
            throw sdm::exception::Exception("MaxPlan Backup with a state that is not a Belief State or an Occupancy State is impossible");
            break;
        }
    }

    std::shared_ptr<State> MaxPlanBackup::backupOccupancyState(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state,const std::shared_ptr<Action>&, number t)
    {
        double max = -std::numeric_limits<double>::max(), value; 

        //Get the best action
        auto action = vf->getBestAction(state,t);

        std::shared_ptr<State> max_next_step_hyperplan;

        // // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : vf->getSupport(t+1))
        {
            // Determine the max next hyperplan
            if(max < (value = state->toBelief()->operator^(this->setHyperplan(vf,state->toState(), hyperplan->toBelief(),action, t)->toBelief())))
            {
                max = value;
                max_next_step_hyperplan = hyperplan;
            }
        }
        return max_next_step_hyperplan;
    }

    std::shared_ptr<State> MaxPlanBackup::backupBeliefState(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state,const std::shared_ptr<Action>& action, number t)
    {

        auto belief_mdp = std::static_pointer_cast<BeliefMDP>(this->world_);
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());

        std::unordered_map<std::shared_ptr<Action>, std::unordered_map<std::shared_ptr<Observation>,std::shared_ptr<BeliefInterface>>> beta_a_o;
        std::unordered_map<std::shared_ptr<Action>, std::shared_ptr<BeliefInterface>> beta_a;

        // beta_a_o = argmax_alpha ( alpha * belief_t+1)
        for (const auto &action : *under_pb->getActionSpace(t))
        {
            beta_a_o.emplace(action->toAction(),std::unordered_map<std::shared_ptr<Observation>,std::shared_ptr<BeliefInterface>>());
            beta_a.emplace(action->toAction(),std::make_shared<Belief>(std::static_pointer_cast<HyperplanValueFunction>(vf)->getDefaultValue(t)));
            for (const auto &observation : *under_pb->getObservationSpace(t))
            {
                auto next_belief = belief_mdp->nextState(state->toBelief(), action->toAction() , observation->toObservation(),t);
                beta_a_o[action->toAction()].emplace(observation->toObservation(),vf->evaluate(next_belief, t + 1).first->toBelief());
            }
        }

        // \beta_a = R(s,a) + \gamma * \sum_{o, s'} [ \beta_{a,o}(s') * O(s', a, o) * T(s,a,s') ]
        for (const auto &action : *under_pb->getActionSpace(t))
        {
            for (const auto &state : *under_pb->getStateSpace(t))
            {
                double tmp = 0;
                for(const auto &next_state : under_pb->getReachableStates(state->toState(),action->toAction(),t) )
                {
                    for(const auto &observation : under_pb->getReachableObservations(state->toState(),action->toAction(),next_state->toState(),t))
                    {
                        tmp += beta_a_o[action->toAction()][observation->toObservation()]->getProbability(next_state) * under_pb->getDynamics(state->toState(), action->toAction(), next_state->toState(),observation->toObservation(),t);
                    }
                }
                beta_a[action->toAction()]->setProbability(state->toState(), under_pb->getReward(state->toState(), action->toAction(),t) + this->world_->getDiscount(t) * tmp);
            }
        }

        std::shared_ptr<Action> a_max;
        double current, max_v = -std::numeric_limits<double>::max();
        for (const auto &action : *under_pb->getActionSpace(t))
        {
            current = state->toBelief()->operator^(beta_a.at(action->toAction()));
            if (current > max_v)
            {
                max_v = current;
                a_max = action->toAction();
            }
        }
        auto new_plan = beta_a[a_max];
        return new_plan;
    }

    std::shared_ptr<State> MaxPlanBackup::setHyperplan(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &next_hyperplan, const std::shared_ptr<Action> &action, number t)
    {
        auto hyperplan_representation = std::static_pointer_cast<HyperplanValueFunction>(vf);
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());

        auto occupancy_state = state->toOccupancyState();
        auto decision_rule = action->toDecisionRule();

        std::shared_ptr<OccupancyStateInterface> new_hyperplan;

        switch (state->getTypeState())
        {
        case TypeState::OCCUPANCY_STATE : 
            // new_hyperplan = std::make_shared<OccupancyState>(hyperplan_representation->getDefaultValue(t));
            break;
        case TypeState::SERIAL_OCCUPANCY_STATE : 
            // new_hyperplan = std::make_shared<SerialOccupancyState>(hyperplan_representation->getDefaultValue(t));
            break;
        
        default:
            break;
        }

        // Go over all occupancy state
        for (const auto &uncompressed_joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
        {
            //Get information from uncompressed_s_o
            auto compressed_joint_history = occupancy_state->getCompressedJointHistory(uncompressed_joint_history);

            // Determine the history used for the decision rules
            std::shared_ptr<State> history_for_decision_rule;
            switch (state->getTypeState())
            {
            // If it's a occupancy state, we need the JOint histories
            case TypeState::OCCUPANCY_STATE : 
                history_for_decision_rule = compressed_joint_history->getIndividualHistories().template toJoint<State>();
                break;
            // If it's a Serial Occupancy State, we need the individal history
            case TypeState::SERIAL_OCCUPANCY_STATE : 
                // history_for_decision_rule = compressed_joint_history->getIndividualHistory(state->toSerial()->getCurrentAgentId());
                break;
            
            default:
                break;
            }

            // Get the serial action from the serial_decision_rule
            auto action = decision_rule->act(history_for_decision_rule);

            for (const auto &uncompressed_belief_state : occupancy_state->getFullyUncompressedOccupancy()->getBeliefsAt(uncompressed_joint_history))
            {
                // Add the reward of the hyperplan
                new_hyperplan->setProbability(uncompressed_joint_history,uncompressed_belief_state, this->world_->getReward(uncompressed_belief_state,action,t));



                // Est ce que cela est possible ? Je ne pense pas
                 
                // //Go ober all Reachable State
                // for (const auto &next_hidden_state : under_pb->getReachableStates(uncompressed_belief_state, action,t))
                // {
                //     //Go ober all Reachable Observation
                //     for (const auto &next_observation : under_pb->getReachableObservations(uncompressed_belief_state, action, next_hidden_state,t))
                //     {

                //         auto next_joint_history = compressed_joint_history->expand(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_observation))->toJointHistory();
                //         new_hyperplan->addProbability(uncompressed_joint_history, uncompressed_belief_state, this->world_->getDiscount(t) * under_pb->getDynamics(uncompressed_belief_state, action,next_hidden_state,next_observation,t) * next_hyperplan->getProbability(next_hidden_state, next_joint_history));
                //     }
                // }
            }
        }
        return new_hyperplan;
    }
}

