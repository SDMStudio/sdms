#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>
#include <sdm/core/state/occupancy_state_v2.hpp>

#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{

    MaxPlanBackup::MaxPlanBackup(){}

    MaxPlanBackup::MaxPlanBackup(const std::shared_ptr<SolvableByHSVI>& world)
    {
        this->world_ = world;
    }

    std::pair<double, std::shared_ptr<State>> MaxPlanBackup::getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        double current, max = -std::numeric_limits<double>::max();
        std::shared_ptr<BeliefInterface> alpha_vector;

        auto belief_state = state->toBelief();

        for (const auto &plan : vf->getSupport(t))
        {            
            auto belief_plan = plan->toBelief();

            current = belief_state->operator^(belief_plan);

            if (max < current)
            {
                max = current;
                alpha_vector = belief_plan;
            }
        }
        return {max, alpha_vector};
    }

    std::shared_ptr<Action> MaxPlanBackup::getBestAction(const std::shared_ptr<ValueFunction>& , const std::shared_ptr<State>& , number )
    {
        std::cout<<"Action"<<std::endl;
    }

    std::shared_ptr<State> MaxPlanBackup::backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        switch (state->getTypeState())
        {
        case TypeState::BELIEF_STATE :
            return this->backupBeliefState(vf,state,t);
            break;
        case TypeState ::OCCUPANCY_STATE :
            return this->backupOccupancyState(vf,state,t);
        default:
            throw sdm::exception::Exception("MaxPlan Backup with a state that is not a Belief State or an Occupancy State is impossible");
            break;
        }
    }

    std::shared_ptr<State> MaxPlanBackup::backupOccupancyState(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        auto occupancy_state = state->toBelief();

        std::shared_ptr<BeliefInterface> v_max;

        double value_max = -std::numeric_limits<double>::max(), tmp;

        // Go over the hyperplanes of decision step t+1
        for (const auto &next_hyperplan : vf->getSupport(t + 1))
        {
            // Go over all joint decision rules at occupancy occupancy_state
            for (const auto &joint_decision_rule : *this->world_->getActionSpaceAt(state,t))
            {
                const std::shared_ptr<BeliefInterface> new_hyperplan = this->getHyperplanAt(vf,state, next_hyperplan->toBelief(), joint_decision_rule->toAction(), t)->toBelief();
                if (value_max < (tmp = occupancy_state->operator^(new_hyperplan)))
                {
                    value_max = tmp;
                    v_max = new_hyperplan;
                }
            }
        }
        return v_max;
    }

    std::shared_ptr<State> MaxPlanBackup::backupBeliefState(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        auto belief_mdp = std::static_pointer_cast<BeliefMDP>(this->world_);
        auto under_pb = belief_mdp->getUnderlyingPOMDP();

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
                beta_a_o[action->toAction()].emplace(observation->toObservation(),this->getMaxAt(vf,state, t + 1).second->toBelief());
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
                beta_a[action->toAction()]->addProbability(state->toState(), under_pb->getReward(state->toState(), action->toAction(),t) + this->world_->getDiscount(t) * tmp);

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

    std::shared_ptr<State> MaxPlanBackup::getHyperplanAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &next_hyperplan, const std::shared_ptr<Action> &action, number t)
    {
        auto hyperplan_representation = std::static_pointer_cast<HyperplanValueFunction>(vf);
        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());

        auto occupancy_state = state->toOccupancyState();
        auto joint_decision_rule = action->toDecisionRule();

        std::shared_ptr<OccupancyStateInterface> new_hyperplan = std::make_shared<OccupancyState> (hyperplan_representation->getDefaultValue(t));


        // Go over all occupancy state
        for (const auto &uncompressed_joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
        {
            //Get information from uncompressed_s_o
            auto compressed_joint_history = occupancy_state->getCompressedJointHistory(uncompressed_joint_history);

            std::shared_ptr<State> joint_indiv_histories = std::make_shared<Joint<std::shared_ptr<State>>>(compressed_joint_history->JointHistoryTreeToJointState(compressed_joint_history->getIndividualHistories()));

            // Get the serial action from the serial_decision_rule
            auto action = joint_decision_rule->act(joint_indiv_histories);

            for (const auto &uncompressed_hidden_state : occupancy_state->getFullyUncompressedOccupancy()->getStatesAt(uncompressed_joint_history))
            {
                auto uncompressed_state = occupancy_state->HiddenStateAndJointHistoryToState(uncompressed_hidden_state,uncompressed_joint_history);
                // Add the reward of the hyperplan
                new_hyperplan->addProbability(uncompressed_state, this->world_->getReward(uncompressed_hidden_state,action,t));

                //Go ober all Reachable State
                for (const auto &next_hidden_state : under_pb->getReachableStates(uncompressed_hidden_state, action,t))
                {
                    //Go ober all Reachable Observation
                    for (const auto &next_observation : under_pb->getReachableObservations(uncompressed_hidden_state, action, next_hidden_state,t))
                    {

                        auto next_joint_history = compressed_joint_history->expand(std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_observation),std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action))->toJointHistory();
                        new_hyperplan->addProbability(uncompressed_state, this->world_->getDiscount(t) * under_pb->getDynamics(uncompressed_hidden_state, action,next_hidden_state,next_observation,t) * next_hyperplan->getProbability(occupancy_state->HiddenStateAndJointHistoryToState(next_hidden_state, next_joint_history)));
                    }
                }

            }
        }
        return new_hyperplan;
    }
}

