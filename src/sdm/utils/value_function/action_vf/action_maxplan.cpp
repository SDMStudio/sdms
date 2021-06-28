#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>

#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/belief_state.hpp>

#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/utils/value_function/hyperplan_value_function.hpp>

namespace sdm
{
    ActionVFMaxplan::ActionVFMaxplan(){}

    ActionVFMaxplan::ActionVFMaxplan(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase<std::shared_ptr<State>>(world) {}

    Pair<std::shared_ptr<Action>,std::shared_ptr<State>> ActionVFMaxplan::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        switch (state->getTypeState())
        {
        case TypeState::BELIEF_STATE :
            return this->selectBestActionBelief(vf,state,t);
            break;
        case TypeState ::OCCUPANCY_STATE :
            return this->selectBestActionOccupancy(vf,state,t);
        case TypeState ::SERIAL_OCCUPANCY_STATE :
            return this->selectBestActionOccupancy(vf,state,t);
        default:
            throw sdm::exception::Exception("MaxPlan Backup with a state that is not a Belief State or an Occupancy State is impossible");
            break;
        }
    }

    Pair<std::shared_ptr<Action>,std::shared_ptr<State>> ActionVFMaxplan::selectBestActionBelief(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());

        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max(), value;  
        std::shared_ptr<Action> action_;
        std::shared_ptr<State> best_hyperplan;

        for (const auto &action : *under_pb->getActionSpace(t))
        {
            auto hyperplan = this->selectHyperplanKnowingAction(vf,state,action->toAction(),t);

            if(argmax_global < (value = state->toBelief()->operator^(hyperplan->toBelief())) )
            {
                argmax_global = value;
                action_ = action->toAction();
                best_hyperplan = hyperplan;
            }
        }
        return std::make_pair(action_,best_hyperplan);
    }

    Pair<std::shared_ptr<Action>,std::shared_ptr<State>> ActionVFMaxplan::selectBestActionOccupancy(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        // Definie local Variable
        double argmax_global = 0;  
        std::shared_ptr<Action> action_;
        std::shared_ptr<State> best_hyperplan;

        std::shared_ptr<std::set<std::shared_ptr<HistoryInterface>>> histories_to_explore;
        std::shared_ptr<sdm::Space> action_to_explore;

        // switch (state->getTypeState())
        // {
        // case TypeState ::OCCUPANCY_STATE :
            
        //     action_ = std::make_shared<JointDeterministicDecisionRule>();
        //     histories_to_explore = std::make_shared<std::set<std::shared_ptr<HistoryInterface>>>(occupancy_state->getJointHistories());
        //     action_to_explore = under_pb->getActionSpace(t);

        // case TypeState ::SERIAL_OCCUPANCY_STATE :

        //     int agent =  std::dynamic_pointer_cast<SerializedMPOMDP>(under_pb)->getAgentId(t);
        //     action_ = std::make_shared<DeterministicDecisionRule>();
        //     histories_to_explore = std::make_shared<std::set<std::shared_ptr<HistoryInterface>>>(occupancy_state->getIndividualHistories(agent  ));
        //     action_to_explore = under_pb->getActionSpace(agent,t);
        // }


        for (const auto &history : *histories_to_explore)
        {
            double argmax_local = -std::numeric_limits<double>::max(); 
            double probability;
            
            switch (state->getTypeState())
            {
            case TypeState ::OCCUPANCY_STATE :
                
                probability = occupancy_state->getProbabilityOverJointHistory(history->toJointHistory());

            case TypeState ::SERIAL_OCCUPANCY_STATE :

                int agent =  std::dynamic_pointer_cast<SerializedMPOMDP>(under_pb)->getAgentId(t);
                probability = occupancy_state->getProbabilityOverIndividualHistories(agent,history);
                std::shared_ptr<State> private_serial_occupancy_state;// = serial_occupancy_state->getPrivateOccupancyState(agent,private_history)->toSerialOccupancyState();

            }
            
            // Go over all action possible for the current agent
            // for (const auto &action : *action_to_explore)
            // {
            //     // double hyperplan_and_value = this->evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(private_serial_occupancy_state, private_action->toAction(),next_hyperplan, t);
                
            //     if(argmax_local < probability*hyperplan_and_value.second)
            //     {
            //         argmax_local = hyperplan_and_value.second;
            //         action_->toDecisionRule()->setProbability(history,action->toAction(),1);
            //         best_hyperplan = hyperplan_and_value.first;
            //     }
            // }
        }
        return std::make_pair(action_,best_hyperplan);
    }

    std::shared_ptr<State> ActionVFMaxplan::selectHyperplanKnowingAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& belief_state,const std::shared_ptr<Action>&action, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
        auto belief = belief_state->toState()->toBelief();

        auto next_belief = std::make_shared<Belief>(std::static_pointer_cast<HyperplanValueFunction>(vf)->getDefaultValue(t));

        for (const auto &state : belief->getStates())
        {
            double tmp = 0;

            for(const auto &next_state : under_pb->getReachableStates(state->toState(),action->toAction(),t) )
            {
                for(const auto &observation : under_pb->getReachableObservations(state->toState(),action->toAction(),next_state->toState(),t))
                {
                    auto tempo_belief = BeliefMDP::nextBelief(under_pb,belief, action->toAction() , observation->toObservation(),t);
                    tmp += vf->evaluate(tempo_belief.first, t + 1).first->toBelief()->getProbability(next_state) * under_pb->getDynamics(state->toState(), action->toAction(), next_state->toState(),observation->toObservation(),t);
                }
            }
            next_belief->setProbability(state->toState(),under_pb->getReward(state->toState(), action->toAction(),t) + this->world_->getDiscount(t) * tmp);
        }
        return next_belief;
    }

}