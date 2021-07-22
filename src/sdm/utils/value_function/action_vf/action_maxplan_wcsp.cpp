#include <sdm/utils/value_function/action_vf/action_maxplan_wcsp.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/core/action/base_action.hpp>

namespace sdm
{
    ActionVFMaxplanWCSP::ActionVFMaxplanWCSP(){}

    ActionVFMaxplanWCSP::ActionVFMaxplanWCSP(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase(world) {}

    std::shared_ptr<Action> ActionVFMaxplanWCSP::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<Action> max_decision_rule;
        double max = -std::numeric_limits<double>::max();

        // Go over all hyperplan in the Support
        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            this->tmp_representation = hyperplan->toBelief();
            auto pair_action_value = this->createWCSPProblem(vf,state, t);

            // Select the Best Action 
            if (pair_action_value.second > max)
            {
                max_decision_rule = pair_action_value.first;
                max = pair_action_value.second;
            }
        }
        return max_decision_rule;
    }

    Pair<std::shared_ptr<Action>,double>  ActionVFMaxplanWCSP::createWCSPProblem(const std::shared_ptr<ValueFunction>& , const std::shared_ptr<State>& state, number t)
    {
        this->variables.clear();

        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(this->world_);

        auto occupancy_state = state->toOccupancyState();

        this->determineMaxValue(state,t);

        number index;

        double value;
        std::shared_ptr<Action> decision_rule;

        tb2init();              // must be call before setting specific ToulBar2 options and creating a model
        ToulBar2::verbose = -1; // change to 0 or higher values to see more trace information

        // create a problem with multiple variables
        std::shared_ptr<WeightedCSPSolver> wcsp_solver = std::shared_ptr<WeightedCSPSolver>(WeightedCSPSolver::makeWeightedCSPSolver(MAX_COST));

        // building variables a^i(u^i|o^i) for each agent i
        for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
        {
            for(const auto& ihistory : occupancy_state->getIndividualHistories(agent))
            {
                index = wcsp_solver->getWCSP()->makeEnumeratedVariable(this->getVarNameIndividualHistory(ihistory,agent), 0, under_pb->getActionSpace(agent,t)->toDiscreteSpace()->getNumItems()-1);
                this->variables.emplace(this->getVarNameIndividualHistory(ihistory,agent), index);
            }
        }
        // Creation of the cost network

        // Go over all joint histories 
        for(const auto &joint_history : occupancy_state->getJointHistories())
        {
            std::vector<Cost> costs;

            //Go over all joint action
            for(const auto &joint_action : *under_pb->getActionSpace(t))
            {
                costs.push_back(this->getCost(this->getValueAt(occupancy_state,joint_history,joint_action->toAction(),this->tmp_representation,t)));
            }
            wcsp_solver->getWCSP()->postBinaryConstraint(this->variables[this->getVarNameIndividualHistory(joint_history->getIndividualHistory(0),0)], this->variables[this->getVarNameIndividualHistory(joint_history->getIndividualHistory(1),1)], costs);
        }

        
        wcsp_solver->getWCSP()->sortConstraints(); // must be done before the search

        if (wcsp_solver->solve())
        {
            value = 0;
            std::vector<Value> sol;        // show optimal solution
            wcsp_solver->getSolution(sol); // cost optimum

            //Creation of the joint decision rule

            std::vector<std::vector<std::shared_ptr<Item>>> actions;
            std::vector<std::vector<std::shared_ptr<Item>>> joint_histories;

            // Go over each agent
            for (number agent = 0; agent < under_pb->getNumAgents(); agent++)
            {
                std::vector<std::shared_ptr<Item>> indiv_actions;
                std::vector<std::shared_ptr<Item>> indiv_histories;

                // Go over the individual history for a precise agent
                for(const auto &indiv_history : occupancy_state->getIndividualHistories(agent))
                { 
                    indiv_histories.push_back(indiv_history);

                    // Search which action is the solution
                    for(const auto& indiv_action : *under_pb->getActionSpace(agent,t))
                    {
                        if ( indiv_action->str() ==  under_pb->getActionSpace(agent,t)->toDiscreteSpace()->getItem(sol[this->variables[this->getVarNameIndividualHistory(indiv_history,agent)]])->str())
                        {
                           indiv_actions.push_back(indiv_action);
                        }
                    }
                }
                
                actions.push_back(indiv_actions);
                joint_histories.push_back(indiv_histories);
            }

            //Create JOint Deterministic Decision Rule
            decision_rule = std::make_shared<JointDeterministicDecisionRule>(joint_histories,actions);

            for(const auto& joint_history : occupancy_state->getJointHistories())
            {
                auto action = occupancy_mdp->applyDecisionRule(occupancy_state->toOccupancyState(), occupancy_state->toOccupancyState()->getCompressedJointHistory(joint_history), decision_rule, t);
                value += this->getValueAt(occupancy_state,joint_history,action,this->tmp_representation,t);
            }
        }
        else
        {
            std::cout << wcsp_solver->getWCSP() << std::endl;
            exit(-1);
        }

        return std::make_pair(decision_rule,value);
    }

    // A faire pour toute les fonctions maxplan car utilisé très souvent
    double ActionVFMaxplanWCSP::getValueAt(const std::shared_ptr<OccupancyStateInterface>& occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action,const std::shared_ptr<State>& next_hyperplan, number t)
    {
        // Compute \sum_{x} s(o,x)* discount * [ r(x,o) + \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
        double weight = 0.0;
        double factor;

        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());


        // Go over all hidden state in the belief conditionning to a joint history
        for(const auto &hidden_state : occupancy_state->getBeliefAt(joint_history)->getStates())
        {
            // Determine the reward for the hidden state and the action
            factor = under_pb->getReward(hidden_state,action->toAction(),t);

            // Go over all reachable next hidden state 
            for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state,action->toAction(),t))
            {
                // Go over all reachable observation 
                for(const auto &observation : under_pb->getReachableObservations(hidden_state,action->toAction(),next_hidden_state,t))
                {
                    // Determine the next joint history conditionning to the observation
                    auto next_joint_history = joint_history->expand(observation->toObservation())->toJointHistory();

                    factor += next_hyperplan->toOccupancyState()->getProbability(next_joint_history,next_hidden_state) * under_pb->getDynamics(hidden_state,action->toAction(),next_hidden_state,observation,t);
                }
            }
            weight +=  occupancy_state->getProbability(joint_history,hidden_state) * ActionVFBase::world_->getDiscount(t) * factor ;
        }
        return weight;
    }

    long ActionVFMaxplanWCSP::getCost(double value)
    {
        return (long)this->offset * (this->max - value);
    }

    void ActionVFMaxplanWCSP::determineMaxValue(const std::shared_ptr<State>& state, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        this->max = std::numeric_limits<double>::lowest();

        for (const auto& joint_history : occupancy_state->getJointHistories())
        {
            for (const auto &joint_action : *under_pb->getActionSpace(t))
            {
                this->max = std::max(max, this->getValueAt(occupancy_state,joint_history,joint_action->toAction(),this->tmp_representation,t));
            }
        }
    }
}