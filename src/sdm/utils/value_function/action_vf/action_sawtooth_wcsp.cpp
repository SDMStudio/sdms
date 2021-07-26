#include <sdm/utils/value_function/action_vf/action_sawtooth_wcsp.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

#include <sdm/world/base/mmdp_interface.hpp>
// #include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

// #include <sdm/core/action/base_action.hpp>

namespace sdm
{
    ActionVFSawtoothWCSP::ActionVFSawtoothWCSP(){}

    ActionVFSawtoothWCSP::ActionVFSawtoothWCSP(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase(world) {}

    std::shared_ptr<Action> ActionVFSawtoothWCSP::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        // std::shared_ptr<Action> best_action;
        // double min_value = std::numeric_limits<double>::max();

        // this->representation = {std::make_shared<MappedVector<std::shared_ptr<State>,double>>()};
        // this->determineMaxValue(vf,state,t);
        
        // if (vf->getSupport(t + 1).empty())
        // {
        //     // Resolution of the problem when the support of Point Set is empty
        //     this->representation = {std::make_shared<MappedVector<std::shared_ptr<State>,double>>()};
        //     auto [action,value] = this->createWCSPProblem(vf,state, t);

        //     best_action = action;
        //     std::cout<<"Tau "<<t<<std::endl;
        //     std::cout<<"Action "<<action->str()<<std::endl;
        //     std::cout<<"VALUE "<<value<<std::endl;
        // }
        // else
        // {
        //     // For the Relaxation version of Sawtooth, we go over all element in the Point Set
        //     for (const auto &point : std::static_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
        //     {
        //         auto vector = std::make_shared<MappedVector<std::shared_ptr<State>,double>>();
        //         vector->setValueAt(point.first,point.second);

        //         this->representation = vector;

        //         auto [action,value] = this->createWCSPProblem(vf,state, t);

        //         std::cout<<"Tau "<<t<<std::endl;
        //         std::cout<<"Action "<<action->str()<<std::endl;
        //         std::cout<<"VALUE "<<value<<std::endl;

        //         // // Verification of the Relaxation Contraint.
        //         // if(!this->RelaxationConstrainte(vf,state,action,value,t))
        //         // {
        //         //     // If the contraint is not verified , we used the decision rule previously stocked
        //         //     std::cout<<"Contrainte not verified"<<std::endl;
        //         //     action = this->state_linked_to_decision_rule.at(state);

        //         //     // The value is calculated by using the decision rule previously stockeed
        //         //     value = this->calculRelaxationContrainte(vf,state, action, t);
        //         // }

        //         // We take the best action with the minimum value
        //         if (min_value > value)
        //         {
        //             min_value = value;
        //             best_action = action;
        //         }
        //     }
        // }
        // // Save the best action associed to a state
        // // this->state_linked_to_decision_rule[state] = best_action;
        // return best_action;
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());

        std::shared_ptr<Action> best_action;
        double max_value = -std::numeric_limits<double>::max();


        if (vf->getSupport(t + 1).empty())
        {
            this->support_empty = true;

            auto [action,value] = this->createWCSPProblem(vf,state, t);
            best_action = action;
            max_value = value;
        }
        else
        {
            this->support_empty = false;

            // For the Relaxation version of Sawtooth, we go over all element in the Point Set
            for (const auto &point_AND_value : std::static_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
            {
                this->representation = point_AND_value;

                // We go over the support of the point i.e. we go over the history and the action
                auto decision_rule_associed = this->state_linked_to_decision_rule.at(point_AND_value.first)->toDecisionRule();

                double max_value_support = -std::numeric_limits<double>::max();
                std::shared_ptr<Action> best_action_support;

                // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
                for (const auto& next_joint_history_of_support : point_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy()->getJointHistories())
                {
                    // Search the action and next joint history which have a probability >0
                    for(const auto& action_of_support : *under_pb->getActionSpace(t))
                    {
                        if (decision_rule_associed->elementExist(next_joint_history_of_support->getIndividualHistories().toJoint<State>()))
                        {
                            // Associate the variable next joint history and the action to the support 
                            this->support_of_the_next_history = next_joint_history_of_support;
                            this->support_of_the_next_action = action_of_support->toAction();

                            //Resolve the WCSP problem
                            auto [action,value] = this->createWCSPProblem(vf,state, t);

                            // We take the best action with the minimum value
                            if (max_value_support < value)
                            {
                                max_value_support = value;
                                best_action_support = action->toAction();
                            }
                        }
                    }
                }
                // We take the best action with the minimum value
                if (max_value < max_value_support)
                {
                    max_value = max_value_support;
                    best_action = best_action_support;
                }
            }
        }
        // Save the best action associed to a state
        this->state_linked_to_decision_rule[state] = best_action;
        return best_action;
    }

    Pair<std::shared_ptr<Action>,double>  ActionVFSawtoothWCSP::createWCSPProblem(const std::shared_ptr<ValueFunction>& vf , const std::shared_ptr<State>& state, number t)
    {
        this->variables.clear();

        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(this->world_);

        auto occupancy_state = state->toOccupancyState();

        this->determineMaxValue(vf,state,t);

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
                costs.push_back(this->getCost(this->getValueAt(vf,occupancy_state,joint_history,joint_action->toAction(),nullptr,t)));
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
                value += this->getValueAt(vf,occupancy_state,joint_history,action,nullptr,t);
            }
        }
        else
        {
            // std::cout << "No solution found!" << std::endl;
            std::cout << wcsp_solver->getWCSP() << std::endl;
            exit(-1);
        }

        return std::make_pair(decision_rule,value);
    }

    double ActionVFSawtoothWCSP::getValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface>& occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action,const std::shared_ptr<State>& , number t)
    {
        double factor = 0.0;

        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());
        auto belief = occupancy_state->getBeliefAt(joint_history);

        auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(vf->getInitFunction());
        // Compute the value for the relaxation problem
        double init_value = relaxation->operator()(std::make_pair(belief, action), t);

        // Point set at t+1
        if(!support_empty)
        {
            // Take all information associated to the support i.e the point set and this value at t+1, the joint history associated to the point set 
            // and the action conditionning to the point set and the joint history

            auto point_AND_value_next_point_set = this->representation;
            // Next joint history conditionning to the point set
            auto next_joint_history = this->support_of_the_next_history->toJointHistory();
            // Action conditionning to the point set and next joint history
            auto action_of_support = this->support_of_the_next_action;

            const auto &next_one_step_uncompressed_occupancy_state = point_AND_value_next_point_set.first->toOccupancyState()->getOneStepUncompressedOccupancy();
            
            // Compute the value (v-f(point_set)) i.e. the current value of the point set minus the value at the initialisation of the point set
            double current_upper_bound = point_AND_value_next_point_set.second;
            double initial_upper_bound = vf->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1);

            double difference = current_upper_bound - initial_upper_bound;

            // Compute the denominator
            // Comment faire ? SAchant que mon support est next joint history et action maintenant ? 
            // double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_joint_history,next_hidden_state);
            double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_joint_history);
            // Dans les notes de hier, on a le denominateur en fonction du joint history et de l'action

            // auto joint_observation = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_joint_history->getLastObservation());

            if(action == action_of_support)
            {
                factor = difference/denominator;
            }

            // factor += (difference* under_pb->getDynamics(hidden_state,action,next_hidden_state,joint_observation,t))/denominator;


        }
        // Il faut aussi vérifier que 2 pair <historique,action> ne puissent pas donner le même next joint historique.
        return occupancy_state->getProbability(joint_history) * (init_value + factor);
    }

    long ActionVFSawtoothWCSP::getCost(double value)
    {
        return (long)this->offset * (this->max - value);
    }

    void ActionVFSawtoothWCSP::determineMaxValue(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State>& state, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        this->max = std::numeric_limits<double>::lowest();

        for (const auto& joint_history : occupancy_state->getJointHistories())
        {
            for (const auto &joint_action : *under_pb->getActionSpace(t))
            {
                this->max = std::max(max, this->getValueAt(vf,occupancy_state,joint_history,joint_action->toAction(),nullptr,t));
            }
        }
    }
}