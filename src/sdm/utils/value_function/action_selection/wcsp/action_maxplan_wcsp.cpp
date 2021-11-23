#include <sdm/utils/value_function/action_selection/wcsp/action_maxplan_wcsp.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/core/action/base_action.hpp>

namespace sdm
{
    ActionSelectionMaxplanWCSP::ActionSelectionMaxplanWCSP() {}

    ActionSelectionMaxplanWCSP::ActionSelectionMaxplanWCSP(const std::shared_ptr<SolvableByDP> &world) : MaxPlanSelectionBase(world) {}

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanWCSP::computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &hyperplane, number t)
    {
        return this->createAndSolveWCSP(value_function, state, hyperplane, t);
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanWCSP::createAndSolveWCSP(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &hyperplane, number t)
    {
        this->variables.clear();

        auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(getWorld());
        auto underlying_problem = std::dynamic_pointer_cast<MMDPInterface>(occupancy_mdp->getUnderlyingMPOMDP());

        auto occupancy_state = state->toOccupancyState();

        this->determineMaxValue(value_function, occupancy_state, hyperplane, t);

        number index;

        double value;
        std::shared_ptr<Action> decision_rule;

        tb2init();              // must be call before setting specific ToulBar2 options and creating a model
        ToulBar2::verbose = -1; // change to 0 or higher values to see more trace information

        // create a problem with multiple variables
        std::shared_ptr<WeightedCSPSolver> wcsp_solver = std::shared_ptr<WeightedCSPSolver>(WeightedCSPSolver::makeWeightedCSPSolver(MAX_COST));

        // building variables a^i(u^i|o^i) for each agent i
        for (number agent = 0; agent < underlying_problem->getNumAgents(); ++agent)
        {
            for (const auto &ihistory : occupancy_state->getIndividualHistories(agent))
            {
                index = wcsp_solver->getWCSP()->makeEnumeratedVariable(this->getVarNameIndividualHistory(ihistory, agent), 0, underlying_problem->getActionSpace(agent, t)->toDiscreteSpace()->getNumItems() - 1);
                this->variables.emplace(this->getVarNameIndividualHistory(ihistory, agent), index);
            }
        }
        // Creation of the cost network

        // Go over all joint histories
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            std::vector<Cost> costs;

            //Go over all joint action
            for (const auto &joint_action : *underlying_problem->getActionSpace(t))
            {
                costs.push_back(this->getCost(this->getWeight(value_function, occupancy_state, joint_history, joint_action->toAction(), hyperplane, t)));
            }
            wcsp_solver->getWCSP()->postBinaryConstraint(this->variables[this->getVarNameIndividualHistory(joint_history->getIndividualHistory(0), 0)], this->variables[this->getVarNameIndividualHistory(joint_history->getIndividualHistory(1), 1)], costs);
        }

        wcsp_solver->getWCSP()->sortConstraints(); // must be done before the search

        if (wcsp_solver->solve())
        {
            value = 0.0;
            std::vector<Value> sol;        // show optimal solution
            wcsp_solver->getSolution(sol); // cost optimum

            //Creation of the joint decision rule

            std::vector<std::vector<std::shared_ptr<Item>>> actions;
            std::vector<std::vector<std::shared_ptr<Item>>> joint_histories;

            // Go over each agent
            for (number agent = 0; agent < underlying_problem->getNumAgents(); agent++)
            {
                std::vector<std::shared_ptr<Item>> indiv_actions;
                std::vector<std::shared_ptr<Item>> indiv_histories;

                // Go over the individual history for a precise agent
                for (const auto &indiv_history : occupancy_state->getIndividualHistories(agent))
                {
                    indiv_histories.push_back(indiv_history);

                    // Search which action is the solution
                    for (const auto &indiv_action : *underlying_problem->getActionSpace(agent, t))
                    {
                        if (indiv_action->str() == underlying_problem->getActionSpace(agent, t)->toDiscreteSpace()->getItem(sol[this->variables[this->getVarNameIndividualHistory(indiv_history, agent)]])->str())
                        {
                            indiv_actions.push_back(indiv_action);
                        }
                    }
                }

                actions.push_back(indiv_actions);
                joint_histories.push_back(indiv_histories);
            }

            //Create JOint Deterministic Decision Rule
            decision_rule = std::make_shared<JointDeterministicDecisionRule>(joint_histories, actions);

            for (const auto &joint_history : occupancy_state->getJointHistories())
            {
                auto action = occupancy_mdp->applyDecisionRule(occupancy_state->toOccupancyState(), occupancy_state->toOccupancyState()->getCompressedJointHistory(joint_history), decision_rule, t);
                value += this->getWeight(value_function, occupancy_state, joint_history, action, hyperplane, t);
            }
        }
        else
        {
            exit(-1);
        }

        return std::make_pair(decision_rule, value);
    }

    // double ActionSelectionMaxplanWCSP::getWeight(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, const std::shared_ptr<Action> action, number t)
    // {
    //     // Compute \sum_{x} s(o,x)* [ r(x,u) + discount * \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
    //     double weight = 0.0;

    //     // Go over all hidden state in the belief conditionning to a joint history
    //     for (const auto &state : occupancy_state->getBeliefAt(joint_history)->getStates())
    //     {
    //         weight += occupancy_state->getProbability(joint_history, state) * std::dynamic_pointer_cast<PWLCValueFunctionInterface>(value_function)->getBeta(hyperplan, state, joint_history, action, t);
    //     }
    //     return weight;
    // }

    long ActionSelectionMaxplanWCSP::getCost(double value)
    {
        return (long)this->offset * (this->max - value);
    }

    void ActionSelectionMaxplanWCSP::determineMaxValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<BeliefInterface> &hyperplane, number t)
    {
        auto mpomdp = std::dynamic_pointer_cast<MPOMDPInterface>(this->getWorld()->getUnderlyingProblem());

        this->max = std::numeric_limits<double>::lowest();

        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            for (const auto &action : *mpomdp->getActionSpace(t))
            {
                this->max = std::max(max, this->getWeight(value_function, occupancy_state, joint_history, action->toAction(), hyperplane, t));
            }
        }
    }
}