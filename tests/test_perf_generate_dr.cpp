#include <memory>
#include <iostream>
#include <boost/program_options.hpp>

#include <sdm/exception.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/space/function_space.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    number horizon, truncation, num_agents;
    std::vector<number> list_num_histories, list_num_action;

    po::options_description options("Options");
    options.add_options()
    ("num_agents,n", po::value<number>(&num_agents)->default_value(2), "the number of agents")
    ("action,a", po::value<std::vector<number>>(&list_num_action)->multitoken(), "list of number of observation")
    ("list_num_histories,h", po::value<std::vector<number>>(&list_num_histories)->multitoken(), "list of number of individual histories");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    po::notify(vm);

    std::cout << "Num Agents : " << num_agents << std::endl;
    std::cout << "Histories : " << list_num_histories << std::endl;
    std::cout << "Actions : " << list_num_action << std::endl;

    std::vector<std::shared_ptr<Space>> history_spaces, action_spaces;
    for (number agent_id = 0; agent_id < num_agents; agent_id++)
    {
        std::vector<std::shared_ptr<Item>> history_list, action_list;
        for (number history = 0; history < list_num_histories[agent_id]; history++)
        {
            history_list.push_back(std::make_shared<DiscreteState>(history));
        }
        for (number action = 0; action < list_num_action[agent_id]; action++)
        {
            action_list.push_back(std::make_shared<DiscreteAction>(action));
        }
        history_spaces.push_back(std::make_shared<DiscreteSpace>(history_list));
        action_spaces.push_back(std::make_shared<DiscreteSpace>(action_list));
    }
    auto history_space = std::make_shared<MultiDiscreteSpace>(history_spaces, false);
    auto action_space = std::make_shared<MultiDiscreteSpace>(action_spaces, false);

    std::vector<std::shared_ptr<Space>> vector_indiv_space;
    for (int agent_id = 0; agent_id < num_agents; agent_id++)
    {
        // Get history space of agent i
        auto history_space_i = std::static_pointer_cast<Joint<std::shared_ptr<Space>>>(history_space)->get(agent_id);
        // Get action space of agent i
        auto action_space_i = std::static_pointer_cast<Joint<std::shared_ptr<Space>>>(action_space)->get(agent_id);
        // Add individual decision rule space of agent i
        vector_indiv_space.push_back(std::make_shared<FunctionSpace<DeterministicDecisionRule>>(history_space_i, action_space_i, false));
    }

    // Now we can return a discrete space of all joint decision rules
    auto multi_dspace = std::make_shared<MultiDiscreteSpace>(vector_indiv_space, false);

    std::cout << "--- START DECISION RULE GENERATION ---" << std::endl;

    int i = 0;
    for (const auto &joint_dr : *multi_dspace)
    {
        std::cout << "\033[1m\033[36m  <decision id=\"" << i << "\" addr=\"" << joint_dr->str() << "\" /> \033[0m" << std::endl;
        i++;
    }

    std::cout << "--- END DECISION RULE GENERATION ---" << std::endl;

} // END main