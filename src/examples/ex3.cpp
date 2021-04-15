/**
 * @file ex3.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that show how to use genericity to generate all discrete function of discrete function
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <cassert>
#include <sdm/worlds.hpp>
#include <sdm/spaces.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/core/action/discrete_action.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    char const *filename;

    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    try
    {
        // Defines the different types 
        using TObservation = number;
        using TState = number;

        using TActionDescriptor = number;
        using TStateDescriptor = HistoryTree_p<TObservation>;

        using TActionPrescriptor = DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
        using TStatePrescriptor = JointHistoryTree_p<TObservation>;

        using TPrescriptorN2Action = DeterministicDecisionRule<TStatePrescriptor, Joint<TActionPrescriptor>>;

        
		auto dpomdp_world = sdm::parser::parse_file(filename);


        number num_agents = dpomdp_world->getNumAgents();

        // Instanciate joint history of max depth 3
        TStatePrescriptor j_history = std::make_shared<JointHistoryTree<TObservation>>(num_agents, 3);

        // We will show how to expand a joint history and generate all joint decision rules 
        for (int trial = 0; trial < 100; trial++)
        {
            // Sample a new joint observation and expand the history given this new observation
            Joint<TObservation> new_obs = dpomdp_world->getObsSpace()->sample();
            j_history = j_history->expand(new_obs);

            // Instanciate the list (one by agent) of list of decision rule
            std::vector<std::vector<TActionPrescriptor>> vect_i_dr = {};
            for (int ag_id = 0; ag_id < num_agents; ag_id++)
            {
                // Generate all individual decision rules for agent 'ag_id' 
                FunctionSpace<TActionPrescriptor> f_indiv_dr_space({j_history->getIndividualHistory(ag_id)}, dpomdp_world->getActionSpace()->getSpace(ag_id)->getAll());
                vect_i_dr.push_back(f_indiv_dr_space.getAll());
            }

            // Now that we stored all possible decision rule for each agent, we can get all joint decision rules for the controler (map a joint history to a joint individual decision rule)
            FunctionSpace<TPrescriptorN2Action> f_joint_dr_space({j_history}, MultiDiscreteSpace(vect_i_dr).getAll());
            std::cout << "Joint Decision Rules : T = " << trial << std::endl;
            for (auto val : f_joint_dr_space.getAll())
            {
                std::cout << val << std::endl;
            }
        }

        // Display the joint history tree
        std::cout << *j_history->getOrigin() << std::endl;

    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}
