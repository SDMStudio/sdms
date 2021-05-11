#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>

#include<sdm/algorithms.hpp>
#include <sdm/utils/value_function/sawtooth_vf_with_lp.hpp>


using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    number horizon = 1;
    number discount = 1;
    double error = 0.00001;
    number trials = 10;

	if (argc > 1)
	{
		filename = argv[1];
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{

		std::cout << "----- Usage : class Joint ( sdm/core/state/jhistory_tree.hpp ) ---------" << std::endl
				<< std::endl;

		using TObservation = number;

		number num_agents = 2, max_depth = 3;

		JointHistoryTree_p<TObservation> jhistory(new JointHistoryTree<TObservation>(num_agents, max_depth));

		// Get basic elements of joint histories
		std::cout << "\n--- 1) Basic access" << std::endl;

		std::cout << "\n#> Number of agents = " << jhistory->getNumAgents() << std::endl;
		std::cout << "#> Horizon = " << jhistory->getHorizon() << std::endl; // equivalent to jhistory->getDepth()
		std::cout << "#> MaxDepth = " << jhistory->getMaxDepth() << std::endl;
		std::cout << "#> Initial Joint history : " << *jhistory << std::endl;

		// How to expand a joint history
		std::cout << "\n--- 2) Instanciate and expand a joint history" << std::endl;

		// List of joint observation for the example
		std::vector<Joint<TObservation>> list_joint_obs = {{1, 0}, {0, 0}, {2, 2}, {2, 1}};
		for (const auto &joint_obs : list_joint_obs)
		{
			std::cout << "\n#> Expand with observation " << joint_obs << std::endl;
			jhistory = jhistory->expand(joint_obs);
			std::cout << "#> Expanded joint history --> " << *jhistory << std::endl;
		}

		std::cout<<"\n get Last Observation "<<jhistory->getData()<<std::endl;

		std::cout<<"\n Get Parent of Joint History "<<*jhistory->getOrigin()<<std::endl;
		
		std::cout<<"\n jhistory->getChildren().size()"<<jhistory->getChildren().size()<<std::endl;
		for(const auto &jh : jhistory->getChildren())
		{
			std::cout<<"\n Get Children of Joint History "<<*jh<<std::endl;
		}

		// How to access individual histories and expand them
		std::cout << "\n--- 3) Access individual histories" << std::endl;

		std::cout << "\n#> List of pointer on individual histories = " << jhistory->getIndividualHistories() << std::endl;

		for (number agent_id = 0; agent_id < jhistory->getNumAgents(); ++agent_id)
		{
			std::cout << "#> IndividualHistory(" << agent_id << ") = " << *jhistory->getIndividualHistory(agent_id) << std::endl; // equivalent to jhistory->get(agent_id)
		}


	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main