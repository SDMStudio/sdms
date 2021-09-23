#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/world/serialized_belief_mdp.hpp>
#include <sdm/core/state/serialized_belief_state.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/initializers.hpp>



using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    number horizon;
    number discount = 1;
    double error = 0.00001;
    number trials = 100;

	if (argc > 2)
	{
		filename = argv[1];
		horizon = std::atoi( argv[2] );
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{

		using TState = SerializedBeliefState;
		using TAction = number;
		using TObservation = Joint<number>;

		std::cout << "#> Parsing file \"" << filename << "\"\n";

		auto serial_belief_MDP = std::make_shared<SerializedBeliefMDP<TState, TAction, TObservation>>(filename);        

		serial_belief_MDP->getUnderlyingProblem()->setDiscount(discount);
		serial_belief_MDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		if (serial_belief_MDP->isSerialized())
		{
			horizon = horizon * serial_belief_MDP->getUnderlyingProblem()->getNumAgents();
		}

		// We will show how to expand an initial occupancy state and generate next ones
        int depth = 0, limit = 10;

        auto new_belief = serial_belief_MDP->getInitialState();

        std::cout << "#> Print occupancy state \n"
                  << new_belief << "\n";

        auto action = serial_belief_MDP->getActionSpaceAt(new_belief)->sample();
        std::cout << "#> Print action \n"
                  << action << "\n";

		auto obs = serial_belief_MDP->getUnderlyingProblem()->getObsSpace()->sample();
        std::cout << "#> Print obs \n"
                  << obs << "\n";

        do
        {
            depth++;
            std::cout << "#> Print depth \"" << depth << "\"\n";
			number ag_id = new_belief.getCurrentAgentId();

            // Compute the next compressed occupancy state
			TState next_belief;

			if (ag_id != serial_belief_MDP->getUnderlyingProblem()->getNumAgents() - 1)
			{
				next_belief = serial_belief_MDP->nextStateSerialStep(new_belief,action);
			}
			else
			{
                next_belief = serial_belief_MDP->nextStateSerialLastAgent(new_belief, action, obs);
			}

			new_belief = next_belief;
            std::cout << "#> Print next belief state \n"
                      << new_belief << "\n";
            // Sample a decision rule
            action = serial_belief_MDP->getActionSpaceAt(new_belief)->sample();
            std::cout << "#> Print action \n"
                      << action << "\n";	

			obs = serial_belief_MDP->getUnderlyingProblem()->getObsSpace()->sample();
       		std::cout << "#> Print obs \n"
			   		  << obs << "\n";

        } while (depth < limit);
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main