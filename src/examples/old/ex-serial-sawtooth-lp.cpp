#include <cstdlib>
#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>

#include<sdm/algorithms.hpp>
#include <sdm/utils/value_function/sawtooth_vf_with_lp.hpp>


using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    number horizon;
    number discount = 1;
    double error = 0.00001;
    number trials = 100;

	TypeOfResolution type_of_resolution = TypeOfResolution::IloIfThenResolution;
	auto ValueBigM = 100;

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

		using TActionPrescriptor = DeterministicDecisionRule<HistoryTree_p<number>, number>;
		using TStatePrescriptor = SerialOccupancyState<SerialState, JointHistoryTree_p<number>>;;

		std::cout << "#> Parsing file \"" << filename << "\"\n";

		std::shared_ptr<SolvableByHSVI<TStatePrescriptor, TActionPrescriptor>>  soMDP = std::make_shared<SerialOccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(filename, horizon);        

		soMDP->getUnderlyingProblem()->setDiscount(discount);
		soMDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		if (soMDP->isSerial())
		{
			horizon = horizon * soMDP->getUnderlyingProblem()->getNumAgents();
		}

		// Instanciate initializers 
		auto lb_init = std::make_shared<MinInitializer<TStatePrescriptor, TActionPrescriptor>>();
		auto ub_init = sdm::makeInitializer<TStatePrescriptor, TActionPrescriptor>("MdpHsviInitializer");

		// Instanciate the Tabular version for the lower bound
		std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> lower_bound = std::make_shared<MappedValueFunction<TStatePrescriptor, TActionPrescriptor>>(soMDP, horizon, lb_init); 

		// Instanciate the Sawtooth version for the upper bound 
		std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> upper_bound = std::make_shared<SawtoothValueFunctionLP<TStatePrescriptor, TActionPrescriptor>>(soMDP, horizon, ub_init,type_of_resolution,ValueBigM );

		auto p_algo = std::make_shared<HSVI<TStatePrescriptor, TActionPrescriptor>>(soMDP, lower_bound, upper_bound, horizon, error, trials, "Example-SawtoothLP-SerialOccupancyMDP");

		//Initialization of HSVI
		p_algo->initialize();

		//Resolution of HSVI
		p_algo->solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main