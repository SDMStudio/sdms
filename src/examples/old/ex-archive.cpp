#include <fstream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/serialization/nvp.hpp>

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

    std::string filename;

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
        // Construct OccupancyMDP using parser
        std::cout << "#> Parsing file \"" << filename << "\"\n";

		using TActionDescriptor = number;
		using TStateDescriptor = HistoryTree_p<number>;

		using TState = OccupancyState<number, JointHistoryTree_p<number>>;
		using TAction = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;

		number horizon = 3;
		double discount = 1.0, error = 0.1, trial = 1000;

		std::shared_ptr<SolvableByHSVI<TState, TAction>> omdp_world = std::make_shared<OccupancyMDP<TState,TAction>>(filename, horizon);

        // // Set params in the environment
        omdp_world->getUnderlyingProblem()->setDiscount(discount);
        omdp_world->getUnderlyingProblem()->setPlanningHorizon(horizon);

        // Instanciate initializers
        auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();
        auto ub_init = std::make_shared<MaxInitializer<TState, TAction>>();

        // Instanciate the max-plan representation of the lower bound
        //auto lower_bound = std::make_shared<MaxPlanValueFunction<TState, TAction>>(omdp_world, horizon, lb_init); //
        auto lower_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, lb_init);
        auto copy_lower_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, lb_init);

        // Instanciate the Tabular version for the upper bound
        auto upper_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, ub_init);

        auto algo = std::make_shared<HSVI<TState, TAction>>(omdp_world, lower_bound, upper_bound, horizon, error, trial, "");

        algo->initialize();
        algo->solve();

        auto to_be_saved = std::static_pointer_cast<MappedValueFunction<TState, TAction>>(algo->getLowerBound());

        std::cout << *to_be_saved << std::endl;

        algo->getLowerBound()->save("mdp_value.bin");

        copy_lower_bound->load("mdp_value.bin");
        std::cout << *copy_lower_bound << std::endl;
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

    return 0;
}
