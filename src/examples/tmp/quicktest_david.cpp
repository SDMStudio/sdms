
#include <iostream>

#include <boost/program_options.hpp>

#include <memory>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>

#include <sdm/algorithms/planning/hsvi.hpp>

#include <sdm/utils/struct/graph.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

#include <sdm/algorithms.hpp>

using namespace sdm;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    try
    {
        std::string path, formalism, name;
        unsigned long trial;
        number horizon, memory, batch_size;
        double error, discount, belief_precision, ostate_precision, compress_precision;
        int seed;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("path,p", po::value<std::string>(&path)->default_value("tiger"), "the path to the problem to be solved")
        ("belief_precision,r", po::value<double>(&belief_precision)->default_value(0.00001), "the precision of hierarchical private occupancy states (occupancy states) ")
        ("ostate_precision", po::value<double>(&ostate_precision)->default_value(0.00001), "the precision of occupancy states (occupancy states)")
        ("compress_precision,c", po::value<double>(&compress_precision)->default_value(0.001), "the precision for compression")
        ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
        ("error,e", po::value<double>(&error)->default_value(0.001), "the error")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("memory,m", po::value<number>(&memory)->default_value(0), "the memory. If 0 then infinite memory.")
        ("trial,t", po::value<unsigned long>(&trial)->default_value(10000), "the maximum number of timesteps")
        ("seed,s", po::value<int>(&seed)->default_value(1), "random seed")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment")
        ("compression", "do compression")
        ("store_action_spaces", "store_action_spaces")
        ("store_states", "store_states");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a path with specified algorithms and configurations.");
        visible.add(options).add(config);

        po::options_description config_file_options;
        config_file_options.add(config);

        po::variables_map vm;
        try
        {
            po::store(po::command_line_parser(argc, argv).options(visible).run(), vm);
            po::notify(vm);
            if (vm.count("help"))
            {
                std::cout << visible << std::endl;
                return sdm::SUCCESS;
            }
        }
        catch (po::error &e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl;
            std::cerr << visible << std::endl;
            return sdm::ERROR_IN_COMMAND_LINE;
        }

        common::global_urng().seed(seed);

        // Set precision
        Belief::PRECISION = belief_precision;
        OccupancyState::PRECISION = ostate_precision;
        PrivateOccupancyState::PRECISION_COMPRESSION = compress_precision;

        std::cout << "Precision Belief =" << Belief::PRECISION << std::endl;
        std::cout << "Precision OccupancyState =" << OccupancyState::PRECISION << std::endl;
        std::cout << "Precision Compression =" << PrivateOccupancyState::PRECISION_COMPRESSION << std::endl;

        auto algo = algo::make("hsvi", path, "DecPOMDP", "tabular", "tabular", "Max", "Min", discount, error, horizon,trial, memory);

        // auto mdp = sdm::parser::parse_file(path);
        // mdp->setHorizon(horizon);
        // mdp->setDiscount(discount);

        // std::cout << "# Build OccupancyMDP" << std::endl;
        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mdp);
        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, memory, vm.count("compression"), vm.count("store_states"), vm.count("store_action_spaces"));

        // auto state = hsvi_mdp->getInitialState();
        // std::cout << "# State 0\n"
        //           << *state << std::endl;
        // for (int i = 0; i < 5; i++)
        // {
        //     auto action = std::static_pointer_cast<DiscreteSpace>(hsvi_mdp->getActionSpaceAt(state, i))->sample();

        //     std::cout << "# Action " << i << "\n"
        //               << *action << std::endl;
        //     state = hsvi_mdp->nextState(state, action->toAction());
        //     std::cout << "------------------" << std::endl;
        //     std::cout << "# State " << i + 1 << "\n"
        //               << *state << std::endl;
        // }

        // auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
        // auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

        // auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
        // auto init_ub = std::make_shared<MDPInitializer>(hsvi_mdp, "");

        // auto lb = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular);
        // auto ub = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);
        // auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), error, trial);

        // std::cout << "# Initialize Algo" << std::endl;
        algo->initialize();
        algo->solve();

        // algo->saveResults(name + "_test.csv", compress_precision);

        // // // std::cout << *algo->getLowerBound() << std::endl;
        // // // std::cout << *algo->getUpperBound() << std::endl;
        // // // std::cout << *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->state_space_.size() << std::endl;
        // std::cout << "PASSAGE IN NEXT STATE : " << OccupancyMDP::PASSAGE_IN_NEXT_STATE << std::endl;
        // std::cout << "MEAN SIZE STATE : " << OccupancyMDP::MEAN_SIZE_STATE << std::endl;
        // std::cout << "\nTOTAL TIME IN STEP : " << OccupancyMDP::TIME_IN_STEP << std::endl;
        // std::cout << "TOTAL TIME IN GET REWARD : " << OccupancyMDP::TIME_IN_GET_REWARD << std::endl;
        // std::cout << "TOTAL TIME IN GET ACTION : " << OccupancyMDP::TIME_IN_GET_ACTION << std::endl;
        // std::cout << "TOTAL TIME IN NEXT Occupancy STATE : " << OccupancyMDP::TIME_IN_NEXT_OSTATE << std::endl;
        // std::cout << "\nTOTAL TIME IN NEXT STATE : " << OccupancyMDP::TIME_IN_NEXT_STATE << std::endl;
        // std::cout << "TOTAL TIME IN COMPRESS : " << OccupancyMDP::TIME_IN_COMPRESS << std::endl;
        // std::cout << "\nTOTAL TIME IN Occupancy::operator== : " << OccupancyState::TIME_IN_EQUAL_OPERATOR << std::endl;
        // std::cout << "TOTAL TIME IN Occupancy::getProba : " << OccupancyState::TIME_IN_GET_PROBA << std::endl;
        // std::cout << "TOTAL TIME IN Occupancy::setProba : " << OccupancyState::TIME_IN_SET_PROBA << std::endl;
        // std::cout << "TOTAL TIME IN Occupancy::addProba : " << OccupancyState::TIME_IN_ADD_PROBA << std::endl;
        // std::cout << "TOTAL TIME IN Occupancy::finalize : " << OccupancyState::TIME_IN_FINALIZE << std::endl;
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

} // END main
