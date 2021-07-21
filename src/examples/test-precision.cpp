
#include <iomanip>
#include <iostream>

#include <boost/program_options.hpp>

#include <memory>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>

#include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

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
        options.add_options()("help", "produce help message")("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()("path,p", po::value<std::string>(&path)->default_value("tiger"), "the path to the problem to be solved")("belief_precision,r", po::value<double>(&belief_precision)->default_value(0.00001), "the precision of hierarchical private occupancy states (occupancy states) ")("ostate_precision", po::value<double>(&ostate_precision)->default_value(0.00001), "the precision of occupancy states (occupancy states)")("compress_precision,c", po::value<double>(&compress_precision)->default_value(0.001), "the precision for compression")("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")("error,e", po::value<double>(&error)->default_value(0.001), "the error")("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")("memory,m", po::value<number>(&memory)->default_value(0), "the memory. If 0 then infinite memory.")("trial,t", po::value<unsigned long>(&trial)->default_value(10000), "the maximum number of timesteps")("seed,s", po::value<int>(&seed)->default_value(1), "random seed")("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment")("compression", "do compression")("store_actions", "store_actions")("store_states", "store_states");

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
        clock_t t_begin = clock();

        // Parse file into MPOMDP
        auto mdp = sdm::parser::parse_file(path);
        horizon = horizon * mdp->getNumAgents();
        memory = memory * mdp->getNumAgents();
        mdp->setHorizon(horizon);
        mdp->setDiscount(discount);

        // Instanciate the problem

        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mdp);
        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, memory, vm.count("compression"), vm.count("store_states"), vm.count("store_actions"));

        auto serialized_mpomdp = std::make_shared<SerializedMPOMDP>(mdp);
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SerialOccupancyMDP>(serialized_mpomdp, memory, vm.count("compression"), vm.count("store_states"), vm.count("store_actions"));

        // ---------- Comment / Uncomment this section to enable solving with HSVI ----------
        //
        auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
        auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

        auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi_mdp);
        auto action_maxplan = std::make_shared<ActionVFMaxplan>(hsvi_mdp);
        auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(hsvi_mdp);
        auto action_sawtooth_lp = std::make_shared<ActionVFSawtoothLP>(hsvi_mdp, TypeOfResolution::IloIfThenResolution, 0, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);

        // Instanciate Initializer
        auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
        auto init_ub = std::make_shared<POMDPInitializer>(hsvi_mdp, "");

        std::shared_ptr<ValueFunction> lb, ub;
        // Instanciate value functions
        if (vm.count("store_states") && vm.count("store_actions"))
        {
            lb = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular);
            ub = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);
            // lb = std::make_shared<HyperplanValueFunction>(mdp->getHorizon(), init_lb, maxplan_backup, action_maxplan);
            // ub = std::make_shared<PointSetValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);
            // lb = std::make_shared<HyperplanValueFunction>(mdp->getHorizon(), init_lb, maxplan_backup, action_maxplan_lp);
            // ub = std::make_shared<PointSetValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_sawtooth_lp);
        }
        else
        {
            lb = std::make_shared<TabularValueFunction2>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular);
            ub = std::make_shared<TabularValueFunction2>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);
            // lb = std::make_shared<HyperplanValueFunction>(mdp->getHorizon(), init_lb, maxplan_backup, action_maxplan);
            // ub = std::make_shared<PointSetValueFunction2>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);
            // lb = std::make_shared<HyperplanValueFunction>(mdp->getHorizon(), init_lb, maxplan_backup, action_maxplan_lp);
            // ub = std::make_shared<PointSetValueFunction2>(mdp->getHorizon(), init_ub, tabular_backup, action_sawtooth_lp);
        }

        // Instanciate HSVI
        auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), error, trial);

        // Initialize and solve the problem
        algo->do_initialize();
        algo->do_solve();

        double TOTAL_TIME = ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

        // Save results in a CSV file
        algo->saveResults(name + "_test.csv", compress_precision);

        std::cout << "History Graph" << std::dynamic_pointer_cast<Tree<std::shared_ptr<Observation>>>(hsvi_mdp->initial_history_);

        // Display bounds
        // std::cout << *algo->getLowerBound() << std::endl;
        // std::cout << *algo->getUpperBound() << std::endl;

        // -----------------------------------------------------------------------------

        /* // ---------- Comment / Uncomment this section to enable test compression ---------- 

        auto state = hsvi_mdp->getInitialState();
        std::cout << "# State 0\n" << *state << std::endl;
        for (int i = 0; i < 5; i++)
        {
            auto action = std::static_pointer_cast<DiscreteSpace>(hsvi_mdp->getActionSpaceAt(state, i))->sample();

            std::cout << "# Action " << i << "\n" << *action << std::endl;
            state = hsvi_mdp->nextState(state, action->toAction());
            std::cout << "------------------" << std::endl;
            std::cout << "# State " << i + 1 << "\n" << *state << std::endl;
        }

        // ----------------------------------------------------------------------------- */

        // Log execution times
        std::cout << std::setprecision(5) << std::fixed;
        std::cout << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        std::cout << "| NAME\t\t\t\t|\tTIME\t\t|\tPERCENT\t\t|" << std::endl;
        std::cout << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        std::cout << "| TOTAL_TIME \t\t\t|\t" << TOTAL_TIME << " s\t|\t" << 100 * (TOTAL_TIME / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        std::cout << "| HSVI::TIME_INITIALIZATION \t|\t" << HSVI::TIME_INITIALIZATION << " s\t|\t" << 100 * (HSVI::TIME_INITIALIZATION / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| HSVI::TIME_IN_SELECT_ACTION \t|\t" << HSVI::TIME_IN_SELECT_ACTION << " s\t|\t" << 100 * (HSVI::TIME_IN_SELECT_ACTION / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| HSVI::TIME_IN_SELECT_STATE \t|\t" << HSVI::TIME_IN_SELECT_STATE << " s\t|\t" << 100 * (HSVI::TIME_IN_SELECT_STATE / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| HSVI::TIME_IN_UPDATE_LB \t|\t" << HSVI::TIME_IN_UPDATE_LB << " s\t|\t" << 100 * (HSVI::TIME_IN_UPDATE_LB / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| HSVI::TIME_IN_UPDATE_UB \t|\t" << HSVI::TIME_IN_UPDATE_UB << " s\t|\t" << 100 * (HSVI::TIME_IN_UPDATE_UB / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        std::cout << "| OccMDP::TIME_IN_GET_ACTION \t|\t" << SerialOccupancyMDP::TIME_IN_GET_ACTION << " s\t|\t" << 100 * (SerialOccupancyMDP::TIME_IN_GET_ACTION / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| OccMDP::TIME_IN_NEXT_STATE \t|\t" << SerialOccupancyMDP::TIME_IN_NEXT_STATE << " s\t|\t" << 100 * (SerialOccupancyMDP::TIME_IN_NEXT_STATE / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| OccMDP::TIME_IN_COMPRESS \t|\t" << SerialOccupancyMDP::TIME_IN_COMPRESS << " s\t|\t" << 100 * (SerialOccupancyMDP::TIME_IN_COMPRESS / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| OccMDP::TIME_IN_GET_REWARD \t|\t" << SerialOccupancyMDP::TIME_IN_GET_REWARD << " s\t|\t" << 100 * (SerialOccupancyMDP::TIME_IN_GET_REWARD / TOTAL_TIME) << " %\t|" << std::endl;
        std::cout << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

} // END main
