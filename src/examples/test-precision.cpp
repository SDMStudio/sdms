
#include <iomanip>
#include <iostream>

#include <boost/program_options.hpp>

#include <memory>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>

#include <sdm/algorithms.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/hierarchical_occupancy_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp_serial.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

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
        std::string path, formalism, name, lower_bound, upper_bound, lb_init, ub_init;
        unsigned long trials;
        number horizon, memory, batch_size, freq_update_lb, freq_update_ub;
        double error, discount, p_b, p_o, p_c;
        int seed;
        bool compress, store_actions, store_states;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("path,p", po::value<std::string>(&path)->default_value("tiger"), "the path to the problem to be solved")
        ("formalism,f", po::value<std::string>(&formalism)->default_value("DecPOMDP"), "the precision of hierarchical private occupancy states (occupancy states) ")
        ("p_b", po::value<double>(&p_b)->default_value(0.0001), "the precision of hierarchical private occupancy states (occupancy states) ")
        ("p_o", po::value<double>(&p_o)->default_value(0.0001), "the precision of occupancy states (occupancy states)")
        ("p_c", po::value<double>(&p_c)->default_value(0.01), "the precision for compression")
        ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
        ("error,e", po::value<double>(&error)->default_value(0.1), "the error")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("memory,m", po::value<number>(&memory)->default_value(0), "the memory. If 0 then infinite memory.")
        ("trial,t", po::value<unsigned long>(&trials)->default_value(10000), "the maximum number of timesteps")
        ("seed,s", po::value<int>(&seed)->default_value(1), "random seed")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment")
        ("lower_bound", po::value<std::string>(&lower_bound)->default_value("tabular"), "the lower bound representation (HSVI, ValurIteration)")
        ("upper_bound", po::value<std::string>(&upper_bound)->default_value("tabular"), "the upper bound representation (HSVI)")
        ("lb_init", po::value<std::string>(&lb_init)->default_value("Min"), "the lower bound initialization method (HSVI, ValurIteration)")
        ("ub_init", po::value<std::string>(&ub_init)->default_value("Max"), "the upper bound initialization method (HSVI)")
        ("compress", po::value<bool>(&compress)->default_value(true), "If true, apply compression when required.")
        ("store_states", po::value<bool>(&store_states)->default_value(true), "If true, store the macro states when required.")
        ("store_actions", po::value<bool>(&store_actions)->default_value(true), "If true, store the macro actions when required.")
        ("freq_update_lb", po::value<number>(&freq_update_lb)->default_value(1), "the frequency for updating the lower bound")
        ("freq_update_ub", po::value<number>(&freq_update_ub)->default_value(1), "the frequency for updating the upper bound")
        ("display_graph_b", "Display the graph of beliefs")
        ("display_graph_o", "Display the graph of occupancy states");

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
        Belief::PRECISION = p_b;
        OccupancyState::PRECISION = p_o;
        PrivateOccupancyState::PRECISION_COMPRESSION = p_c;

        std::cout << "Precision Belief =" << Belief::PRECISION << std::endl;
        std::cout << "Precision OccupancyState =" << OccupancyState::PRECISION << std::endl;
        std::cout << "Precision Compression =" << PrivateOccupancyState::PRECISION_COMPRESSION << std::endl;

        // Parse file into MPOMDP
        auto mdp = sdm::parser::parse_file(path);
        mdp->setHorizon(horizon);
        mdp->setDiscount(discount);

        // Instanciate the problem

        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mdp);
        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, memory, vm.count("compression"), vm.count("store_states"), vm.count("store_actions"));

        // auto serialized_mpomdp = std::make_shared<SerializedMPOMDP>(mdp);
        // // auto serialized_mpomdp = mdp;
        // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SerialOccupancyMDP>(serialized_mpomdp, memory, vm.count("compression"), vm.count("store_states"), vm.count("store_actions"));
        // // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(serialized_mpomdp, memory, vm.count("compression"), vm.count("store_states"), vm.count("store_actions"));

        // // ---------- Comment / Uncomment this section to enable solving with HSVI ----------
        // //
        // auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
        // auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

        // auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi_mdp);
        // auto action_maxplan = std::make_shared<ActionVFMaxplan>(hsvi_mdp);
        // auto action_maxplan_serial = std::make_shared<ActionVFMaxplanSerial>(hsvi_mdp);
        // auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(hsvi_mdp);
        // auto action_sawtooth_lp = std::make_shared<ActionVFSawtoothLP>(hsvi_mdp, TypeOfResolution::IloIfThenResolution, 0, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);
        // auto action_sawtooth_lp_serial = std::make_shared<ActionVFSawtoothLPSerial>(hsvi_mdp, TypeOfResolution::IloIfThenResolution, 0, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);

        // // Instanciate Initializer
        // auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
        // auto init_ub = std::make_shared<MDPInitializer>(hsvi_mdp, "");

        // std::shared_ptr<ValueFunction> lb, ub;
        // // Instanciate value functions
        // if (vm.count("store_states") && vm.count("store_actions"))
        // {
        //     // lb = std::make_shared<TabularValueFunction>(serialized_mpomdp->getHorizon(), init_lb, tabular_backup, action_tabular);
        //     // ub = std::make_shared<TabularValueFunction>(serialized_mpomdp->getHorizon(), init_ub, tabular_backup, action_tabular);
        //     lb = std::make_shared<HyperplanValueFunction>(serialized_mpomdp->getHorizon(), init_lb, maxplan_backup, action_maxplan_serial);
        //     // ub = std::make_shared<PointSetValueFunction>(serialized_mpomdp->getHorizon(), init_ub, tabular_backup, action_tabular);
        //     // lb = std::make_shared<HyperplanValueFunction>(serialized_mpomdp->getHorizon(), init_lb, maxplan_backup, action_maxplan_lp);
        //     ub = std::make_shared<PointSetValueFunction>(serialized_mpomdp->getHorizon(), init_ub, tabular_backup, action_sawtooth_lp_serial);
        // }
        // else
        // {
        //     // lb = std::make_shared<TabularValueFunction2>(serialized_mpomdp->getHorizon(), init_lb, tabular_backup, action_tabular);
        //     // ub = std::make_shared<TabularValueFunction2>(serialized_mpomdp->getHorizon(), init_ub, tabular_backup, action_tabular);
        //     lb = std::make_shared<HyperplanValueFunction>(serialized_mpomdp->getHorizon(), init_lb, maxplan_backup, action_maxplan_serial);
        //     // ub = std::make_shared<PointSetValueFunction2>(serialized_mpomdp->getHorizon(), init_ub, tabular_backup, action_tabular);
        //     // lb = std::make_shared<HyperplanValueFunction>(serialized_mpomdp->getHorizon(), init_lb, maxplan_backup, action_maxplan_lp);
        //     ub = std::make_shared<PointSetValueFunction2>(serialized_mpomdp->getHorizon(), init_ub, tabular_backup, action_sawtooth_lp_serial);
        // }

        double MAX_RUNNING_TIME = 1800;
        std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();

        auto formalism_problem = algo::makeFormalism(path, formalism, discount, horizon, memory, compress, store_states, store_actions, batch_size);
        auto hsvi = algo::makeHSVI(formalism_problem, upper_bound, lower_bound, ub_init, lb_init, discount, error,
                                   formalism_problem->getUnderlyingProblem()->getHorizon(), trials,
                                   store_states, store_actions, (name == "") ? "tab_ext_ohsvi" : name, MAX_RUNNING_TIME,
                                   freq_update_lb, freq_update_ub, "", 1000, "", PAIRWISE, -1, NONE, -1);

        // Initialize and solve the problem
        hsvi->initialize();

        hsvi->solve();
        std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();

        double TOTAL_TIME = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count();

        // if (vm.count("display_graph_b"))
        // {
        //     std::cout << "Belief Graph" << std::endl;
        //     std::cout << *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->getMDPGraph() << std::endl;
        // }

        // if (vm.count("display_graph_o"))
        // {
        //     std::cout << "Belief Graph" << std::endl;
        //     std::cout << *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getMDPGraph() << std::endl;
        // }

        // Save results in a CSV file
        // std::static_pointer_cast<HSVI>(algo)->saveResults(name + "_test.csv", horizon);

        // auto casted_hsvi_mdp = std::static_pointer_cast<OccupancyMDP>(hsvi_mdp);
        // std::cout << "History Graph" << std::dynamic_pointer_cast<Tree<std::shared_ptr<Observation>>>(casted_hsvi_mdp->initial_history_)->str() << std::endl;

        // Display bounds
        // std::cout << *algo->getLowerBound() << std::endl;
        // std::cout << *algo->getUpperBound() << std::endl;

        // -----------------------------------------------------------------------------

        // ---------- Comment / Uncomment this section to enable test compression ----------

        // auto state = hsvi_mdp->getInitialState();
        // std::cout << "# State 0\n" << *state << std::endl;
        // for (int i = 0; i < 5; i++)
        // {
        //     auto action = std::static_pointer_cast<DiscreteSpace>(hsvi_mdp->getActionSpaceAt(state, i))->sample();

        //     std::cout << "# Action " << i << "\n" << *action << std::endl;
        //     state = hsvi_mdp->nextState(state, action->toAction(), i, algo);
        //     std::cout << "------------------" << std::endl;
        //     std::cout << "# State " << i + 1 << "\n" << *state << std::endl;
        // }

        // -----------------------------------------------------------------------------
        // Log execution times
        // std::ofstream ofs;
        // ofs.open(name + "_profiling.md", std::ios::out | std::ios::app);

        // ofs << std::setprecision(4) << std::fixed;
        // ofs << "## " << name << " - precision_compress=" << p_c << std::endl;
        // ofs << "| NAME\t\t\t\t|\tTIME\t\t|\tPERCENT\t\t|" << std::endl;
        // ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        // ofs << "| TOTAL_TIME \t\t\t|\t" << TOTAL_TIME << " s\t|\t" << 100 * (TOTAL_TIME / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        // ofs << "| HSVI::INITIALIZATION \t|\t" << HSVI::TIME_INITIALIZATION << " s\t|\t" << 100 * (HSVI::TIME_INITIALIZATION / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::SELECT_ACTION \t|\t" << HSVI::TIME_IN_SELECT_ACTION << " s\t|\t" << 100 * (HSVI::TIME_IN_SELECT_ACTION / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::SELECT_STATE \t|\t" << HSVI::TIME_IN_SELECT_STATE << " s\t|\t" << 100 * (HSVI::TIME_IN_SELECT_STATE / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::UPDATE_LB \t|\t" << HSVI::TIME_IN_UPDATE_LB << " s\t|\t" << 100 * (HSVI::TIME_IN_UPDATE_LB / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::PRUNING_LB \t|\t" << HSVI::TIME_IN_PRUNING_LB << " s\t|\t" << 100 * (HSVI::TIME_IN_PRUNING_LB / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::UPDATE_UB \t|\t" << HSVI::TIME_IN_UPDATE_UB << " s\t|\t" << 100 * (HSVI::TIME_IN_UPDATE_UB / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::PRUNING_UB \t|\t" << HSVI::TIME_IN_PRUNING_UB << " s\t|\t" << 100 * (HSVI::TIME_IN_PRUNING_UB / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| HSVI::DO_EXCESS \t|\t" << HSVI::TIME_IN_DO_EXCESS << " s\t|\t" << 100 * (HSVI::TIME_IN_DO_EXCESS / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        // ofs << "| OccMDP::GET_ACTION \t|\t" << OccupancyMDP::TIME_IN_GET_ACTION << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_GET_ACTION / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccMDP::NEXT_OSTATE \t|\t" << OccupancyMDP::TIME_IN_NEXT_OSTATE << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_NEXT_OSTATE / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccMDP::COMP_NEXT_STATE \t|\t" << OccupancyMDP::TIME_IN_NEXT_STATE << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_NEXT_STATE / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccMDP::COMPRESS \t|\t" << OccupancyMDP::TIME_IN_COMPRESS << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_COMPRESS / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccMDP::GET_REWARD \t|\t" << OccupancyMDP::TIME_IN_GET_REWARD << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_GET_REWARD / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccMDP::APPLY_DR \t|\t" << OccupancyMDP::TIME_IN_APPLY_DR << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_APPLY_DR / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
        // ofs << "| OccState::GET_PROBA \t|\t" << OccupancyState::TIME_IN_GET_PROBA << " s\t|\t" << 100 * (OccupancyState::TIME_IN_GET_PROBA / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccState::SET_PROBA \t|\t" << OccupancyState::TIME_IN_SET_PROBA << " s\t|\t" << 100 * (OccupancyState::TIME_IN_SET_PROBA / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccState::HASH \t|\t" << OccupancyState::TIME_IN_HASH << " s\t|\t" << 100 * (OccupancyState::TIME_IN_HASH / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccState::EQUAL \t|\t" << OccupancyState::TIME_IN_EQUAL_OPERATOR << " s\t|\t" << 100 * (OccupancyState::TIME_IN_EQUAL_OPERATOR / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccState::COMPRESS \t|\t" << OccupancyState::TIME_IN_COMPRESS << " s\t|\t" << 100 * (OccupancyState::TIME_IN_COMPRESS / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccState::FINALIZE \t|\t" << OccupancyState::TIME_IN_FINALIZE << " s\t|\t" << 100 * (OccupancyState::TIME_IN_FINALIZE / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| OccState::FINALIZE_PRI \t|\t" << OccupancyState::TIME_IN_FINALIZE_PRIVATE << " s\t|\t" << 100 * (OccupancyState::TIME_IN_FINALIZE_PRIVATE / TOTAL_TIME) << " %\t|" << std::endl;
        // ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;

        // ofs.close();
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

} // END main
