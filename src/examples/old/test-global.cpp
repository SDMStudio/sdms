
// #include <iomanip>
// #include <iostream>

// #include <boost/program_options.hpp>

// #include <memory>
// #include <sdm/config.hpp>
// #include <sdm/exception.hpp>

// #include <sdm/algorithms/planning/hsvi.hpp>

// #include <sdm/core/action/action.hpp>
// #include <sdm/core/base_item.hpp>
// #include <sdm/core/action/base_action.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/base_state.hpp>

// #include <sdm/world/solvable_by_mdp.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/hierarchical_occupancy_mdp.hpp>
// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/world/serial_occupancy_mdp.hpp>
// #include <sdm/world/serial_mpomdp.hpp>
// #include <sdm/world/hierarchical_mpomdp.hpp>
// #include <sdm/parser/parser.hpp>

// #include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
// #include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

// #include <sdm/utils/value_function/backup/maxplan_backup.hpp>
// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

// #include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
// #include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp_serial.hpp>
// #include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>
// #include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/utils/value_function/point_set_value_function.hpp>
// #include <sdm/utils/value_function/hyperplan_value_function.hpp>

// #include <sdm/core/state/belief_state.hpp>
// #include <sdm/core/state/occupancy_state.hpp>
// #include <sdm/core/state/private_occupancy_state.hpp>
// #include <sdm/algorithms.hpp>

// using namespace sdm;
// namespace po = boost::program_options;

// int main(int argc, char **argv)
// {
//     try
//     {
//         std::string path,problem, formalism, name, lower_bound, upper_bound, lb_init, ub_init, type_lb_pruning, type_ub_pruning, sawtooth_type_of_resolution, sawtooth_type_of_linear_program, save_path;
//         unsigned long trials, time_max;
//         number horizon, memory, batch_size;
//         double error, discount, belief_precision, ostate_precision, compress_precision;
//         int seed, freq_lb_pruning, freq_ub_pruning, sawtooth_BigM;
//         bool compression, store_actions, store_states;

//         po::options_description options("Options");
//         options.add_options()("help", "produce help message")("test", "test the policy found");

//         po::options_description config("Configuration");
//         config.add_options()("path", po::value<std::string>(&path)->default_value("../data/world/dpomdp/"), "the path to the problem to be solved")
//                             ("problem,p", po::value<std::string>(&problem)->default_value("tiger.dpomdp"), "the problem to be solved")
//                             ("formalism,f", po::value<std::string>(&formalism)->default_value("DecPOMDP"), "the formalism to use")
//                             ("discount,d", po::value<double>(&discount)->default_value(1.0), "the discount factor")
//                             ("error,e", po::value<double>(&error)->default_value(0.1), "the error")
//                             ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
//                             ("memory,m", po::value<number>(&memory)->default_value(0), "the memory. If 0 then infinite memory.")
//                             ("trial", po::value<unsigned long>(&trials)->default_value(10000), "the maximum number of trials")
//                             ("time", po::value<unsigned long>(&time_max)->default_value(10000), "the maximum number of seconds")
//                             ("seed,s", po::value<int>(&seed)->default_value(1), "random seed")
                            
//                             ("save_path", po::value<std::string>(&save_path)->default_value("../tests/tests_results/"), "the path to the save the data")

//                             ("belief_precision", po::value<double>(&belief_precision)->default_value(0.00001), "the precision for the belief")
//                             ("ostate_precision", po::value<double>(&ostate_precision)->default_value(0.00001), "the precision for the occupancy states")
//                             ("compress_precision", po::value<double>(&compress_precision)->default_value(0.001), "the precision for compression")

//                             ("lower_bound", po::value<std::string>(&lower_bound)->default_value("tabular"), "the lower bound representation (Tabular,Maxplan,MaxplanLP,MaxPlanSerial)")
//                             ("upper_bound", po::value<std::string>(&upper_bound)->default_value("tabular"), "the upper bound representation (Tabular, Sawtooth,SawtoothLP,SawtoothLPSerial)")
//                             ("lb_init", po::value<std::string>(&lb_init)->default_value("Min"), "the lower bound initialization method (Min)")
//                             ("ub_init", po::value<std::string>(&ub_init)->default_value("Max"), "the upper bound initialization method (Max,MdpHsvi,PomdpHsvi)")
                            
//                             ("frequence_lower_bound_pruning", po::value<int>(&freq_lb_pruning)->default_value(-1), "")
//                             ("type_of_lower_bound_pruning", po::value<std::string>(&type_lb_pruning)->default_value("Bounded"), "")
//                             ("frequence_upper_bound_pruning", po::value<int>(&freq_ub_pruning)->default_value(-1), "")
//                             ("type_of_upper_bound_pruning", po::value<std::string>(&type_ub_pruning)->default_value("Global"), "")
                            
//                             ("sawtooth_type_of_resolution", po::value<std::string>(&sawtooth_type_of_resolution)->default_value("IloIfThen"), "")
//                             ("sawtooth_BigM_value", po::value<int>(&sawtooth_BigM)->default_value(100), "")
//                             ("sawtooth_type_of_linear_program", po::value<std::string>(&sawtooth_type_of_linear_program)->default_value("Full"), "")

//                             ("compression", po::value<bool>(&compression)->default_value(true),"do compression")
//                             ("store_actions", po::value<bool>(&store_actions)->default_value(true),"store_actions")
//                             ("store_states", po::value<bool>(&store_states)->default_value(true),"store_states");

//         po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a path with specified algorithms and configurations.");
//         visible.add(options).add(config);

//         po::options_description config_file_options;
//         config_file_options.add(config);

//         po::variables_map vm;
//         try
//         {
//             po::store(po::command_line_parser(argc, argv).options(visible).run(), vm);
//             po::notify(vm);
//             if (vm.count("help"))
//             {
//                 std::cout << visible << std::endl;
//                 return sdm::SUCCESS;
//             }
//         }
//         catch (po::error &e)
//         {
//             std::cerr << "ERROR: " << e.what() << std::endl;
//             std::cerr << visible << std::endl;
//             return sdm::ERROR_IN_COMMAND_LINE;
//         }

//         common::global_urng().seed(seed);

//         // Set precision
//         Belief::PRECISION = belief_precision;
//         OccupancyState::PRECISION = ostate_precision;
//         PrivateOccupancyState::PRECISION_COMPRESSION = compress_precision;

//         name =  "#"+problem+
//                 "#"+formalism+
//                 "#"+std::to_string(horizon)+
//                 "#"+std::to_string(discount)+
//                 "#"+std::to_string(memory)+
//                 "#"+std::to_string(trials)+
//                 "#"+std::to_string(time_max)+

//                 "#"+std::to_string(belief_precision)+
//                 "#"+std::to_string(ostate_precision)+
//                 "#"+std::to_string(compress_precision)+

//                 "#"+ub_init+
//                 "#"+lb_init+
//                 "#"+upper_bound+
//                 "#"+lower_bound+

//                 "#"+sawtooth_type_of_resolution+
//                 "#"+std::to_string(sawtooth_BigM)+
//                 "#"+sawtooth_type_of_linear_program+

//                 "#"+type_lb_pruning+
//                 "#"+std::to_string(freq_lb_pruning)+
//                 "#"+type_ub_pruning+
//                 "#"+std::to_string(freq_ub_pruning)+

//                 "#"+std::to_string(0);

//         TypeOfMaxPlanPrunning type_lb_pruning_;
//         if(type_lb_pruning == "Bounded")
//         {
//             type_lb_pruning_ = TypeOfMaxPlanPrunning::BOUNDED;
//         }else if(type_lb_pruning == "Pairwise")
//         {
//             type_lb_pruning_ = TypeOfMaxPlanPrunning::PAIRWISE;
//         }

//         TypeOfSawtoothPrunning type_ub_pruning_;
//         if(type_ub_pruning == "Global")
//         {
//             type_ub_pruning_ = TypeOfSawtoothPrunning::GLOBAL;
//         }else if(type_ub_pruning == "Iterative")
//         {
//             type_ub_pruning_ = TypeOfSawtoothPrunning::ITERATIVE;
//         }

//         auto algo = sdm::algo::make("hsvi",
//                                     path+problem,
//                                     formalism,
//                                     upper_bound,
//                                     lower_bound,
//                                     ub_init,
//                                     lb_init,
//                                     discount,
//                                     error,
//                                     horizon,
//                                     trials,
//                                     memory,
//                                     compression,
//                                     store_states,
//                                     store_actions,
//                                     save_path+name,
//                                     time_max,
//                                     1, 
//                                     1,
//                                     sawtooth_type_of_resolution,
//                                     sawtooth_BigM,
//                                     sawtooth_type_of_linear_program,
//                                     type_lb_pruning_,
//                                     freq_lb_pruning,
//                                     type_ub_pruning_,
//                                     freq_ub_pruning);

//         // Initialize and solve the problem
//         algo->initialize();
// #ifdef LOGTIME
//         OccupancyMDP::TIME_IN_NEXT_STATE = 0;
//         OccupancyMDP::TIME_IN_COMPRESS = 0;
//         OccupancyMDP::TIME_IN_GET_ACTION = 0;
//         OccupancyMDP::TIME_IN_STEP = 0;
//         OccupancyMDP::TIME_IN_UNDER_STEP = 0;
//         OccupancyMDP::TIME_IN_GET_REWARD = 0;
//         OccupancyMDP::TIME_IN_EXP_NEXT = 0;
//         OccupancyMDP::TIME_IN_APPLY_DR = 0;
//         OccupancyMDP::TIME_IN_NEXT_OSTATE = 0;
//         OccupancyMDP::PASSAGE_IN_NEXT_STATE = 0;
//         OccupancyMDP::MEAN_SIZE_STATE = 0;

//         OccupancyState::cleanTIME();
//         HSVI::cleanTIME();
// #endif
//         std::chrono::high_resolution_clock::time_point t_begin = std::chrono::high_resolution_clock::now();

//         algo->solve();
//         std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();

//         // std::cout<<"Borne POMDP"<<std::static_pointer_cast<RelaxedValueFunction>(std::static_pointer_cast<HSVI>(algo)->getUpperBound()->getInitFunction())->getRelaxation()->str()<<std::endl;
//         // std::cout<<"Borne "<<std::static_pointer_cast<HSVI>(algo)->getUpperBound()->str()<<std::endl;
//         double TOTAL_TIME = std::chrono::duration_cast<std::chrono::duration<double>>(t_end-t_begin).count();
        
//         // Save results in a CSV file
//         std::static_pointer_cast<HSVI>(algo)->saveResults(save_path+name + "_test.csv");
//         // -----------------------------------------------------------------------------

//         // std::cout<<"Upper bound "<<std::static_pointer_cast<HSVI>(algo)->getUpperBound()->str()<<std::endl;
//         // algo->test();
//         struct sysinfo memInfo;

//         std::cout<<"Total Memory in this computer !!!  "<<std::Performance::totalMemory(memInfo)<<std::endl;

//         // Log execution times
//         std::ofstream ofs;
//         ofs.open(save_path+name + "_profiling.md", std::ios::out | std::ios::app);

//         ofs << std::setprecision(4) << std::fixed;
//         ofs << "## " << name << " - precision_compress=" << compress_precision << std::endl;
//         ofs << "| NAME\t\t\t\t|\tTIME\t\t|\tPERCENT\t\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
//         ofs << "| TOTAL_TIME \t\t\t|\t" << TOTAL_TIME << " s\t|\t" << 100 * (TOTAL_TIME / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
// #ifdef LOGTIME
//         ofs << "| HSVI::TIME_INITIALIZATION \t|\t" << HSVI::TIME_INITIALIZATION << " s\t|\t" << 100 * (HSVI::TIME_INITIALIZATION / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| HSVI::TIME_IN_SELECT_ACTION \t|\t" << HSVI::TIME_IN_SELECT_ACTION << " s\t|\t" << 100 * (HSVI::TIME_IN_SELECT_ACTION / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| HSVI::TIME_IN_SELECT_STATE \t|\t" << HSVI::TIME_IN_SELECT_STATE << " s\t|\t" << 100 * (HSVI::TIME_IN_SELECT_STATE / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| HSVI::TIME_IN_UPDATE_LB \t|\t" << HSVI::TIME_IN_UPDATE_LB << " s\t|\t" << 100 * (HSVI::TIME_IN_UPDATE_LB / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| HSVI::TIME_IN_UPDATE_UB \t|\t" << HSVI::TIME_IN_UPDATE_UB << " s\t|\t" << 100 * (HSVI::TIME_IN_UPDATE_UB / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_GET_ACTION \t|\t" << OccupancyMDP::TIME_IN_GET_ACTION << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_GET_ACTION / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_NEXT_OSTATE \t|\t" << OccupancyMDP::TIME_IN_NEXT_OSTATE << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_NEXT_OSTATE / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_COMP_NEXT_STATE \t|\t" << OccupancyMDP::TIME_IN_NEXT_STATE << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_NEXT_STATE / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_COMP_NEXT_STATE_MEAN \t|\t" << OccupancyMDP::TIME_IN_NEXT_STATE/OccupancyMDP::PASSAGE_IN_NEXT_STATE << " s\t|\t" << 100 * ((OccupancyMDP::TIME_IN_NEXT_STATE/OccupancyMDP::PASSAGE_IN_NEXT_STATE) / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_COMPRESS \t|\t" << OccupancyMDP::TIME_IN_COMPRESS << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_COMPRESS / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_GET_REWARD \t|\t" << OccupancyMDP::TIME_IN_GET_REWARD << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_GET_REWARD / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccMDP::TIME_IN_APPLY_DR \t|\t" << OccupancyMDP::TIME_IN_APPLY_DR << " s\t|\t" << 100 * (OccupancyMDP::TIME_IN_APPLY_DR / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
//         ofs << "| OccState::TIME_IN_GET_PROBA \t|\t" << OccupancyState::TIME_IN_GET_PROBA << " s\t|\t" << 100 * (OccupancyState::TIME_IN_GET_PROBA / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_MEAN_IN_GET_PROBA \t|\t" << OccupancyState::TIME_IN_GET_PROBA/OccupancyState::PASSAGE_GET_PROBA << " s\t|\t" << 100 * ( (OccupancyState::TIME_IN_GET_PROBA/OccupancyState::PASSAGE_GET_PROBA) / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_SET_PROBA \t|\t" << OccupancyState::TIME_IN_SET_PROBA << " s\t|\t" << 100 * (OccupancyState::TIME_IN_SET_PROBA / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_ADD_PROBA \t|\t" << OccupancyState::TIME_IN_ADD_PROBA << " s\t|\t" << 100 * (OccupancyState::TIME_IN_ADD_PROBA / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_FINALIZE \t|\t" << OccupancyState::TIME_IN_FINALIZE << " s\t|\t" << 100 * (OccupancyState::TIME_IN_FINALIZE / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_MEAN_IN_FINALIZE \t|\t" << OccupancyState::TIME_IN_FINALIZE/OccupancyState::PASSAGE_FINALIZE << " s\t|\t" << 100 * ( (OccupancyState::TIME_IN_FINALIZE/OccupancyState::PASSAGE_FINALIZE) / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_EQUAL \t|\t" << OccupancyState::TIME_IN_EQUAL_OPERATOR << " s\t|\t" << 100 * (OccupancyState::TIME_IN_EQUAL_OPERATOR / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_HASH \t|\t" << OccupancyState::TIME_IN_HASH << " s\t|\t" << 100 * (OccupancyState::TIME_IN_HASH / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_MINUS_OPERATOR \t|\t" << OccupancyState::TIME_IN_MINUS_OPERATOR << " s\t|\t" << 100 * (OccupancyState::TIME_IN_MINUS_OPERATOR / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_COMPRESS \t|\t" << OccupancyState::TIME_IN_COMPRESS << " s\t|\t" << 100 * (OccupancyState::TIME_IN_COMPRESS / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_DOT_OPERATOR \t|\t" << OccupancyState::TIME_IN_DOT_OPERATOR << " s\t|\t" << 100 * (OccupancyState::TIME_IN_DOT_OPERATOR / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| OccState::TIME_IN_INFERIOR_OPERATOR \t|\t" << OccupancyState::TIME_IN_INFERIOR_OPERATOR << " s\t|\t" << 100 * (OccupancyState::TIME_IN_INFERIOR_OPERATOR / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
//         ofs << "| LB::TIME_IN_BACKUP \t|\t" << std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_update_backup << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_update_backup / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| LB::TIME_IN_SELECT_BEST_ACTION \t|\t" << std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_update_best_action << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_update_best_action / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| LB::TIME_IN_EXIST \t|\t" << std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_exist << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_exist / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| LB::TIME_IN_EVALUATE \t|\t" << std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_evaluate << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getLowerBound()->total_time_evaluate / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| LB::TIME_IN_UPDATE \t|\t" << std::static_pointer_cast<HSVI>(algo)->getLowerBound()->time_update_value << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getLowerBound()->time_update_value / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| LB::TIME_IN_PRUNING\t|\t" << std::static_pointer_cast<HSVI>(algo)->getLowerBound()->time_pruning << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getLowerBound()->time_pruning / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
//         ofs << "| UB::TIME_IN_BACKUP \t|\t" << std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_update_backup << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_update_backup / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| UB::TIME_IN_SELECT_BEST_ACTION \t|\t" << std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_update_best_action << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_update_best_action / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| UB::TIME_IN_EXIST \t|\t" << std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_exist << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_exist / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| UB::TIME_IN_EVALUATE \t|\t" << std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_evaluate << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getUpperBound()->total_time_evaluate / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| UB::TIME_IN_UPDATE \t|\t" << std::static_pointer_cast<HSVI>(algo)->getUpperBound()->time_update_value << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getUpperBound()->time_update_value / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| UB::TIME_IN_PRUNING\t|\t" << std::static_pointer_cast<HSVI>(algo)->getUpperBound()->time_pruning << " s\t|\t" << 100 * (std::static_pointer_cast<HSVI>(algo)->getUpperBound()->time_pruning / TOTAL_TIME) << " %\t|" << std::endl;
//         ofs << "| ------------------------------|-----------------------|-----------------------|" << std::endl;
// #endif
//         ofs.close();
//     }
//     catch (sdm::exception::Exception &e)
//     {
//         std::cout << "!!! Exception: " << e.what() << std::endl;
//     }

// } // END main
