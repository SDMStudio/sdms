#include <sdm/algorithms.hpp>
#include <sdm/algorithms/alpha_star.hpp>
#include <sdm/algorithms/q_learning.hpp>
#include <sdm/algorithms/planning/vi.hpp>
#include <sdm/algorithms/planning/pbvi.hpp>
#include <sdm/algorithms/backward_induction.hpp>

#include <sdm/utils/value_function/initializer/initializers.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/qfunction/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/vfunction/point_set_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>

#include <sdm/utils/value_function/update_operator.hpp>

#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp_serial.hpp>

#include <sdm/utils/value_function/action_selection/lp/action_maxplan_lp.hpp>
#include <sdm/utils/value_function/action_selection/lp/action_maxplan_lp_serial.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_serial.hpp>
#include <sdm/utils/value_function/action_selection/wcsp/action_maxplan_wcsp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/worlds.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>
#include <sdm/world/hierarchical_occupancy_mdp.hpp>

#include <sdm/utils/rl/eps_greedy.hpp>
#include <sdm/utils/rl/experience_memory.hpp>

namespace sdm
{
    namespace algo
    {

        std::shared_ptr<ActionSelectionInterface> makeActionSawtoothLP(std::shared_ptr<SolvableByDP> problem,
                                                                       std::string value_name,
                                                                       std::string type_of_resolution_name)
        {
            std::shared_ptr<ActionSelectionInterface> action_selection;
#ifdef WITH_CPLEX
            // Get the type of resolution to use (for CPLEX)
            TypeOfResolution type_of_resolution;
            number BigM;

            if (type_of_resolution_name.find("BigM") != string::npos)
            {
                type_of_resolution = TypeOfResolution::BigM;
                BigM = std::stoi(type_of_resolution_name.substr(std::string("BigM").size() + 1));
            }
            else
            {
                type_of_resolution = TypeOfResolution::IloIfThenResolution;
            }

            // Get the type of sawtooth linear program
            TypeSawtoothLinearProgram type_of_sawtooth_linear_program;
            if (value_name.find("relaxed") != string::npos)
            {
                type_of_sawtooth_linear_program = TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING;
            }
            else
            {
                type_of_sawtooth_linear_program = TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING;
            }

            if (isInstanceOf<SerialProblemInterface>(problem))
            {
                action_selection = std::make_shared<ActionSelectionSawtoothLPSerial>(problem, type_of_resolution, BigM, type_of_sawtooth_linear_program);
            }
            else
            {
                action_selection = std::make_shared<ActionSelectionSawtoothLP>(problem, type_of_resolution, BigM, type_of_sawtooth_linear_program);
            }
#endif
            if (!action_selection)
            {
                throw sdm::exception::Exception("LP is disable. Please install CPLEX and recompile with adequate arguments.");
            }
            return action_selection;
        }

        std::shared_ptr<ValueFunction> makeValueFunction(std::shared_ptr<SolvableByDP> problem,
                                                         std::string value_name,
                                                         std::string init_name,
                                                         bool store_state,
                                                         std::string type_of_resolution_name,
                                                         std::string type_of_pruning_name,
                                                         int freq_pruning)
        {
            std::shared_ptr<sdm::ValueFunction> value_function;
            std::shared_ptr<sdm::Initializer> initializer;
            std::shared_ptr<ActionSelectionInterface> action_selection;
            std::shared_ptr<UpdateOperatorInterface> update_operator;

            // Instanciate initializer
            initializer = sdm::makeInitializer(init_name, std::dynamic_pointer_cast<SolvableByHSVI>(problem));

            // Build value function based on parameters
            if (value_name.find("maxplan") != string::npos)
            {
                // Action selection;
                if (value_name.find("wcsp") != string::npos)
                    action_selection = std::make_shared<ActionSelectionMaxplanWCSP>(problem);
                else if (value_name.find("lp") != string::npos)
                {
                    if (isInstanceOf<SerialProblemInterface>(problem))
                        action_selection = std::make_shared<ActionSelectionMaxplanLPSerial>(problem);
                    else
                        action_selection = std::make_shared<ActionSelectionMaxplanLP>(problem);
                }
                else
                {
                    if (isInstanceOf<SerialProblemInterface>(problem))
                        action_selection = std::make_shared<ActionSelectionMaxplanSerial>(problem);
                    else
                        action_selection = std::make_shared<ExhaustiveActionSelection>(problem);
                }

                // Pruning type
                MaxplanPruning::Type type_of_pruning;
                if (type_of_pruning_name == "bounded")
                    type_of_pruning = MaxplanPruning::BOUNDED;
                else if (type_of_pruning_name == "pairwise")
                    type_of_pruning = MaxplanPruning::PAIRWISE;
                else if (type_of_pruning_name == "none")
                    type_of_pruning = MaxplanPruning::NONE;
                else
                    throw sdm::exception::Exception("Unrecognized maxplan pruning name.");

                // PWLC value function
                value_function = std::make_shared<PWLCValueFunction>(problem, initializer, action_selection, nullptr, freq_pruning, type_of_pruning);

                // Update operator
                update_operator = std::make_shared<update::PWLCUpdate>(value_function);
            }
            else if (value_name.find("sawtooth") != string::npos)
            {
                // Action selection;
                if (value_name.find("lp") != string::npos)
                    action_selection = makeActionSawtoothLP(problem, value_name, type_of_resolution_name);
                else
                    action_selection = std::make_shared<ExhaustiveActionSelection>(problem);

                // Pruning type
                SawtoothPrunning::Type type_of_pruning;
                if (type_of_pruning_name == "iterative")
                    type_of_pruning = SawtoothPrunning::ITERATIVE;
                else if (type_of_pruning_name == "global")
                    type_of_pruning = SawtoothPrunning::GLOBAL;
                else if (type_of_pruning_name == "both")
                    type_of_pruning = SawtoothPrunning::BOTH;
                else if (type_of_pruning_name == "none")
                    type_of_pruning = SawtoothPrunning::NONE;
                else
                    throw sdm::exception::Exception("Unrecognized sawtooth pruning name.");

                // Point set value function
                if (store_state)
                    value_function = std::make_shared<PointSetValueFunction>(problem, initializer, action_selection, nullptr, freq_pruning, type_of_pruning);
                else
                    value_function = std::make_shared<PointSetValueFunction2>(problem, initializer, action_selection, nullptr, freq_pruning, type_of_pruning);

                // Update operator
                update_operator = std::make_shared<update::TabularUpdate>(value_function);
            }
            else if (value_name.find("tabular") != string::npos)
            {
                // Action selection;
                action_selection = std::make_shared<ExhaustiveActionSelection>(problem);
                // Tabular value function
                if (store_state)
                    value_function = std::make_shared<TabularValueFunction>(problem, initializer, action_selection);
                else
                    value_function = std::make_shared<TabularValueFunction2>(problem, initializer, action_selection);
                // Update operator
                update_operator = std::make_shared<update::TabularUpdate>(value_function);
            }
            else
            {
                std::cout << "Unrecognized value function name" << std::endl;
            }
            value_function->setUpdateOperator(update_operator);

            return value_function;
        }

        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem,
                                            double discount,
                                            double error,
                                            number horizon,
                                            int trials,
                                            bool store_state,
                                            bool store_action,
                                            std::string name,
                                            double time_max,
                                            std::string lower_bound_name,
                                            std::string upper_bound_name,
                                            std::string lb_init_name,
                                            std::string ub_init_name,
                                            number lb_freq_update,
                                            number ub_freq_update,
                                            std::string lb_type_of_resolution_name,
                                            std::string ub_type_of_resolution_name,
                                            int lb_freq_pruning,
                                            int ub_freq_pruning,
                                            std::string lb_type_of_pruning,
                                            std::string ub_type_of_pruning)
        {
            assert(((discount < 1) || (horizon > 0)));

            // Instanciate bounds
            std::shared_ptr<sdm::ValueFunction> lower_bound = makeValueFunction(problem, lower_bound_name, lb_init_name, store_state, lb_type_of_resolution_name, lb_type_of_pruning, lb_freq_pruning);
            std::shared_ptr<sdm::ValueFunction> upper_bound = makeValueFunction(problem, upper_bound_name, ub_init_name, store_state, ub_type_of_resolution_name, ub_type_of_pruning, ub_freq_pruning);

            return std::make_shared<HSVI>(problem, lower_bound, upper_bound, error, trials, name, lb_freq_update, ub_freq_update, time_max);
        }

        std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem, std::string value_function_name,
                                                                std::string vf_init_name, double discount, double error, number horizon,
                                                                bool store_state, std::string name, double time_max)
        {
            // Instanciate the initializer
            auto init = sdm::makeInitializer(vf_init_name, problem);

            // Instanciate the action selection procedure
            auto exhaustive_selection = std::make_shared<ExhaustiveActionSelection>(problem);

            std::shared_ptr<ValueFunction> value_function;
            if (store_state)
                value_function = std::make_shared<TabularValueFunction>(problem, init, exhaustive_selection);
            else
                value_function = std::make_shared<TabularValueFunction2>(problem, init, exhaustive_selection);

            // Instanciate the update operator
            value_function->setUpdateOperator(std::make_shared<TabularUpdate>(value_function));

            return std::make_shared<PBVI>(problem, value_function, error, time_max, name);
        }

        std::shared_ptr<QValueFunction> makeQValueFunction(std::shared_ptr<SolvableByDP> problem, std::string qvalue_name, std::string q_init_name)
        {
            std::shared_ptr<sdm::QValueFunction> qvalue;
            std::shared_ptr<sdm::Initializer> q_init;
            std::shared_ptr<ActionSelectionInterface> action_selection;
            std::shared_ptr<QUpdateOperatorInterface> q_update_operator;

            // Instanciate initializer
            q_init = sdm::makeInitializer(q_init_name, std::dynamic_pointer_cast<SolvableByHSVI>(problem));

            if (qvalue_name.find("maxplan") != string::npos)
            {
                if (qvalue_name.find("wcsp") != string::npos)
                {
                    action_selection = std::make_shared<ActionSelectionMaxplanWCSP>(problem);
                }
                else if (qvalue_name.find("lp") != string::npos)
                {
                    action_selection = std::make_shared<ActionSelectionMaxplanLP>(problem);
                }
                else
                {
                    action_selection = std::make_shared<ExhaustiveActionSelection>(problem);
                }
                qvalue = std::make_shared<PWLCQValueFunction>(problem, q_init, action_selection);
            }
            else if (qvalue_name.find("tabular") != string::npos)
            {
                action_selection = std::make_shared<ExhaustiveActionSelection>(problem);
                qvalue = std::make_shared<TabularQValueFunction>(problem, q_init, action_selection);
            }
            else
            {
                std::cout << "Unrecognized Lower bound name" << std::endl;
            }
            return qvalue;
        }

        std::shared_ptr<sdm::QLearning> makeQLearning(std::shared_ptr<SolvableByDP> problem,
                                                      std::string qvalue_name,
                                                      std::string q_init_name,
                                                      number horizon,
                                                      double discount,
                                                      double lr,
                                                      double batch_size,
                                                      unsigned long num_episodes,
                                                      std::string name)
        {
            assert(((discount < 1) || (horizon > 0)));

            // Instanciate exploration process
            std::shared_ptr<EpsGreedy> exploration = std::make_shared<EpsGreedy>();

            // Instanciate the memory
            std::shared_ptr<ExperienceMemory> experience_memory = std::make_shared<ExperienceMemory>(horizon);

            // Instanciate initializer
            std::shared_ptr<ZeroInitializer> initializer = std::make_shared<sdm::ZeroInitializer>();

            // Instanciate qvalue function
            std::shared_ptr<QValueFunction> qvalue = makeQValueFunction(problem, qvalue_name, q_init_name);

            // Instanciate target qvalue function
            std::shared_ptr<QValueFunction> target_qvalue = makeQValueFunction(problem, qvalue_name, q_init_name);

            // Instanciate udpate operator
            std::shared_ptr<QUpdateOperatorInterface> update_operator;
            if (qvalue_name.find("maxplan") != string::npos)
            {
                update_operator = std::make_shared<update::PWLCQUpdate>(experience_memory, qvalue, target_qvalue, lr);
            }
            else
            {
                update_operator = std::make_shared<update::TabularQUpdate>(experience_memory, qvalue, target_qvalue, lr);
            }

            // Set update operator
            qvalue->setUpdateOperator(update_operator);

            // Instanciate algorithme
            std::shared_ptr<QLearning> algorithm = std::make_shared<QLearning>(std::dynamic_pointer_cast<GymInterface>(problem), experience_memory, qvalue, qvalue, exploration, horizon, batch_size, num_episodes, name);

            return algorithm;
        }

        std::shared_ptr<SolvableByHSVI> makeFormalism(std::string problem_path,
                                                      std::string formalism,
                                                      double discount,
                                                      number horizon,
                                                      int memory,
                                                      bool compression,
                                                      bool store_state,
                                                      bool store_action,
                                                      number batch_size)
        {
            // Parse the problem
            auto problem = sdm::parser::parse_file(problem_path);

            problem->setHorizon(horizon);
            problem->setDiscount(discount);

            // Build the transformed problem
            std::shared_ptr<SolvableByHSVI> formalism_problem;

            if ((formalism == "mdp") || (formalism == "MDP"))
            {
                formalism_problem = std::make_shared<SolvableByMDP>(problem);
            }
            else if ((formalism == "belief-mdp") || (formalism == "BeliefMDP") || (formalism == "bmdp") || (formalism == "bMDP"))
            {
                formalism_problem = std::make_shared<BeliefMDP>(problem, batch_size);
            }
            else if ((formalism == "occupancy-mdp") || (formalism == "OccupancyMDP") || (formalism == "omdp") || (formalism == "oMDP"))
            {
                formalism_problem = std::make_shared<OccupancyMDP>(problem, memory, compression, store_state, store_action, batch_size);
            }
            else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP") || (formalism == "ext-MDP") || (formalism == "ext-mdp"))
            {
                auto serial_mmdp = std::make_shared<SerialMMDP>(problem);
                formalism_problem = std::make_shared<SolvableByMDP>(serial_mmdp);
            }
            else if ((formalism == "extensive-belief-mdp") || (formalism == "Extensive-BeliefMDP") || (formalism == "ext-bMDP") || (formalism == "ext-bmdp"))
            {
                auto serial_mpomdp = std::make_shared<SerialMPOMDP>(problem);
                formalism_problem = std::make_shared<BeliefMDP>(serial_mpomdp, batch_size);
            }
            else if ((formalism == "extensive-occupancy-mdp") || (formalism == "Extensive-OccupancyMDP") || (formalism == "ext-oMDP") || (formalism == "ext-omdp"))
            {
                auto serial_mpomdp = std::make_shared<SerialMPOMDP>(problem);
                formalism_problem = std::make_shared<SerialOccupancyMDP>(serial_mpomdp, memory, compression, store_state, store_action, batch_size);
            }
            else if ((formalism == "hierarchical-mdp") || (formalism == "Hierarchical-MDP") || (formalism == "hMDP") || (formalism == "hmdp"))
            {
                auto hierarchical_mpomdp = std::make_shared<HierarchicalMPOMDP>(problem);
                formalism_problem = std::make_shared<SolvableByMDP>(hierarchical_mpomdp);
            }
            else if ((formalism == "hierarchical-belief-mdp") || (formalism == "Hierarchical-BeliefMDP") || (formalism == "hbMDP") || (formalism == "hbmdp"))
            {
                auto hierarchical_mpomdp = std::make_shared<HierarchicalMPOMDP>(problem);
                formalism_problem = std::make_shared<BeliefMDP>(hierarchical_mpomdp, batch_size);
            }
            else if ((formalism == "hierarchical-occupancy-mdp") || (formalism == "Hierarchical-OccupancyMDP") || (formalism == "hoMDP") || (formalism == "homdp"))
            {
                auto hierarchical_mpomdp = std::make_shared<HierarchicalMPOMDP>(problem);
                formalism_problem = std::make_shared<HierarchicalOccupancyMDP>(hierarchical_mpomdp, memory, compression, store_state, store_action, batch_size);
            }
            else
            {
                std::ostringstream res;
                res << "Undefined formalism type : " << formalism << std::endl
                    << std::endl;
                res << "#> Available formalisms are : " << std::endl;
                res << "FORMALISMS\t" << std::endl
                    << "---" << std::endl;
                for (auto algo : sdm::world::available())
                {
                    res << algo << std::endl;
                }
                throw sdm::exception::Exception(res.str());
            }
            return formalism_problem;
        }

        std::shared_ptr<Algorithm> makeAlgorithm(std::string algo_name, std::shared_ptr<SolvableByHSVI> formalism, double discount,
                                                 double error, int trials, bool store_state, bool store_action, std::string name, double time_max,
                                                 std::string value_function_1, std::string init_v1, number freq_update_v1, std::string type_of_resolution_v1, int freq_pruning_v1, std::string type_of_pruning_v1,
                                                 std::string value_function_2, std::string init_v2, number freq_update_v2, std::string type_of_resolution_v2, int freq_pruning_v2, std::string type_of_pruning_v2)
        {
            //  Build the algorithm
            std::shared_ptr<Algorithm> p_algo;
            if ((algo_name == "hsvi") || (algo_name == "HSVI"))
            {
                p_algo = makeHSVI(formalism, discount, error, formalism->getHorizon(), trials, store_state, store_action, name, time_max,
                                  value_function_1, value_function_2, init_v1, init_v2, freq_update_v1, freq_update_v2,
                                  type_of_resolution_v1, type_of_resolution_v2, freq_pruning_v1, freq_pruning_v2, type_of_pruning_v1, type_of_pruning_v2);
            }
            else if ((algo_name == "qlearning") || (algo_name == "QLearning") || (algo_name == "QLEARNING"))
            {
                // std::shared_ptr<GymInterface> gym = std::dynamic_pointer_cast<GymInterface>(formalism);
                p_algo = makeQLearning(formalism, value_function_1, init_v1, formalism->getHorizon(), discount, error, 1, trials, name);
            }
            else if ((algo_name == "Alpha*") || (algo_name == "A*"))
            {
                if (isInstanceOf<BeliefMDPInterface>(formalism))
                {
                    std::shared_ptr<sdm::ValueFunction> value_function = makeValueFunction(formalism, value_function_1, init_v1, store_state, type_of_resolution_v1, type_of_pruning_v1, freq_pruning_v1);
                    p_algo = std::make_shared<AlphaStar>(formalism, value_function, name);
                }
                else
                {
                    throw sdm::exception::Exception("Formalism impossible for A* algorithm, the problem have to inherit from belief MDP");
                }
            }
            else if ((algo_name == "BackwardInduction"))
            {
                p_algo = std::make_shared<BackwardInduction>(formalism);
            }
            else if ((algo_name == "ValueIteration") || (algo_name == "VI"))
            {
                p_algo = makeValueIteration(formalism, value_function_1,
                                            init_v1, discount, error,
                                            formalism->getHorizon(),
                                            store_state, name, time_max);
            }
            else
            {
                std::ostringstream res;
                res << "Undefined algorithm type : " << algo_name << std::endl
                    << std::endl;
                res << "#> Available algorithms are : " << std::endl;
                res << "ALGORITHM\t" << std::endl
                    << "---" << std::endl;
                for (auto algo : sdm::algo::available())
                {
                    res << algo << std::endl;
                }
                throw sdm::exception::Exception(res.str());
            }

            return p_algo;
        }

        std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, std::string formalism_name, number horizon, double discount, double error, int trials, double time_max, std::string name,
                                        int memory, bool compression, bool store_state, bool store_action, number batch_size,
                                        std::string value_function_1, std::string init_v1, number freq_update_v1, std::string type_of_resolution_v1, int freq_pruning_v1, std::string type_of_pruning_v1,
                                        std::string value_function_2, std::string init_v2, number freq_update_v2, std::string type_of_resolution_v2, int freq_pruning_v2, std::string type_of_pruning_v2)
        {
            std::string COLOR_BLOCKS = config::SDMS_THEME_1,
                        COLOR_NAME_PARAMS = config::SDMS_THEME_2,
                        COLOR_PARAMS = config::NO_COLOR;

            std::cout << COLOR_BLOCKS << "------------------------------------" << std::endl;
            std::cout << COLOR_BLOCKS << "[Algorithm]" << std::endl;
            std::cout << COLOR_NAME_PARAMS << "algo_name=" << COLOR_PARAMS << algo_name << std::endl;
            std::cout << COLOR_NAME_PARAMS << "problem_path=" << COLOR_PARAMS << problem_path << std::endl;
            std::cout << COLOR_NAME_PARAMS << "formalism_name=" << COLOR_PARAMS << formalism_name << std::endl;
            std::cout << std::endl;

            std::cout << COLOR_BLOCKS << "[Algorithm.Config]" << std::endl;
            std::cout << COLOR_NAME_PARAMS << "horizon=" << COLOR_PARAMS << horizon << std::endl;
            std::cout << COLOR_NAME_PARAMS << "discount=" << COLOR_PARAMS << discount << std::endl;
            std::cout << COLOR_NAME_PARAMS << "error=" << COLOR_PARAMS << error << std::endl;
            std::cout << COLOR_NAME_PARAMS << "trials=" << COLOR_PARAMS << trials << std::endl;
            std::cout << COLOR_NAME_PARAMS << "time_max=" << COLOR_PARAMS << time_max << std::endl;
            std::cout << COLOR_NAME_PARAMS << "name=" << COLOR_PARAMS << name << std::endl;
            std::cout << std::endl;

            std::cout << COLOR_BLOCKS << "[Algorithm.ValueFunction1]" << std::endl;
            std::cout << COLOR_NAME_PARAMS << "value_function_1=" << COLOR_PARAMS << value_function_1 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "init_v1=" << COLOR_PARAMS << init_v1 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "freq_update_v1=" << COLOR_PARAMS << freq_update_v1 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "type_of_resolution_v1=" << COLOR_PARAMS << type_of_resolution_v1 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "freq_pruning_v1=" << COLOR_PARAMS << freq_pruning_v1 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "type_of_pruning_v1=" << COLOR_PARAMS << type_of_pruning_v1 << std::endl;
            std::cout << std::endl;

            std::cout << COLOR_BLOCKS << "[Algorithm.ValueFunction2]" << std::endl;
            std::cout << COLOR_NAME_PARAMS << "value_function_2=" << COLOR_PARAMS << value_function_2 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "init_v2=" << COLOR_PARAMS << init_v2 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "freq_update_v2=" << COLOR_PARAMS << freq_update_v2 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "type_of_resolution_v2=" << COLOR_PARAMS << type_of_resolution_v2 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "freq_pruning_v2=" << COLOR_PARAMS << freq_pruning_v2 << std::endl;
            std::cout << COLOR_NAME_PARAMS << "type_of_pruning_v2=" << COLOR_PARAMS << type_of_pruning_v2 << std::endl;
            std::cout << COLOR_BLOCKS << "------------------------------------" << std::endl;
            std::cout << config::NO_COLOR;

            // Build the formalism
            auto formalism = makeFormalism(problem_path, formalism_name, discount, horizon, memory, compression, store_state, store_action, batch_size);

            // Build the algorithm
            return makeAlgorithm(algo_name, formalism, discount, error, trials, store_state, store_action, name, time_max,
                                 value_function_1, init_v1, freq_update_v1, type_of_resolution_v1, freq_pruning_v1, type_of_pruning_v1,
                                 value_function_2, init_v2, freq_update_v2, type_of_resolution_v2, freq_pruning_v2, type_of_pruning_v2);
        }

        std::vector<std::string> available()
        {
            return {"A*", "BackwardInduction", "HSVI", "QLearning", "ValueIteration"};
        }

    }
}