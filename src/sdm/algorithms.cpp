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
        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem,
                                            std::string upper_bound_name,
                                            std::string lower_bound_name,
                                            std::string ub_init_name,
                                            std::string lb_init_name,
                                            double discount,
                                            double error,
                                            number horizon,
                                            int trials,
                                            bool store_state,
                                            bool store_action,
                                            std::string name,
                                            double time_max,
                                            number freq_update_lb,
                                            number freq_update_ub,
                                            std::string current_type_of_resolution,
                                            number BigM,
                                            std::string type_sawtooth_linear_programming,
                                            TypeOfMaxPlanPrunning type_of_maxplan_prunning,
                                            int freq_prunning_lower_bound,
                                            TypeOfSawtoothPrunning type_of_sawtooth_pruning,
                                            int freq_prunning_upper_bound)
        {
            assert(((discount < 1) || (horizon > 0)));

            // Type of resolution to use (for CPLEX)
            TypeOfResolution type_of_resolution = (current_type_of_resolution == "BigM") ? TypeOfResolution::BigM
                                                                                         : TypeOfResolution::IloIfThenResolution;

            // Type of sawtooth linear program
            TypeSawtoothLinearProgram type_of_sawtooth_linear_program;
            if (type_sawtooth_linear_programming == "Full")
            {
                type_of_sawtooth_linear_program = TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING;
            }

            // Instanciate possible action selection
            auto exhaustive_selection = std::make_shared<ExhaustiveActionSelection>(problem);
            auto action_maxplan_wcsp = std::make_shared<ActionSelectionMaxplanWCSP>(problem);
#ifdef WITH_CPLEX
            auto action_maxplan_serial = std::make_shared<ActionSelectionMaxplanSerial>(problem);
            auto action_maxplan_lp = std::make_shared<ActionSelectionMaxplanLP>(problem);
            auto action_maxplan_lp_serial = std::make_shared<ActionSelectionMaxplanLPSerial>(problem);
            auto action_sawtooth_lp = std::make_shared<ActionSelectionSawtoothLP>(problem, type_of_resolution, BigM, type_of_sawtooth_linear_program);
            auto action_sawtooth_lp_serial = std::make_shared<ActionSelectionSawtoothLPSerial>(problem, type_of_resolution, BigM, type_of_sawtooth_linear_program);
#endif

            // Instanciate initializers
            auto lb_init = sdm::makeInitializer(lb_init_name, problem);
            auto ub_init = sdm::makeInitializer(ub_init_name, problem);

            // Instanciate update operators
            std::shared_ptr<UpdateOperatorInterface> lb_update_operator, ub_update_operator;

            // Instanciate bounds
            std::shared_ptr<sdm::ValueFunction> lower_bound;
            std::shared_ptr<sdm::ValueFunction> upper_bound;

            // Lower Bound
            if (lower_bound_name == "maxplan")
            {
                lower_bound = std::make_shared<PWLCValueFunction>(problem, lb_init, exhaustive_selection, nullptr, freq_prunning_lower_bound, type_of_maxplan_prunning);
                lb_update_operator = std::make_shared<update::MaxPlanUpdateOperator>(lower_bound);
            }
            else if (lower_bound_name == "maxplan_wcsp")
            {
                lower_bound = std::make_shared<PWLCValueFunction>(problem, lb_init, action_maxplan_wcsp, nullptr, freq_prunning_lower_bound, type_of_maxplan_prunning);
                lb_update_operator = std::make_shared<update::MaxPlanUpdateOperator>(lower_bound);
            }
#ifdef WITH_CPLEX
            else if (lower_bound_name == "maxplan_serial")
            {
                lower_bound = std::make_shared<PWLCValueFunction>(problem, lb_init, action_maxplan_serial, nullptr, freq_prunning_lower_bound, type_of_maxplan_prunning);
                lb_update_operator = std::make_shared<update::MaxPlanUpdateOperator>(lower_bound);
            }
            else if (lower_bound_name == "maxplan_lp")
            {
                lower_bound = std::make_shared<PWLCValueFunction>(problem, lb_init, action_maxplan_lp, nullptr, freq_prunning_lower_bound, type_of_maxplan_prunning);
                lb_update_operator = std::make_shared<update::MaxPlanUpdateOperator>(lower_bound);
            }
            else if (lower_bound_name == "maxplan_lp_serial")
            {
                lower_bound = std::make_shared<PWLCValueFunction>(problem, lb_init, action_maxplan_lp_serial, nullptr, freq_prunning_lower_bound, type_of_maxplan_prunning);
                lb_update_operator = std::make_shared<update::MaxPlanUpdateOperator>(lower_bound);
            }
#endif
            else if (lower_bound_name == "tabular")
            {
                if (store_state)
                    lower_bound = std::make_shared<TabularValueFunction>(problem, lb_init, exhaustive_selection);
                else
                    lower_bound = std::make_shared<TabularValueFunction2>(problem, lb_init, exhaustive_selection);
                lb_update_operator = std::make_shared<update::TabularUpdate>(lower_bound);
            }
            else
            {
                std::cout << "Unrecognized Lower bound name" << std::endl;
            }
            lower_bound->setUpdateOperator(lb_update_operator);

            // Upper Bound
            if (upper_bound_name == "sawtooth")
            {
                if (store_state)
                    upper_bound = std::make_shared<PointSetValueFunction>(problem, ub_init, exhaustive_selection, nullptr, freq_prunning_upper_bound, type_of_sawtooth_pruning);
                else
                    upper_bound = std::make_shared<PointSetValueFunction2>(problem, ub_init, exhaustive_selection, nullptr, freq_prunning_upper_bound, type_of_sawtooth_pruning);
                ub_update_operator = std::make_shared<update::TabularUpdate>(upper_bound);
            }
#ifdef WITH_CPLEX
            else if (upper_bound_name == "sawtooth_lp")
            {
                if (store_state)
                    upper_bound = std::make_shared<PointSetValueFunction>(problem, ub_init, action_sawtooth_lp, nullptr, freq_prunning_upper_bound, type_of_sawtooth_pruning);
                else
                    upper_bound = std::make_shared<PointSetValueFunction2>(problem, ub_init, action_sawtooth_lp, nullptr, freq_prunning_upper_bound, type_of_sawtooth_pruning);
                ub_update_operator = std::make_shared<update::TabularUpdate>(upper_bound);
            }
            else if (upper_bound_name == "sawtooth_lp_serial")
            {
                if (store_state)
                    upper_bound = std::make_shared<PointSetValueFunction>(problem, ub_init, action_sawtooth_lp_serial, nullptr, freq_prunning_upper_bound, type_of_sawtooth_pruning);
                else
                    upper_bound = std::make_shared<PointSetValueFunction2>(problem, ub_init, action_sawtooth_lp_serial, nullptr, freq_prunning_upper_bound, type_of_sawtooth_pruning);
                ub_update_operator = std::make_shared<update::TabularUpdate>(upper_bound);
            }
#endif
            else
            {
                if (store_state)
                    upper_bound = std::make_shared<TabularValueFunction>(problem, ub_init, exhaustive_selection);
                else
                    upper_bound = std::make_shared<TabularValueFunction2>(problem, ub_init, exhaustive_selection);
                ub_update_operator = std::make_shared<update::TabularUpdate>(upper_bound);
            }
            upper_bound->setUpdateOperator(ub_update_operator);

            return std::make_shared<HSVI>(problem, lower_bound, upper_bound, error, trials, name, freq_update_lb, freq_update_ub, time_max);
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

        std::shared_ptr<sdm::QLearning> makeQLearning(std::shared_ptr<SolvableByDP> problem,
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

            // Instanciate action selection
            std::shared_ptr<ActionSelectionInterface> exhaustive_selection = std::make_shared<ExhaustiveActionSelection>(problem);

            // Instanciate qvalue function
            std::shared_ptr<TabularQValueFunction> qvalue;
            qvalue = std::make_shared<TabularQValueFunction>(problem, initializer, exhaustive_selection);

            // Instanciate target qvalue function
            std::shared_ptr<TabularQValueFunction> target_qvalue;
            target_qvalue = std::make_shared<TabularQValueFunction>(problem, initializer, exhaustive_selection);

            // Set update operator
            qvalue->setUpdateOperator(std::make_shared<TabularQUpdate>(experience_memory, qvalue, target_qvalue, discount, lr));

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

        std::shared_ptr<Algorithm> makeAlgorithm(std::string algo_name,
                                                 std::shared_ptr<SolvableByHSVI> formalism_problem,
                                                 std::string formalism_name,
                                                 std::string upper_bound,
                                                 std::string lower_bound,
                                                 std::string ub_init,
                                                 std::string lb_init,
                                                 double discount,
                                                 double error,
                                                 int trials,
                                                 bool store_state,
                                                 bool store_action,
                                                 std::string name,
                                                 double time_max,
                                                 number freq_update_lb,
                                                 number freq_update_ub,
                                                 std::string current_type_of_resolution,
                                                 number BigM,
                                                 std::string type_sawtooth_linear_programming,
                                                 TypeOfMaxPlanPrunning type_of_maxplan_prunning,
                                                 int freq_prunning_lower_bound,
                                                 TypeOfSawtoothPrunning type_of_sawtooth_pruning,
                                                 int freq_prunning_upper_bound)
        {
            //  Build the algorithm
            std::shared_ptr<Algorithm> p_algo;
            if ((algo_name == "hsvi") || (algo_name == "HSVI"))
            {
                p_algo = makeHSVI(formalism_problem, upper_bound, lower_bound, ub_init, lb_init,
                                  discount, error, formalism_problem->getUnderlyingProblem()->getHorizon(), trials,
                                  store_state, store_action, (name == "") ? "tab_ext_ohsvi" : name, time_max,
                                  freq_update_lb, freq_update_ub, current_type_of_resolution, BigM, type_sawtooth_linear_programming, type_of_maxplan_prunning, freq_prunning_lower_bound, type_of_sawtooth_pruning, freq_prunning_upper_bound);
            }
            else if ((algo_name == "qlearning") || (algo_name == "QLearning") || (algo_name == "QLEARNING"))
            {
                // std::shared_ptr<GymInterface> gym = std::dynamic_pointer_cast<GymInterface>(formalism_problem);
                p_algo = makeQLearning(formalism_problem, formalism_problem->getUnderlyingProblem()->getHorizon(), discount, error, 1, trials, name);
            }
            else if ((algo_name == "Alpha*") || (algo_name == "A*"))
            {
                if (!((formalism_name == "Extensive-OccupancyMDP") || (formalism_name == "extensive-occupancy-mdp") ||
                      (formalism_name == "ext-oMDP") || (formalism_name == "ext-omdp") || (formalism_name == "omdp") ||
                      (formalism_name == "OccupancyMDP") || (formalism_name == "oMDP") || (formalism_name == "occupancy-mdp")))
                {
                    throw sdm::exception::Exception("Formalism impossible for A* algorithm, the problem have to be a derive from occupancy MDP");
                }
                p_algo = std::make_shared<AlphaStar>(formalism_problem);
            }
            else if ((algo_name == "BackwardInduction"))
            {
                p_algo = std::make_shared<BackwardInduction>(formalism_problem);
            }
            else if ((algo_name == "ValueIteration") || (algo_name == "VI"))
            {
                p_algo = makeValueIteration(formalism_problem, lower_bound,
                                            lb_init, discount, error,
                                            formalism_problem->getUnderlyingProblem()->getHorizon(),
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

        std::shared_ptr<Algorithm> make(std::string algo_name,
                                        std::string problem_path,
                                        std::string formalism,
                                        std::string upper_bound,
                                        std::string lower_bound,
                                        std::string ub_init,
                                        std::string lb_init,
                                        double discount,
                                        double error,
                                        number horizon,
                                        int trials,
                                        int memory,
                                        bool compression,
                                        bool store_state,
                                        bool store_action,
                                        std::string name,
                                        double time_max,
                                        number freq_update_lb,
                                        number freq_update_ub,
                                        std::string current_type_of_resolution,
                                        number BigM,
                                        std::string type_sawtooth_linear_programming,
                                        TypeOfMaxPlanPrunning type_of_maxplan_prunning,
                                        int freq_prunning_lower_bound,
                                        TypeOfSawtoothPrunning type_of_sawtooth_pruning,
                                        int freq_prunning_upper_bound,
                                        number batch_size)
        {

            // Build the formalism
            auto formalism_problem = algo::makeFormalism(problem_path, formalism, discount, horizon, memory, compression, store_state, store_action, batch_size);

            // Build the algorithm
            auto algo = algo::makeAlgorithm(algo_name, formalism_problem, formalism,
                                            upper_bound, lower_bound, ub_init, lb_init,
                                            discount, error, trials, store_state, store_action, name, time_max,
                                            freq_update_lb, freq_update_ub,
                                            current_type_of_resolution, BigM, type_sawtooth_linear_programming,
                                            type_of_maxplan_prunning, freq_prunning_lower_bound, type_of_sawtooth_pruning, freq_prunning_upper_bound);
            return algo;
        }

        std::vector<std::string> available()
        {
            return {"A*", "BackwardInduction", "HSVI", "QLearning", "ValueIteration"};
        }

    }
}