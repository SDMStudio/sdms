#include <sdm/algorithms.hpp>
#include <sdm/algorithms/alpha_star.hpp>
#include <sdm/algorithms/backward_induction.hpp>

#include <sdm/utils/value_function/initializer/initializers.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp_serial.hpp>

#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp_serial.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_wcsp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

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
                                            std::string name,
                                            double time_max,
                                            std::string current_type_of_resolution,
                                            number BigM,
                                            std::string type_sawtooth_linear_programming,
                                            TypeOfMaxPlanPrunning type_of_maxplan_prunning,
                                            int freq_prunning_lower_bound,
                                            TypeOfSawtoothPrunning type_of_sawtooth_pruning,
                                            int freq_prunning_upper_bound)
        {
            assert(((discount < 1) || (horizon > 0)));

            auto tabular_backup = std::make_shared<TabularBackup>(problem);
            auto maxplan_backup = std::make_shared<MaxPlanBackup>(problem);

            auto action_tabular = std::make_shared<ActionVFTabulaire>(problem);
            auto action_maxplan = std::make_shared<ActionVFMaxplan>(problem);
            auto action_maxplan_serial = std::make_shared<ActionVFMaxplanSerial>(problem);
            auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(problem);
            auto action_maxplan_lp_serial = std::make_shared<ActionVFMaxplanLPSerial>(problem);
            auto action_maxplan_wcsp = std::make_shared<ActionVFMaxplanWCSP>(problem);

            TypeOfResolution type_of_resolution;

            if (current_type_of_resolution == "BigM")
            {
                type_of_resolution = TypeOfResolution::BigM;
            }
            else
            {
                type_of_resolution = TypeOfResolution::IloIfThenResolution ;
            }

            TypeSawtoothLinearProgram type_of_sawtooth_linear_program;
            if (type_sawtooth_linear_programming == "Full")
            {
                type_of_sawtooth_linear_program = TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING;
            }

            auto action_sawtooth_lp = std::make_shared<ActionVFSawtoothLP>(problem, type_of_resolution, BigM, type_of_sawtooth_linear_program);
            auto action_sawtooth_lp_serial = std::make_shared<ActionVFSawtoothLPSerial>(problem, type_of_resolution, BigM, type_of_sawtooth_linear_program);


            // Instanciate initializers
            auto lb_init = sdm::makeInitializer(lb_init_name, problem);
            auto ub_init = sdm::makeInitializer(ub_init_name, problem);

            // Instanciate bounds
            std::shared_ptr<sdm::ValueFunction> lower_bound;
            std::shared_ptr<sdm::ValueFunction> upper_bound;

            // Lower Bound
            if (lower_bound_name == "maxplan")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon, lb_init, maxplan_backup, action_maxplan, freq_prunning_lower_bound, type_of_maxplan_prunning);
            }
            else if (lower_bound_name == "maxplan_serial")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon, lb_init, maxplan_backup, action_maxplan_serial, freq_prunning_lower_bound, type_of_maxplan_prunning);
            }
            else if (lower_bound_name == "maxplan_lp")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon, lb_init, maxplan_backup, action_maxplan_lp, freq_prunning_lower_bound, type_of_maxplan_prunning);
            }
            else if (lower_bound_name == "maxplan_lp_serial")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon, lb_init, maxplan_backup, action_maxplan_lp_serial, freq_prunning_lower_bound, type_of_maxplan_prunning);
            }
            else if (lower_bound_name == "maxplan_wcsp")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon, lb_init, maxplan_backup, action_maxplan_wcsp,freq_prunning_lower_bound,type_of_maxplan_prunning);
            }
            else
            {
                lower_bound = std::make_shared<TabularValueFunction>(horizon, lb_init, tabular_backup, action_tabular, false);
            }

            // Upper Bound
            if (upper_bound_name == "sawtooth")
            {
                upper_bound = std::make_shared<PointSetValueFunction>(horizon, ub_init, tabular_backup, action_tabular, freq_prunning_upper_bound, type_of_sawtooth_pruning);
            }
            else if (upper_bound_name == "sawtooth_lp")
            {
                upper_bound = std::make_shared<PointSetValueFunction>(horizon, ub_init, tabular_backup, action_sawtooth_lp, freq_prunning_upper_bound, type_of_sawtooth_pruning);
            }
            else if (upper_bound_name == "sawtooth_lp_serial")
            {
                upper_bound = std::make_shared<PointSetValueFunction>(horizon, ub_init, tabular_backup, action_sawtooth_lp_serial, freq_prunning_upper_bound, type_of_sawtooth_pruning);
            }
            else
            {
                upper_bound = std::make_shared<TabularValueFunction>(horizon, ub_init, tabular_backup, action_tabular, true);
            }

            return std::make_shared<HSVI>(problem, lower_bound, upper_bound, horizon, error, trials, name, 1, 1, time_max);
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
                                        int truncation, 
                                        std::string name, 
                                        double time_max, 
                                        std::string current_type_of_resolution, 
                                        number BigM, 
                                        std::string type_sawtooth_linear_programming, 
                                        TypeOfMaxPlanPrunning type_of_maxplan_prunning, 
                                        int freq_prunning_lower_bound, 
                                        TypeOfSawtoothPrunning type_of_sawtooth_pruning, 
                                        int freq_prunning_upper_bound, 
                                        bool compression, 
                                        bool store_action,
                                        bool store_state)
        {
            std::shared_ptr<Algorithm> p_algo;

            auto problem = sdm::parser::parse_file(problem_path);

            problem->setHorizon(horizon);
            problem->setDiscount(discount);

            std::shared_ptr<SolvableByHSVI> formalism_problem;

            if ((formalism == "mdp") || (formalism == "MDP"))
            {
                formalism_problem = std::make_shared<SolvableByMDP>(problem);
            }
            else if ((formalism == "pomdp") || (formalism == "POMDP"))
            {
                formalism_problem = std::make_shared<BeliefMDP>(problem);
            }
            else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
            {
                formalism_problem = std::make_shared<OccupancyMDP>(problem, truncation, compression, store_state, store_action);
            }
            else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP"))
            {
                auto serialized_mmdp = std::make_shared<SerializedMMDP>(problem);
                formalism_problem = std::make_shared<SolvableByMDP>(serialized_mmdp);
            }
            else if ((formalism == "extensive-pomdp") || (formalism == "Extensive-POMDP"))
            {
                auto serialized_mpomdp = std::make_shared<SerializedMPOMDP>(problem);
                formalism_problem = std::make_shared<BeliefMDP>(serialized_mpomdp);
            }
            else if ((formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") || (formalism == "Extensive-DPOMDP"))
            {
                auto serialized_mpomdp = std::make_shared<SerializedMPOMDP>(problem);
                formalism_problem = std::make_shared<SerialOccupancyMDP>(serialized_mpomdp, truncation, compression, store_state, store_action);
            }

            if ((algo_name == "hsvi") || (algo_name == "HSVI"))
            {
                p_algo = makeHSVI(formalism_problem, upper_bound, lower_bound, ub_init, lb_init, discount, error, formalism_problem->getUnderlyingProblem()->getHorizon(), trials, (name == "") ? "tab_ext_ohsvi" : name, time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming, type_of_maxplan_prunning, freq_prunning_lower_bound, type_of_sawtooth_pruning, freq_prunning_upper_bound);
            }
            else if ((algo_name == "qlearning") || (algo_name == "QLEARNING"))
            {
                throw sdm::exception::Exception("QLearning not added in \"algorithms.hpp\" ");
            }
            else if ((algo_name == "Alpha*") || (algo_name == "A*"))
            {
                if ( ! ( (formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") ||
                     (formalism == "Extensive-DPOMDP") || (formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP")) )
                {
                    throw sdm::exception::Exception("Formalism impossible for A* algorithm, the problem have to be a decpomdp or extensive-decpomdp");
                }
                p_algo = std::make_shared<AlphaStar>(formalism_problem);
            }
            else if ((algo_name == "Backward Induction*"))
            {
                p_algo = std::make_shared<BackwardInduction>(formalism_problem);
            }
            else
            {
                throw sdm::exception::Exception("Undefined algorithm type : " + algo_name);
            }

            return p_algo;
        }
    }

    std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem,
                                                        double error,
                                                        number horizon)
    {
        // Increase the horizon for the value function if the problem is serialized
        if (problem->isSerialized())
        {
            horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
        }
        return std::make_shared<ValueIteration>(problem, error, horizon);
    }
}