#include <sdm/algorithms.hpp>

#include <sdm/utils/value_function/initializer/initializers.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

namespace sdm
{
    namespace algo
    {
        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound_name, std::string lower_bound_name, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, double time_max, std::string current_type_of_resolution, number BigM, std::string type_sawtooth_linear_programming )
        {
            assert(((discount < 1) || (horizon > 0)));

            // Increase the horizon for the value function if the problem is serialized
            if (problem->isSerialized())
            {
                horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
            }

            auto tabular_backup = std::make_shared<TabularBackup>(problem);
            auto maxplan_backup = std::make_shared<MaxPlanBackup>(problem);

            auto action_tabular = std::make_shared<ActionVFTabulaire>(problem);
            auto action_maxplan = std::make_shared<ActionVFMaxplan>(problem);
            auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(problem);

            // Instanciate initializers
            auto lb_init = sdm::makeInitializer(lb_init_name,problem);
            auto ub_init = sdm::makeInitializer(ub_init_name,problem);

            // Instanciate bounds
            std::shared_ptr<sdm::ValueFunction> lower_bound;
            std::shared_ptr<sdm::ValueFunction> upper_bound;

            // Lower Bound
            if (lower_bound_name == "maxplan")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon,lb_init,maxplan_backup,action_maxplan);
            }
            else if (lower_bound_name == "maxplan_serial")
            {
                // lower_bound = std::make_shared<sdm::MaxPlanValueFunctionSerialized>(problem, horizon, lb_init);
            }
            else if (lower_bound_name == "maxplan_lp")
            {
                lower_bound = std::make_shared<HyperplanValueFunction>(horizon,lb_init,maxplan_backup,action_maxplan_lp);
            }
            else
            {                
                lower_bound = std::make_shared<TabularValueFunction>(horizon,lb_init,tabular_backup,action_tabular);
            }

            // Upper Bound
            if (upper_bound_name == "sawtooth")
            {
                upper_bound = std::make_shared<PointSetValueFunction>(horizon,ub_init,tabular_backup,action_tabular);
            }
            else if (upper_bound_name == "sawtooth_lp")
            {
                if (type_sawtooth_linear_programming == "Full")
                {
                    if (current_type_of_resolution == "BigM")
                    {
                        upper_bound = std::make_shared<PointSetValueFunction>(horizon,ub_init,tabular_backup, std::make_shared<ActionVFSawtoothLP>(problem, TypeOfResolution::BigM,BigM,TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING));
                    }
                    else
                    {
                        upper_bound = std::make_shared<PointSetValueFunction>(horizon,ub_init,tabular_backup, std::make_shared<ActionVFSawtoothLP>(problem,TypeOfResolution::IloIfThenResolution,0,TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING));
                    }
                }
                else
                {
                    upper_bound = std::make_shared<PointSetValueFunction>(horizon,ub_init,tabular_backup,std::make_shared<ActionVFSawtoothLP>(problem, TypeOfResolution::BigM,BigM,TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING));
                }
            }
            else
            {
                upper_bound = std::make_shared<TabularValueFunction>(horizon,ub_init,tabular_backup,action_tabular);
            }

            return std::make_shared<HSVI>(problem, lower_bound, upper_bound, horizon, error, trials, name, time_max);
        }


        std::shared_ptr<Algorithm> make(std::string algo_name, std::string problem_path, std::string formalism, std::string upper_bound, std::string lower_bound, std::string ub_init, std::string lb_init, double discount, double error, number horizon, int trials, int truncation, std::string name, double time_max, std::string current_type_of_resolution, number BigM, std::string type_sawtooth_linear_programming)
        {
            std::shared_ptr<Algorithm> p_algo;

            auto file = sdm::parser::parse_file(problem_path);

            file->setHorizon(horizon);
            file->setDiscount(discount);

            if ((algo_name == "hsvi"))
            {
                if ((formalism == "mdp") || (formalism == "MDP"))
                {
                    auto mdp  = std::make_shared<SolvableByMDP>(file);
                    p_algo = makeHSVI(mdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_mdphsvi" : name,time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
                }
                else if ((formalism == "pomdp") || (formalism == "POMDP"))
                {
                    auto beliefMDP = std::make_shared<BeliefMDP>(file);
                    p_algo = makeHSVI(beliefMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name,time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
                }
                else if ((formalism == "decpomdp") || (formalism == "DecPOMDP") || (formalism == "dpomdp") || (formalism == "DPOMDP"))
                {
                    auto oMDP = std::make_shared<OccupancyMDP>(file, (truncation > 0) ? truncation : horizon,true);
                    p_algo = makeHSVI(oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ohsvi" : name,time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
                }
                else if ((formalism == "extensive-mdp") || (formalism == "Extensive-MDP"))
                {
                    auto serialized_mdp = std::make_shared<SerializedMMDP>(file);
                    auto s_mdp  = std::make_shared<SolvableByMDP>(serialized_mdp);
                    p_algo = makeHSVI(s_mdp, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ext_mdphsvi" : name,time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
                }
                else if ((formalism == "extensive-pomdp") || (formalism == "Extensive-POMDP"))
                {
                    auto serialized_pomdp = std::make_shared<SerializedMPOMDP>(file);
                    auto s_beliefMDP = std::make_shared<BeliefMDP>(serialized_pomdp);
                    p_algo = makeHSVI(s_beliefMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_hsvi" : name,time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
                }
                else if ((formalism == "extensive-decpomdp") || (formalism == "Extensive-DecPOMDP") || (formalism == "extensive-dpomdp") || (formalism == "Extensive-DPOMDP"))
                {
                    auto serialized_pomdp = std::make_shared<SerializedMPOMDP>(file);
                    auto s_oMDP = std::make_shared<OccupancyMDP>(serialized_pomdp, (truncation > 0) ? truncation : horizon,true);
                    p_algo = makeHSVI(s_oMDP, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, (name == "") ? "tab_ext_ohsvi" : name,time_max, current_type_of_resolution, BigM, type_sawtooth_linear_programming);
                }
            }
            else
            {
                throw sdm::exception::Exception("Undefined algorithm type : " + algo_name);
            }

            return p_algo;
        }
    }
}