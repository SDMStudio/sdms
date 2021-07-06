#include <sdm/algorithms.hpp>


namespace sdm
{
    namespace algo
    {
        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound_name, std::string lower_bound_name, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution, number BigM, std::string type_sawtooth_linear_programming)
        {
            assert(((discount < 1) || (horizon > 0)));

            // Increase the horizon for the value function if the problem is serialized
            if (problem->isSerialized())
            {
                horizon = horizon * problem->getUnderlyingProblem()->getNumAgents();
            }

            auto tabular_backup = std::make_shared<TabularBackup>(hsvi);
            auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi);

            auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi);
            auto action_maxplan = std::make_shared<ActionVFMaxplan>(hsvi);
            auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(hsvi);

            // Instanciate initializers
            // auto lb_init = sdm::makeInitializer(lb_init_name);
            // auto ub_init = sdm::makeInitializer(ub_init_name);

            // // Instanciate bounds
            // std::shared_ptr<sdm::ValueFunction> lower_bound;
            // if (lower_bound_name == "maxplan")
            // {
            //     // lower_bound = std::make_shared<sdm::MaxPlanValueFunction>(problem, horizon, lb_init);
            // }
            // else if (lower_bound_name == "maxplan_serial")
            // {
            //     // lower_bound = std::make_shared<sdm::MaxPlanValueFunctionSerialized>(problem, horizon, lb_init);
            // }
            // else if (lower_bound_name == "maxplan_lp")
            // {
            //     // lower_bound = std::make_shared<sdm::MaxPlanValueFunctionLP>(problem, horizon, lb_init);
            // }
            // else
            // {
            //     // lower_bound = std::make_shared<sdm::MappedValueFunction>(problem, horizon, lb_init);
            // }

            // std::shared_ptr<sdm::ValueFunction> upper_bound;
            // if (upper_bound_name == "sawtooth")
            // {
            //     // upper_bound = std::make_shared<sdm::SawtoothValueFunction>(problem, horizon, ub_init);
            // }
            // else if (upper_bound_name == "sawtooth_lp")
            // {
            //     if (type_sawtooth_linear_programming == "Full")
            //     {
            //         if (current_type_of_resolution == "BigM")
            //         {
            //             // upper_bound = std::make_shared<sdm::SawtoothValueFunctionLP>(problem, horizon, ub_init, TypeOfResolution::BigM, BigM, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);
            //         }
            //         else
            //         {
            //             // upper_bound = std::make_shared<sdm::SawtoothValueFunctionLP>(problem, horizon, ub_init, TypeOfResolution::IloIfThenResolution, BigM, TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING);
            //         }
            //     }
            //     else
            //     {
            //         // upper_bound = std::make_shared<sdm::SawtoothValueFunctionLP>(problem, horizon, ub_init, TypeOfResolution::BigM, BigM, TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING);
            //     }
            // }
            // else
            // {
            //     // upper_bound = std::make_shared<sdm::MappedValueFunction>(problem, horizon, ub_init);
            // }

            // return std::make_shared<HSVI>(problem, lower_bound, upper_bound, horizon, error, trials, name);
        }
    }
}