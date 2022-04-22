
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/algorithms/planning/value_iteration.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>

#include <sdm/utils/value_function/initializer/mdp_relaxation.hpp>
#include <sdm/world/solvable_by_mdp.hpp>

namespace sdm
{

    MDPInitializer::MDPInitializer(std::shared_ptr<SolvableByDP> world, Config config)
        : algo_config(config), world(world)
    {
    }

    MDPInitializer::MDPInitializer(std::shared_ptr<SolvableByDP> world, std::string algo_name, double error, int trials) : world(world)
    {
        this->algo_config = {
            {"algo_name", algo_name},
            {"error", error},
            {"trials", trials},
        };
    }

    void MDPInitializer::init(std::shared_ptr<ValueFunctionInterface> vf)
    {
        auto value_function = std::dynamic_pointer_cast<ValueFunction>(vf);

        // Get relaxed MDP problem and thgetUnderlyingProbleme underlying problem
        auto mdp = this->world->getUnderlyingProblem();
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

        std::string algo_name = this->algo_config.get("algo_name", std::string("ValueIteration"));
        double error = this->algo_config.get("error", 0.01);
        int trials = this->algo_config.get("trials", 10000);

        if (algo_name == "ValueIteration")
        {

            // Exhaustive action selection
            auto action_tabular = std::make_shared<ExhaustiveActionSelection>(hsvi_mdp);

            // Initializer
            auto vf_init = std::make_shared<MinInitializer>(hsvi_mdp);

            // Instanciate value function
            std::shared_ptr<sdm::ValueFunction> mdp_vf = std::make_shared<TabularValueFunction>(hsvi_mdp, vf_init, action_tabular);
            mdp_vf->setUpdateRule(std::make_shared<update::TabularUpdate>(mdp_vf));
            std::shared_ptr<sdm::ValueFunction> tmp_mdp_vf = std::make_shared<TabularValueFunction>(hsvi_mdp, vf_init, action_tabular);
            tmp_mdp_vf->setUpdateRule(std::make_shared<update::TabularUpdate>(tmp_mdp_vf));

            auto value_iteration = std::make_shared<ValueIteration>(hsvi_mdp, mdp_vf, error, trials, "MdpValueIteration_Init");
            value_iteration->setTmpValueFunction(tmp_mdp_vf);

            value_iteration->initialize();
            value_iteration->solve();

            value_function->setInitFunction(std::make_shared<MDPRelaxation>(value_iteration->getValueFunction()));
        }
        else
        {
            // Exhaustive action selection
            auto action_tabular = std::make_shared<ExhaustiveActionSelection>(hsvi_mdp);

            // Initializers for HSVI MDP
            auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
            auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);

            // Lower bound instanciation
            auto lb = std::make_shared<TabularValueFunction>(hsvi_mdp, init_lb, action_tabular);
            lb->setUpdateRule(std::make_shared<update::TabularUpdate>(lb));

            // Upper bound instanciation
            auto ub = std::make_shared<TabularValueFunction>(hsvi_mdp, init_ub, action_tabular);
            ub->setUpdateRule(std::make_shared<update::TabularUpdate>(ub));

            // HSVI instanciation
            auto algorithm = std::make_shared<HSVI>(hsvi_mdp, lb, ub, error, trials, "MdpHsvi_Init");

            // Solve MDP with HSVI instanciation
            algorithm->initialize();

            for (const auto &element : *mdp->getStateSpace(0))
            {
                auto state = element;
                hsvi_mdp->setInitialState(state);
                algorithm->solve();
            }

            auto ubound = algorithm->getUpperBound();
            // Set the function that will be used to get interactively upper bounds
            value_function->setInitFunction(std::make_shared<MDPRelaxation>(ubound));
        }
    }
} // namespace sdm
