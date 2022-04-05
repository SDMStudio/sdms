#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_relaxation.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/utils/value_function/vfunction/sawtooth_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/pwlc_update.hpp>

#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

namespace sdm
{
    POMDPInitializer::POMDPInitializer(std::shared_ptr<SolvableByDP> world, Config config) : algo_config(config), world(world)
    {
        // this->algorithm = sdm::algo::registry::make(config.get<std::string>("algo_name", "HSVI"), config);
    }

    POMDPInitializer::POMDPInitializer(std::shared_ptr<SolvableByDP> world, std::string algo_name, double error, int trials) : world(world)
    {
        this->algo_config = {
            {"algo_name", algo_name},
            {"error", error},
            {"trials", trials},
        };
    }

    void POMDPInitializer::init(std::shared_ptr<ValueFunctionInterface> vf)
    {
        auto value_function = std::dynamic_pointer_cast<ValueFunction>(vf);
        // Get relaxed belief MDP
        std::shared_ptr<SolvableByHSVI> hsvi_pomdp = std::dynamic_pointer_cast<BeliefMDPInterface>(this->world)->getUnderlyingBeliefMDP();

        auto exhaustive_selection = std::make_shared<ExhaustiveActionSelection>(hsvi_pomdp);

        auto init_lb = std::make_shared<MinInitializer>(hsvi_pomdp);
        auto init_ub = std::make_shared<MDPInitializer>(hsvi_pomdp, "ValueIteration");

        std::string algo_name = this->algo_config.get("algo_name", std::string("HSVI"));
        double error = this->algo_config.get("error", 0.001), time_max = this->algo_config.get("time_max", 3600);
        int trials = this->algo_config.get("trials", 10000);

        std::shared_ptr<ValueFunction> lb, ub;
        if (algo_name == "TabHSVI")
        {
            // Instanciate lower bound
            lb = std::make_shared<TabularValueFunction>(hsvi_pomdp, init_lb, exhaustive_selection);
            lb->setUpdateRule(std::make_shared<update::TabularUpdate>(lb));
            // Instanciate upper bound
            ub = std::make_shared<TabularValueFunction>(hsvi_pomdp, init_ub, exhaustive_selection);
            ub->setUpdateRule(std::make_shared<update::TabularUpdate>(ub));
        }
        else
        {
            // Instanciate lower bound
            lb = std::make_shared<PWLCValueFunction>(hsvi_pomdp, init_lb, exhaustive_selection, 1, MaxplanPruning::PAIRWISE);
            lb->setUpdateRule(std::make_shared<update::PWLCUpdate>(lb));

            // Instanciate upper bound
            ub = std::make_shared<SawtoothValueFunction>(hsvi_pomdp, init_ub, exhaustive_selection, 1, SawtoothPruning::PAIRWISE);
            ub->setUpdateRule(std::make_shared<update::TabularUpdate>(ub));
        }

        auto algorithm = std::make_shared<HSVI>(hsvi_pomdp, lb, ub, error, trials, "PomdpHsvi_Init", 1, 1, time_max);

        algorithm->initialize();
        algorithm->solve();

        auto ubound = algorithm->getUpperBound();

        value_function->setInitFunction(std::make_shared<POMDPRelaxation>(ubound));
    }
} // namespace sdm
