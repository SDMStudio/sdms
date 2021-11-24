#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_relaxation.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/utils/value_function/vfunction/sawtooth_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/pwlc_update.hpp>

#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

namespace sdm
{
    namespace algo
    {
        // std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution, number BigM, std::string type_sawtooth_linear_programming);
    }
}

namespace sdm
{
    POMDPInitializer::POMDPInitializer(std::shared_ptr<SolvableByHSVI> world, std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials), world_(world)
    {
    }

    void POMDPInitializer::init(std::shared_ptr<ValueFunctionInterface> vf)
    {
        auto value_function = std::dynamic_pointer_cast<ValueFunction>(vf);
        // Get relaxed MDP problem and thgetUnderlyingProbleme underlying problem
        std::shared_ptr<SolvableByHSVI> hsvi_pomdp = std::dynamic_pointer_cast<BeliefMDPInterface>(this->world_)->getUnderlyingBeliefMDP();

        auto exhaustive_selection = std::make_shared<ExhaustiveActionSelection>(hsvi_pomdp);

        auto init_lb = std::make_shared<MinInitializer>(hsvi_pomdp);
        auto init_ub = std::make_shared<MDPInitializer>(hsvi_pomdp, "ValueIteration");

        std::shared_ptr<ValueFunction> lb, ub;
        if (this->algo_name_ == "TabHSVI")
        {
            // Instanciate lower bound
            lb = std::make_shared<TabularValueFunction>(hsvi_pomdp, init_lb, exhaustive_selection);
            lb->setUpdateOperator(std::make_shared<update::TabularUpdate>(lb));
            // Instanciate upper bound
            ub = std::make_shared<TabularValueFunction>(hsvi_pomdp, init_ub, exhaustive_selection);
            ub->setUpdateOperator(std::make_shared<update::TabularUpdate>(ub));
        }
        else
        {
            // Instanciate lower bound
            lb = std::make_shared<PWLCValueFunction>(hsvi_pomdp, init_lb, exhaustive_selection, nullptr, 1);
            lb->setUpdateOperator(std::make_shared<update::PWLCUpdate>(lb));

            // Instanciate upper bound
            ub = std::make_shared<SawtoothValueFunction>(hsvi_pomdp, init_ub, exhaustive_selection, nullptr, 1, SawtoothPruning::PAIRWISE);
            ub->setUpdateOperator(std::make_shared<update::TabularUpdate>(ub));
        }

        auto algorithm = std::make_shared<HSVI>(hsvi_pomdp, lb, ub, this->error_, this->trials_, "PomdpHsvi_Init", 1, 1, 3600);

        algorithm->initialize();
        algorithm->solve();

        auto ubound = algorithm->getUpperBound();

        value_function->setInitFunction(std::make_shared<POMDPRelaxation>(ubound));
    }
} // namespace sdm
