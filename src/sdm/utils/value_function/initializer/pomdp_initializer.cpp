#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/belief_2_occupancy_vf.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/utils/value_function/vfunction/point_set_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/maxplan_update.hpp>

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
        auto pomdp = this->world_->getUnderlyingProblem();
        std::shared_ptr<SolvableByHSVI> hsvi_pomdp = std::static_pointer_cast<OccupancyMDP>(this->world_)->getUnderlyingBeliefMDP();


        auto exhaustive_selection = std::make_shared<ExhaustiveActionSelection>(hsvi_pomdp);

        auto init_lb = std::make_shared<MinInitializer>(hsvi_pomdp);
        auto init_ub = std::make_shared<MDPInitializer>(hsvi_pomdp, "ValueIteration", 0);

        auto lb = std::make_shared<TabularValueFunction>(hsvi_pomdp, init_lb, exhaustive_selection, nullptr, false);
        lb->setUpdateOperator(std::make_shared<update::TabularUpdate>(lb));

        auto ub = std::make_shared<TabularValueFunction>(hsvi_pomdp, init_ub, exhaustive_selection, nullptr, true);
        ub->setUpdateOperator(std::make_shared<update::TabularUpdate>(ub));

        auto algorithm = std::make_shared<HSVI>(hsvi_pomdp, lb, ub, this->error_, 5000, "pomdp_" + this->algo_name_ + "_init", 1, 1, 1000);

        algorithm->initialize();
        algorithm->solve();

        auto ubound = algorithm->getUpperBound();

        value_function->setInitFunction(std::make_shared<Belief2OccupancyValueFunction>(ubound));
    }
} // namespace sdm
