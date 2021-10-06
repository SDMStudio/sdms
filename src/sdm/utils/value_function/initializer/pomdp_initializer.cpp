#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/belief_2_occupancy_vf.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>

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

    void POMDPInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        // Get relaxed MDP problem and thgetUnderlyingProbleme underlying problem
        auto pomdp = this->world_->getUnderlyingProblem();
        std::shared_ptr<SolvableByHSVI> hsvi_pomdp = std::static_pointer_cast<OccupancyMDP>(this->world_)->getUnderlyingBeliefMDP();

        auto tabular_backup = std::make_shared<TabularBackup>(hsvi_pomdp);
        auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi_pomdp);

        auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_pomdp);
        auto action_maxplan = std::make_shared<ActionVFMaxplan>(hsvi_pomdp);

        auto init_lb = std::make_shared<MinInitializer>(hsvi_pomdp);
        auto init_ub = std::make_shared<MDPInitializer>(hsvi_pomdp,"ValueIteration",0);

        auto lb = std::make_shared<TabularValueFunction>(pomdp->getHorizon(), init_lb, tabular_backup, action_tabular, false);
        auto ub = std::make_shared<TabularValueFunction>(pomdp->getHorizon(), init_ub, tabular_backup, action_tabular, true);

        auto algorithm = std::make_shared<HSVI>(hsvi_pomdp, lb, ub, this->error_, 5000, "pomdp_"+ this->algo_name_+ "_init", 1, 1, 1000);

        algorithm->initialize();
        algorithm->solve();
        
        auto ubound = algorithm->getUpperBound();

        vf->initialize(std::make_shared<Belief2OccupancyValueFunction>(ubound));
    }
} // namespace sdm
