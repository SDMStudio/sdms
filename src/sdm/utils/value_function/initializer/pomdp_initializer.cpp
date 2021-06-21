#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/belief_2_occupancy_vf.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
 

namespace sdm
{
    namespace algo
    {
        // std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution, number BigM, std::string type_sawtooth_linear_programming);
    }
}

namespace sdm
{
    POMDPInitializer::POMDPInitializer(std::shared_ptr<SolvableByHSVI> world,std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials), world_(world)
    {
        std::cout << "In POMDPInitializer" << std::endl;
    }

    void POMDPInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        // Get relaxed MDP problem and thgetUnderlyingProbleme underlying problem
        auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
        std::shared_ptr<SolvableByHSVI> hsvi_pomdp = std::make_shared<BeliefMDP>(pomdp);


        auto tabular_backup = std::make_shared<TabularBackup>(hsvi_pomdp);

        auto init_lb = std::make_shared<MinInitializer>(hsvi_pomdp);
        auto init_ub = std::make_shared<MDPInitializer>(hsvi_pomdp,"HSVI");

        auto lb = std::make_shared<TabularValueFunction>(pomdp->getHorizon(),init_lb,tabular_backup);
        auto ub = std::make_shared<TabularValueFunction>(pomdp->getHorizon(),init_ub,tabular_backup);

        auto algorithm = std::make_shared<HSVI>(hsvi_pomdp, lb, ub, pomdp->getHorizon(), this->error_);

        algorithm->do_initialize();
        algorithm->do_solve();

        auto ubound = algorithm->getUpperBound();

        vf->initialize(std::make_shared<Belief2OccupancyValueFunction>(this->world_,ubound));
    }
} // namespace sdm
