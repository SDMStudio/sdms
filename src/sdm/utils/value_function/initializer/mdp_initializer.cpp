
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/initializer/state_2_occupancy_vf.hpp>
#include <sdm/world/solvable_by_mdp.hpp>

namespace sdm
{
    namespace algo
    {
        // std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution , number BigM , std::string type_sawtooth_linear_programming );

        // std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem, double discount, double error, number horizon);
    }
}
namespace sdm
{
    MDPInitializer::MDPInitializer(std::shared_ptr<SolvableByHSVI> world, std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials), world_(world)
    {
        std::cout << "In MDPInitializer" << std::endl;
    }

    void MDPInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        // Get relaxed MDP problem and thgetUnderlyingProbleme underlying problem
        auto mdp = this->world_->getUnderlyingProblem();
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

        if (this->algo_name_ == "ValueIteration")
        {
            // std::cout<<"Value Iteration";
            // auto value_iteration = algo::makeValueIteration(hsvi_mdp, mdp->getDiscount(), this->error_, mdp->getPlanningHorizon());

            // value_iteration->do_initialize();
            // value_iteration->do_solve();

            // vf->initialize(std::make_shared<State2OccupancyValueFunction<decltype(hsvi_mdp->getInitialState()), TState>>(value_iteration->getResult()));
        }
        else
        {
            std::cout << "MDP Iteration";

            auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
            auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

            auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
            auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);

            auto lb = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular);
            auto ub = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular);

            auto algorithm = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), this->error_, 100000, "MDP_Initialisation");

            algorithm->do_initialize();

            for (const auto &element : *mdp->getStateSpace(0))
            {
                auto state = element->toState();
                hsvi_mdp->setInitialState(state);
                algorithm->do_solve();
            }

            auto ubound = algorithm->getUpperBound();
            vf->initialize(std::make_shared<State2OccupancyValueFunction>(ubound));
        }
        // Set the function that will be used to get interactively upper bounds
    }
} // namespace sdm
