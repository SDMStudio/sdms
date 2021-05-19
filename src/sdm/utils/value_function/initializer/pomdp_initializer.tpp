#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/world_type.hpp>
#include <sdm/utils/value_function/belief_2_occupancy_vf.hpp>

namespace sdm
{
    namespace algo
    {
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution , number BigM , std::string type_sawtooth_linear_programming );
    }
}

namespace sdm
{
    template <typename TState, typename TAction>
    POMDPInitializer<TState, TAction>::POMDPInitializer(std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials)
    {
        std::cout << "In POMDPInitializer" << std::endl;
    }

    template <typename TState, typename TAction>
    void POMDPInitializer<TState, TAction>::init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
    {
        auto pomdp = std::static_pointer_cast<typename WorldType<TState, TAction>::type>(vf->getWorld())->toBeliefMDP();
        auto underlying_pb = pomdp->getUnderlyingProblem();
        auto algorithm = algo::makeHSVI<decltype(pomdp->getInitialState()), number>(pomdp, "tabular", "tabular", "MdpHsviInitializer", "MinInitializer", underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon(), this->trials_, "pomdp_init","BigM",100,"Full");

        algorithm->do_initialize();
        algorithm->do_solve();
        auto ubound = algorithm->getUpperBound();

        vf->initialize(std::make_shared<Belief2OccupancyValueFunction<decltype(pomdp->getInitialState()), TState>>(ubound));
    }
} // namespace sdm
