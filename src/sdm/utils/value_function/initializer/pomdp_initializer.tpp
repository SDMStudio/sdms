#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/world_type.hpp>
#include <sdm/utils/value_function/belief_2_occupancy_vf.hpp>

namespace sdm
{
    namespace algo
    {
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeMappedHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, std::string ub_init_name, std::string lb_init_name, double discount, double error, int horizon, int trials, std::string name);
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
    void POMDPInitializer<TState, TAction>::init(ValueFunction<TState, TAction> *vf)
    {
        auto pomdp = std::static_pointer_cast<typename WorldType<TState, TAction>::type>(vf->getWorld())->toBeliefMDP();
        auto underlying_pb = pomdp->getUnderlyingProblem();
        auto algorithm = algo::makeMappedHSVI<decltype(pomdp->getInitialState()), number>(pomdp, "MaxInitializer", "MinInitializer", underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon(), this->trials_, "pomdp_init");
        algorithm->do_solve();
        auto ubound = algorithm->getUpperBound();


        std::cout<<"\n mmmmmmmmmm"<<ubound->str();        
        // Je remarque que cette fonction est appelé 2 fois, mais je ne comprends pas pourquoi ? A vérifier la raison
        
        vf->initialize(std::make_shared<Belief2OccupancyValueFunction<decltype(pomdp->getInitialState()), TState>>(ubound));
    }
} // namespace sdm
