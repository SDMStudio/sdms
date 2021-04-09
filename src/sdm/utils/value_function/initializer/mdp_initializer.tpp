#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/world_type.hpp>

namespace sdm
{
    namespace algo
    {
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name);
    }
}
namespace sdm
{
    template <typename TState, typename TAction>
    MDPInitializer<TState, TAction>::MDPInitializer(std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials)
    {
        std::cout << "In MDPInitializer" << std::endl;
    }

    template <typename TState, typename TAction>
    void MDPInitializer<TState, TAction>::init(ValueFunction<TState, TAction> *vf)
    {
        // Get relaxed MDP problem and the underlying problem
        auto mdp = std::static_pointer_cast<typename WorldType<TState, TAction>::type>(vf->getWorld())->toMDP();
        auto underlying_pb = mdp->getUnderlyingProblem();

        // Instanciate HSVI for MDP
        auto algorithm = algo::makeHSVI<decltype(mdp->getInitialState()), number>(mdp, "tabular", "tabular", "MaxInitializer", "MinInitializer", underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon(), this->trials_, "mdp_init");
        algorithm->do_initialize();

        // Solve HSVI from every possible initial state
        for (auto &s : underlying_pb->getStateSpace()->getAll())
        {
            double proba_s = underlying_pb->getStartDistrib().probabilities()[s];
            underlying_pb->setInternalState(s);
            algorithm->do_solve();
        }
        auto ubound = algorithm->getUpperBound();

        // Set the function that will be used to get interactively upper bounds
        vf->initialize(std::make_shared<State2OccupancyValueFunction<decltype(mdp->getInitialState()), TState>>(ubound));
    }
} // namespace sdm
