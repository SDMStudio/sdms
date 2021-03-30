#include <sdm/utils/value_function/initializer/sawtooth_initializer.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/world_type.hpp>

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
    SawtoothInitializer<TState, TAction>::SawtoothInitializer(std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials)
    {
        std::cout << "In SawtoothInitializer" << std::endl;
    }

    template <typename TState, typename TAction>
    void SawtoothInitializer<TState, TAction>::init(ValueFunction<TState, TAction> *vf)
    {
        std::cout << "Init start" << std::endl;
        auto mdp = std::static_pointer_cast<typename WorldType<TState, TAction>::type>(vf->getWorld())->toMDP();
        auto underlying_pb = mdp->getUnderlyingProblem();
        std::cout << "Sawtooth Init after" << std::endl;

        auto algorithm = algo::makeMappedHSVI<decltype(mdp->getInitialState()), number>(mdp, "MDPInitializer", "BlindInitializer", underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon(), this->trials_, "mdp_init");
        for (auto &s : underlying_pb->getStateSpace()->getAll())
        {
            double proba_s = underlying_pb->getStartDistrib().probabilities()[s];
            if (proba_s > 0)
            {
                underlying_pb->setInternalState(s);
                algorithm->do_solve();
            }
        }
        std::cout<<"resultat :::::::: "<<algorithm->getUpperBound()->str();
        auto ubound = algorithm->getUpperBound();
        vf->initialize(std::make_shared<SawtoothValueFunction<decltype(mdp->getInitialState()), TState>>(ubound));
    }
} // namespace sdm
