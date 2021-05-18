namespace sdm
{
    namespace algo
    {
        template <typename TState, typename TAction>
        std::shared_ptr<sdm::HSVI<TState, TAction>> makeHSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name);

        template <typename TState, typename TAction>
        std::shared_ptr<sdm::ValueIteration<TState, TAction>> makeValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double discount, double error, number horizon);
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
    void MDPInitializer<TState, TAction>::init(std::shared_ptr<ValueFunction<TState, TAction>> vf)
    {
        // Get relaxed MDP problem and the underlying problem
        auto mdp = std::static_pointer_cast<typename WorldType<TState, TAction>::type>(vf->getWorld())->toMDP();
        auto underlying_pb = mdp->getUnderlyingProblem();

        if (this->algo_name_ == "ValueIteration")
        {
            auto value_iteration = algo::makeValueIteration<decltype(mdp->getInitialState()), number>(mdp, underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon());
            value_iteration->do_initialize();
            std::cout<<"\n initailise ;";
            std::cout<<"\n initailise ;";

            value_iteration->do_solve();

            vf->initialize(std::make_shared<State2OccupancyValueFunction<decltype(mdp->getInitialState()), TState>>(value_iteration->getResult()));
        }
        else
        {
            auto initial = underlying_pb->getInternalState();
            // Instanciate HSVI for MDP
            auto algorithm = algo::makeHSVI<decltype(mdp->getInitialState()), number>(mdp, "tabular", "tabular", "MaxInitializer", "MinInitializer", underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon(), this->trials_, "mdp_init");
            algorithm->do_initialize();

            // Solve HSVI from every possible initial state
            for (auto &s : underlying_pb->getStateSpace()->getAll())
            {
                underlying_pb->setInternalState(s);
                algorithm->do_solve();
            }
            
            auto ubound = algorithm->getUpperBound();

            underlying_pb->setInternalState(initial);

            vf->initialize(std::make_shared<State2OccupancyValueFunction<decltype(mdp->getInitialState()), TState>>(ubound));
        }
        // Set the function that will be used to get interactively upper bounds
    }
} // namespace sdm
