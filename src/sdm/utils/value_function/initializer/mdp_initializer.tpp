namespace sdm
{
    namespace algo
    {
        std::shared_ptr<sdm::HSVI> makeHSVI(std::shared_ptr<SolvableByHSVI> problem, std::string upper_bound, std::string lower_bound, std::string ub_init_name, std::string lb_init_name, double discount, double error, number horizon, int trials, std::string name, std::string current_type_of_resolution , number BigM , std::string type_sawtooth_linear_programming );

        // std::shared_ptr<sdm::ValueIteration> makeValueIteration(std::shared_ptr<SolvableByHSVI> problem, double discount, double error, number horizon);
    }
}
namespace sdm
{
    MDPInitializer::MDPInitializer(std::shared_ptr<SolvableByHSVI> world,std::string algo_name, double error, int trials) : algo_name_(algo_name), error_(error), trials_(trials), world_(world)
    {
        std::cout << "In MDPInitializer" << std::endl;
    }
    void MDPInitializer::init(std::shared_ptr<ValueFunction> vf)
    {
        // Get relaxed MDP problem and thgetUnderlyingProbleme underlying problem
        auto mdp = std::dynamic_pointer_cast<MDPInterface>(this->world_->getUnderlyingProblem());
        std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

        if (this->algo_name_ == "ValueIteration")
        {
            std::cout<<"Value Iteration";
            // auto value_iteration = algo::makeValueIteration(mdp, underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon());
            
            // value_iteration->do_initialize();
            // value_iteration->do_solve();

            // vf->initialize(std::make_shared<State2OccupancyValueFunction<decltype(mdp->getInitialState()), TState>>(value_iteration->getResult()));
        }
        else
        {
            std::cout<<"MDP Iteration";

            auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);

            auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
            auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);

            auto lb = std::make_shared<PointSetValueFunction>(tabular_backup,mdp->getHorizon(),init_lb);
            auto ub = std::make_shared<PointSetValueFunction>(tabular_backup,mdp->getHorizon(),init_ub);

            auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), this->error_);


            // auto initial = underlying_pb->getInternalState();
            // // Instanciate HSVI for MDP
            // auto algorithm = std::make_shared<HSVI>(mdp, ....); 
            
            
            // algo::makeHSVI(mdp, "tabular", "tabular", "MaxInitializer", "MinInitializer", underlying_pb->getDiscount(), this->error_, underlying_pb->getPlanningHorizon(), this->trials_, "mdp_init", "BigM",100,"Full");
            // algorithm->do_initialize();

            // // Solve HSVI from every possible initial state
            // for (const auto &s : underlying_pb->getAllStates())
            // {
            //     underlying_pb->setInternalState(s);
            //     algorithm->do_solve();
            // }
            
            // auto ubound = algorithm->getUpperBound();

            // underlying_pb->setInternalState(initial);

            // vf->initialize(std::make_shared<State2OccupancyValueFunction<decltype(mdp->getInitialState()), TState>>(ubound));
        }
        // Set the function that will be used to get interactively upper bounds
    }
} // namespace sdm
