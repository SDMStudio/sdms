


namespace sdm
{
    template <typename TState, typename TAction>
    ValueIteration<TState,TAction>::ValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem,double discount, double error, int horizon) : problem_(problem)
    {
        problem->getUnderlyingProblem()->setDiscount(discount);
        problem->getUnderlyingProblem()->setPlanningHorizon(horizon);
        problem->getUnderlyingProblem()->setInternalState(0);

    }
    /*
    template <typename TState, typename TAction>
    void ValueIteration<TState,TAction>::policy_iteration_inf()
    {
        auto under_pb = this->problem_->getUnderlyingProblem();

        MappedVector<TState,double> policy_evaluation_tau1;
        MappedVector<TState,double> policy_evaluation;

        for(const auto &state : under_pb->getStateSpace()->getAll())
        {
            policy_evaluation[state] = 0; //Le reste est déjà implémenté derrière
            policy_evaluation_tau1[state] = 0;
        }

        do
        {
            policy_evaluation = policy_evaluation_tau1;

            for(const auto &state : under_pb->getStateSpace()->getAll())
            {
                std::vector<double> resultat_a;
                for(const auto &a : under_pb->getActionSpace()->getAll())
                {                
                    double resultat = under_pb->getReward(state, a);
                    //double reward = under_pb->getReward(state, a);
                    for(auto &state_2 : under_pb->getStateSpace()->getAll())
                    {
                        resultat += under_pb->getStateDynamics()->getTransitionProbability(state, a, state_2)*under_pb->getDiscount()*policy_evaluation[state];
                    }
                    resultat_a.push_back(resultat);
                }
                policy_evaluation_tau1[state] = *std::max_element(resultat_a.begin(), resultat_a.end());
                
            }
        } while (this->borne(policy_evaluation_tau1,policy_evaluation));

        MappedVector<TState,TAction> politique;
        for(const auto &state : under_pb->getStateSpace()->getAll())
        {
            double resultat_a =std::numeric_limits<double>::min();
            TAction argmax_a;
            for(const auto &a : under_pb->getActionSpace()->getAll())
            {                
                double resultat = under_pb->getReward(state, a);
                //double reward = under_pb->getReward(state, a);
                for(auto &state_2 : under_pb->getStateSpace()->getAll())
                {
                    resultat += under_pb->getStateDynamics()->getTransitionProbability(state, a, state_2)*under_pb->getDiscount()*policy_evaluation[state];
                }

                if (resultat >resultat_a)
                {
                    resultat_a = resultat;
                    argmax_a = a;
                }                
            }
            politique[state] = argmax_a;
            
        }
        std::cout<<"Politique : "<<politique;
        std::cout<<"Final value : "<<policy_evaluation_tau1;
    }*/

    template <typename TState, typename TAction>
    void ValueIteration<TState,TAction>::policy_iteration_non_inf()
    {
        auto under_pb = this->problem_->getUnderlyingProblem();

        auto ub_init = sdm::makeInitializer<TState, TAction>("MaxInitializer");

        auto policy_evaluation = sdm::MappedValueFunction<TState, TAction>(this->problem_, under_pb->getPlanningHorizon());
        auto policy_evaluation_tau1 = sdm::MappedValueFunction<TState, TAction>(this->problem_, under_pb->getPlanningHorizon());

        policy_evaluation.initialize();
        policy_evaluation_tau1.initialize();

        std::cout<<"Init : "<<policy_evaluation.str();

        do
        {
            std::cout<<"\n a";
            policy_evaluation = policy_evaluation_tau1;

            for(int t = under_pb->getPlanningHorizon()-1; t>=0;t--)
            {
                // Il y a un problème avec : policy_evaluation.getSupport(t)). Il est totalement vide au début, mais pourtant le .initialize() a été effectué
                // Je pense qu'il va falloir modifier l'initialisation pour avoir au moins les états possibles
                for(auto &state : under_pb->getStateSpace()->getAll() )
                {
                    policy_evaluation_tau1.updateValueAt(state,t,policy_evaluation.getQValueAt(state, t)->max());
                }
            }
        } while (this->borne(policy_evaluation_tau1,policy_evaluation));

        std::cout<<"Final value : "<<policy_evaluation_tau1.str();
    }

    template <typename TState, typename TAction>
    void ValueIteration<TState,TAction>::policy_iteration()
    {
        std::cout<<"Horizon : "<<this->problem_->getUnderlyingProblem()->getPlanningHorizon();
        if(this->problem_->getUnderlyingProblem()->getDiscount() != 1)
        {
            //this->policy_iteration_inf(); Finalement, on peut écrire les deux problèmes avec une seule
            this->policy_iteration_non_inf();
        }else
        {
            this->policy_iteration_non_inf();
        }

    }
    
    template <typename TState, typename TAction>
    bool ValueIteration<TState,TAction>::borne(sdm::MappedValueFunction<TState, TAction> policy_evaluation_tempo,sdm::MappedValueFunction<TState, TAction> policy_evaluation,double epsilon)
    {
        double max_value = std::numeric_limits<double>::min();
        for(auto &state : policy_evaluation_tempo.getSupport(0))
        {
            max_value = std::max(std::abs(policy_evaluation_tempo.getValueAt(state,0) - policy_evaluation.getValueAt(state,0)),max_value);
        }
        if (max_value<= epsilon)
        {
            return false;
        }else
        {
            return true;
        }
    }
    /*
    template <typename TState, typename TAction>
    bool ValueIteration<TState,TAction>::borne(MappedVector<TState,double> policy_evaluation_tempo,MappedVector<TState,double> policy_evaluation,double epsilon)
    {
        double max_value = std::numeric_limits<double>::min();
        for(const auto &state : policy_evaluation_tempo.getIndexes())
        {
            max_value = std::max(std::abs(policy_evaluation_tempo[state] - policy_evaluation[state]),max_value);
        }
        if (max_value<= epsilon)
        {
            return false;
        }else
        {
            return true;
        }
    }*/
}