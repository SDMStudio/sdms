#include <sdm/algorithms/hsvi.hpp>

namespace sdm
{
    // template <typename TState, typename TAction>
    // HSVI<TState, TAction>::HSVI(number trials, std::string results)
    // {
    //     this->MAX_TRIALS = trials;
    //     // this->saveStatsBegins(results);
    // }

    template <typename TState, typename TAction>
    HSVI<TState, TAction>::HSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> &world,
                                std::shared_ptr<ValueFunction<TState, TAction>> lower_bound,
                                std::shared_ptr<ValueFunction<TState, TAction>> upper_bound,
                                number planning_horizon,
                                double error,
                                number num_max_trials) : world_(world),
                                                         lower_bound_(lower_bound),
                                                         upper_bound_(upper_bound),
                                                         error_(error),
                                                         planning_horizon_(planning_horizon)
    {
        this->MAX_TRIALS = num_max_trials;
        this->do_initialize();
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_initialize()
    {
        this->initLogger();

        this->lower_bound_->initialize();
        this->upper_bound_->initialize();
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_solve()
    {
        TState start_state = this->world_->getInitialState();
        this->trial = 0;

        clock_t t_begin, t_end;
        t_begin = clock();
        do
        {
            std::cout << "Trial : " << this->trial << "\tError : " << this->do_excess(start_state, 0) << std::endl;
            // std::cout << "LB : " << this->lower_bound_->str() << "UB : " << this->upper_bound_->str() << std::endl;
            this->do_explore(start_state, 0);
            this->trial++;
        } while (!this->do_stop(start_state, 0));

        t_end = clock();
        float temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;

        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "Number trials : " << this->trial << "\tError : " << this->do_excess(start_state, 0) << std::endl;
        std::cout << "Final LB : \n"
                  << this->lower_bound_->str() << "Final UB : \n"
                  << this->upper_bound_->str() << std::endl;
        printf("\n\nTemps = %f s\n", temps);
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::do_excess(const TState &s, number h)
    {
        return (this->upper_bound_->getValueAt(s, h) - this->lower_bound_->getValueAt(s, h)) - this->error_ / std::pow(this->world_->getDiscount(), h);
    }

    template <typename TState, typename TAction>
    bool HSVI<TState, TAction>::do_stop(const TState &s, number h)
    {
        return ((this->do_excess(s, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_explore(const TState &s, number h)
    {
        if (!this->do_stop(s, h))
        {
            // Update bounds
            // std::cout << "1" << s << std::endl;
            this->lower_bound_->updateValueAt(s, h);
            // std::cout << "2" << s << std::endl;
            this->upper_bound_->updateValueAt(s, h);

            // std::cout << "3" << s << std::endl;
            // Select next action and state following search process
            TAction a = this->selectNextAction(s, h);
            // std::cout << "4" << s << std::endl;
            // std::cout << "4" << a << std::endl;

            TState s_ = this->world_->nextState(s, a, h, this);
            // std::cout << "5" << s << std::endl;
            // std::cout << s << "-"<< a << "-"<<h << std::endl;

            // Recursive explore
            this->do_explore(s_, h + 1);

            // std::cout << "6" << s << std::endl;

            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            // std::cout << "7" << s << std::endl;
            this->upper_bound_->updateValueAt(s, h);
            // std::cout << "8" << s << std::endl;
        }
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_test()
    {
        TState ostate = this->world_->getInitialState();
        TAction jdr;
        number end = (this->planning_horizon_ > 0) ? this->planning_horizon_ : 10;
        for (int i = 0; i < end; i++)
        {
            std::cout << "\n------------------------\nTIMESTEP " << i << "\n------------------------\n"
                      << std::endl;
            jdr = this->lower_bound_->getBestAction(ostate);
            std::cout << "#> State\n"
                      << ostate << "\n"
                      << std::endl;
            std::cout << "#> Action\n"
                      << jdr << std::endl;
            ostate = this->world_->nextState(ostate, jdr, i, this);
        }
    }

    template <typename TState, typename TAction>
    TAction HSVI<TState, TAction>::selectNextAction(const TState &s, number h)
    {
        return this->upper_bound_->getBestAction(s, h); // argmax_{a} q_value(s, a)
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::initLogger()
    {
        // this->logger.time.start();
    }

    // /**
    //  * @brief Select next state when state and action are discrete
    //  *
    //  * @param state the current state
    //  * @param action the greedy action
    //  * @return The next state to explore.
    //  */
    // template <>
    // number HSVI<number, number>::selectNextState(number state, number action, number d)
    // {
    //     double max = std::numeric_limits<double>::min();
    //     number amax = 0;
    //     for (number state_ = 0; state_ < this->world_->getNumStates(); state_++)
    //     {
    //         double tmp = this->world_->getTransitionProba(state, action, state_) * this->do_excess(state_, d + 1);
    //         if (tmp > max)
    //         {
    //             max = tmp;
    //             amax = state_;
    //         }
    //     }
    //     return amax;
    // }

    // template <>
    // number HSVI<number, number>::getInitialState()
    // {
    //     Vector d_init = this->world_->getStartDistrib();
    //     return 1;
    // }

    // /**
    //  * @brief Select next state when state and action are discrete
    //  *
    //  * @param state the current state
    //  * @param action the greedy action
    //  * @return The next state to explore.
    //  */
    // template <>
    // BeliefState HSVI<BeliefState, number>::selectNextState(BeliefState state, number action, number d)
    // {
    //     auto nextState = [](decltype(this->world_) w, BeliefState st, number action, number o, number d) {
    //         BeliefState nextBelief;
    //         double tmp;
    //         for (number s_ = 0; s_ < w->getNumStates(); s_++)
    //         {
    //             tmp = 0;
    //             for (number s = 0; s < w->getNumStates(); s++)
    //             {
    //                 tmp += w->getTransitionProba(s, action, s_) * st.at(s);
    //             }
    //             nextBelief[s_] = w->getObservationProbability(action, o, s_) * tmp;
    //         }
    //         // Normalize the belief
    //         double sum = nextBelief.norm_1();
    //         for (number s_ = 0; s_ < w->getNumStates(); s_++)
    //         {
    //             nextBelief[s_] = nextBelief[s_] / sum;
    //         }
    //         return nextBelief;
    //     };

    //     // compute o*
    //     number selected_o = 0;
    //     double max_o = 0, tmp;

    //     for (number o = 0; o < this->world_->getNumObservations()[0]; o++)
    //     {
    //         tmp = 0;
    //         for (number s = 0; s < this->world_->getNumStates(); s++)
    //         {
    //             tmp += this->world_->getObservationProbability(action, o, s) * state.at(s);
    //         }
    //         auto tau = nextState(this->world_, state, action, o, d);
    //         tmp *= this->do_excess(tau, d + 1);
    //         if (tmp > max_o)
    //         {
    //             max_o = tmp;
    //             selected_o = o;
    //         }
    //     }
    //     auto b = nextState(this->world_, state, action, selected_o, d);
    //     return b;
    // }

    // template <>
    // BeliefState HSVI<BeliefState, number>::getInitialState()
    // {
    //     Vector v_init = this->world_->getStartDistrib();
    //     BeliefState d_init(v_init.size(), 0.);
    //     for (int i = 0; i < v_init.size(); i++)
    //     {
    //         d_init[i] = v_init[i];
    //     }
    //     return d_init;
    // }
} // namespace sdm