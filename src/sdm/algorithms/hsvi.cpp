#include <sdm/algorithms/hsvi.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    HSVI<TState, TAction>::HSVI(number trials, std::string results)
    {
        this->MAX_TRIALS = trials;
        // this->saveStatsBegins(results);
    }

    template <typename TState, typename TAction>
    HSVI<TState, TAction>::HSVI(std::shared_ptr<POSG> &world,
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
        this->do_initialize();

        TState start_state = this->getInitialState();
        this->trial = 0;
        do
        {
            std::cout << "Trial : " << this->trial << "\tError : " << this->do_excess(start_state, 0) << std::endl;
            std::cout << "LB : " << this->lower_bound_->str() << "UB : " << this->upper_bound_->str() << std::endl;
            this->do_explore(start_state, 0);
            this->trial++;
        } while (!this->do_stop(start_state, 0));

        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "Number trials : " << this->trial << "\tError : " << this->do_excess(start_state, 0) << std::endl;
        std::cout << "Final LB : " << this->lower_bound_->str() << "Final UB : " << this->upper_bound_->str() << std::endl;
    }

    template <typename TState, typename TAction>
    double HSVI<TState, TAction>::do_excess(TState s, number h)
    {
        return (this->upper_bound_->getValueAt(s, h) - this->lower_bound_->getValueAt(s, h)) - this->error_ / std::pow(this->world_->getDiscount(), h);
    }

    template <typename TState, typename TAction>
    bool HSVI<TState, TAction>::do_stop(TState s, number h)
    {
        return ((this->do_excess(s, h) <= 0) || (this->trial > this->MAX_TRIALS));
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::do_explore(TState s, number h)
    {
        if (!this->do_stop(s, h))
        {
            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            this->upper_bound_->updateValueAt(s, h);

            TAction a = this->selectNextAction(s, h);
            TState s_ = this->selectNextState(s, a, h);

            // Recursive explore
            this->do_explore(s_, h + 1);

            // Update bounds
            this->lower_bound_->updateValueAt(s, h);
            this->upper_bound_->updateValueAt(s, h);
        }
    }

    template <typename TState, typename TAction>
    TAction HSVI<TState, TAction>::selectNextAction(TState s, number h)
    {
        return this->upper_bound_->getBestAction(s, h);
    }

    template <typename TState, typename TAction>
    void HSVI<TState, TAction>::initLogger()
    {
        // this->logger.time.start();
    }

    /**
     * @brief Select next state when state and action are discrete
     * 
     * @param state the current state
     * @param action the greedy action 
     * @return The next state to explore. 
     */
    template <>
    number HSVI<number, number>::selectNextState(number state, number action, number d)
    {
        double max = std::numeric_limits<double>::min();
        number amax = 0;
        for (number state_ = 0; state_ < this->world_->getNumStates(); state_++)
        {
            double tmp = this->world_->getTransitionProba(state, action, state_) * this->do_excess(state_, d + 1);
            if (tmp > max)
            {
                max = tmp;
                amax = state_;
            }
        }
        return amax;
    }

    template <>
    number HSVI<number, number>::getInitialState()
    {
        Vector d_init = this->world_->getStartDistrib();
        return 1;
    }

    /**
     * @brief Select next state when state and action are discrete
     * 
     * @param state the current state
     * @param action the greedy action 
     * @return The next state to explore. 
     */
    template <>
    BeliefState HSVI<BeliefState, number>::selectNextState(BeliefState state, number action, number d)
    {
        BeliefState nextBelief(state.size());

        // compute o*
        number o = 0;

        double obs_proba;
        for (number i = 0; i < this->world_->getNumStates(); i++)
        {
            obs_proba += this->world_->getObservationProbability(action, o, i) * state[i];
            double p_b_next = 0;
            for (number s_ = 0; s_ < this->world_->getNumStates(); s_++)
            {
                p_b_next += this->world_->getTransitionProba(s_, action, i);
            }
            nextBelief[i] = this->world_->getObservationProbability(action, o, i) * p_b_next;
        }
        return nextBelief;
    }

    template <>
    BeliefState HSVI<BeliefState, number>::getInitialState()
    {
        Vector v_init = this->world_->getStartDistrib();
        BeliefState d_init(v_init.size(), 0.);
        for (int i = 0; i < v_init.size(); i++)
        {
            d_init[i] = v_init[i];
        }
        return d_init;
    }

    template class HSVI<number, number>;

} // namespace sdm
