#include <sdm/world/belief_mdp.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TObservation>
    BeliefMDP<TState, TAction, TObservation>::BeliefMDP()
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    BeliefMDP<TBelief, TAction, TObservation>::BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp) : pomdp_(underlying_pomdp)
    {
        // Set initial belief state
        double proba = 0;
        for (auto &s : this->pomdp_->getStateSpace()->getAll())
        {
            proba = this->pomdp_->getStartDistrib().probabilities()[s];
            if (proba > 0)
            {
                this->initial_state_[s] = proba;
            }
        }
        this->current_state_ = this->initial_state_;
    }

    template <typename TState, typename TAction, typename TObservation>
    BeliefMDP<TState, TAction, TObservation>::BeliefMDP(std::string underlying_dpomdp) : BeliefMDP(std::make_shared<DiscretePOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::reset()
    {
        this->current_state_ = this->initial_state_;
        this->pomdp_->reset();
        return this->current_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief &BeliefMDP<TBelief, TAction, TObservation>::getState()
    {
        return this->current_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::tuple<TBelief, std::vector<double>, bool> BeliefMDP<TBelief, TAction, TObservation>::step(TAction action)
    {
        auto [next_obs, rewards, done] = this->pomdp_->step(action);
        this->current_state_ = this->nextState(this->current_state_, action, next_obs);
        return std::make_tuple(this->current_state_, rewards, done);
    }

    template <typename TBelief, typename TAction, typename TObservation>
    bool BeliefMDP<TBelief, TAction, TObservation>::isSerialized() const
    {
        return false;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    DiscretePOMDP *BeliefMDP<TBelief, TAction, TObservation>::getUnderlyingProblem()
    {
        return this->pomdp_.get();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::getInitialState()
    {
        return this->initial_state_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const
    {
        TBelief nextBelief;
        double tmp, obs_proba;
        for (const auto &nextState : this->pomdp_->getStateSpace()->getAll())
        {
            tmp = 0;
            for (const auto &state : this->pomdp_->getStateSpace()->getAll())
            {
                tmp += this->pomdp_->getStateDynamics()->getTransitionProbability(state, action, nextState) * belief.at(state);
            }
            obs_proba = this->pomdp_->getObsDynamics()->getObservationProbability(nextState,action, obs, nextState);

            if (obs_proba && tmp)
            {
                nextBelief[nextState] = obs_proba * tmp;
            }
        }
        // Normalize the belief
        double sum = nextBelief.norm_1();
        for (const auto &pair_s_p : nextBelief)
        {
            nextBelief[pair_s_p.first] = pair_s_p.second / sum;
        }
        return nextBelief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, number t, std::shared_ptr<HSVI<TBelief, TAction>> hsvi) const
    {
        // Select o* as in the paper
        number selected_o = 0;
        double max_o = -std::numeric_limits<double>::max(), tmp;
        TBelief select_next_state;
        for (const auto &o : this->pomdp_->getObsSpace()->getAll())
        {
            tmp = this->getObservationProbability(belief,action, o, belief);
            auto tau = this->nextState(belief, action, o);
            tmp *= hsvi->do_excess(tau, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                selected_o = o;
                select_next_state = tau;
            }
        }
        return select_next_state;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteSpace<TAction>> BeliefMDP<TBelief, TAction, TObservation>::getActionSpaceAt(const TBelief &)
    {
        return this->pomdp_->getActionSpace();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getReward(const TBelief &belief, const TAction &action) const
    {
        double r = 0;
        for (const auto &s : this->pomdp_->getStateSpace()->getAll())
        {
            r += belief.at(s) * this->pomdp_->getReward()->getReward(s, action);
        }
        return r;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(std::shared_ptr<ValueFunction<TBelief, TAction>> value_function, const TBelief &belief, const TAction &action, number t) const
    {
        double exp_next_v = 0;
        for (TObservation obs : this->pomdp_->getObsSpace()->getAll())
        {
            auto next_belief = this->nextState(belief, action, obs);
            exp_next_v += this->getObservationProbability(belief,action, obs, belief) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(const TBelief &, const TAction &action, const TObservation &obs, const TBelief &belief) const
    {
        double proba = 0, tmp;
        for (number s = 0; s < this->pomdp_->getStateSpace()->getNumItems(); s++)
        {
            tmp = 0;
            for (auto next_s : this->pomdp_->getStateSpace()->getAll())
            {
                tmp += this->pomdp_->getObsDynamics()->getDynamics(s, action, obs, next_s);
            }
            proba += tmp * belief.at(s);
        }
        return proba;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteMDP> BeliefMDP<TBelief, TAction, TObservation>::toMDP()
    {
        return this->pomdp_->toMDP();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> BeliefMDP<TBelief, TAction, TObservation>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

} // namespace sdm
