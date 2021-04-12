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
                this->istate_[s] = proba;
            }
        }
        this->cstate_ = this->istate_;
    }

    template <typename TState, typename TAction, typename TObservation>
    BeliefMDP<TState, TAction, TObservation>::BeliefMDP(std::string underlying_dpomdp) : BeliefMDP(std::make_shared<DiscretePOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::reset()
    {
        this->cstate_ = this->istate_;
        this->pomdp_->reset();
        return this->cstate_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief &BeliefMDP<TBelief, TAction, TObservation>::getState()
    {
        return this->cstate_;
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
        return this->istate_;
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
            obs_proba = this->pomdp_->getObsDynamics()->getObservationProbability(action, obs, nextState);

            if (obs_proba && tmp)
            {
                nextBelief[nextState] = obs_proba * tmp;
            }
        }

        std::cout<<"\n next Belief "<<nextBelief.str();
        // Normalize the belief
        double sum = nextBelief.norm_1();
        for (const auto &pair_s_p : nextBelief)
        {
            nextBelief[pair_s_p.first] = pair_s_p.second / sum;
        }
        return nextBelief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief BeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, int t, HSVI<TBelief, TAction> *hsvi) const
    {
        std::cout<<"\n belief : "<<belief.str();
        std::cout<<"\n belief : "<<std::hash<TBelief>()(belief);
        // Select o* as in the paper
        number selected_o = 0;
        double max_o = -std::numeric_limits<double>::max(), tmp;

        for (const auto &o : this->pomdp_->getObsSpace()->getAll())
        {
            tmp = this->getObservationProbability(action, o, belief);
            auto tau = this->nextState(belief, action, o);
            tmp *= hsvi->do_excess(tau, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                selected_o = o;
            }
        }
        return this->nextState(belief, action, selected_o);
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
    double BeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &belief, const TAction &action, int t) const
    {
        double exp_next_v = 0;
        for (TObservation obs : this->pomdp_->getObsSpace()->getAll())
        {
            auto next_belief = this->nextState(belief, action, obs);
            exp_next_v += this->getObservationProbability(action, obs, belief) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double BeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const
    {
        double proba = 0;
        for (number s = 0; s < this->pomdp_->getStateSpace()->getNumItems(); s++)
        {
            proba += this->pomdp_->getObsDynamics()->getObservationProbability(action, obs, s) * belief.at(s);
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
        return this->pomdp_->toBeliefMDP();
    }

} // namespace sdm
