#include <sdm/world/serialized_belief_mdp.hpp>

namespace sdm
{

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP()
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp) : mpomdp_(underlying_pomdp)
    {
        double proba = 0;
        for (auto &s : this->mpomdp_->getStateSpace()->getAll())
        {
            proba = this->mpomdp_->getStartDistrib().probabilities()[s];
            if (proba > 0)
            {
                this->istate_[SerializedState(s, {})] = proba;
            }
        }
        this->cstate_ = this->istate_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::string underlying_dpomdp) : SerializedBeliefMDP(std::make_shared<DiscretePOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    bool SerializedBeliefMDP<TBelief, TAction, TObservation>::isSerialized() const
    {
        return true;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    DiscretePOMDP *SerializedBeliefMDP<TBelief, TAction, TObservation>::getUnderlyingProblem()
    {
        return this->mpomdp_.get();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::getInitialState()
    {
        return this->istate_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const
    {

        TBelief new_belief;

        number ag_id = belief.getCurrentAgentId();
        auto x = belief.getState();
        auto u = belief.getAction();

        if (ag_id != this->mpomdp_->getNumAgents() - 1)
        {
            u.push_back(action);
            new_belief = TBelief(x, u);
        }
        else
        {
            BeliefState nextBelief;
            double tmp, obs_proba;
            for (number nextState = 0; nextState < this->mpomdp_->getStateSpace()->getNumItems(); nextState++)
            {
                tmp = 0;
                for (number s = 0; s < this->mpomdp_->getStateSpace()->getNumItems(); s++)
                {
                    tmp += this->mpomdp_->getStateDynamics()->getTransitionProbability(s, action, nextState) * x.at(s);
                }
                obs_proba = this->mpomdp_->getObsDynamics()->getObservationProbability(action, obs, nextState);
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
        }
        return new_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, int t, HSVI<TBelief, TAction> *hsvi) const
    {

        TBelief new_belief;

        number ag_id = belief.getCurrentAgentId();
        auto x = belief.getState();
        auto u = belief.getAction();

        if (ag_id != this->mpomdp_->getNumAgents() - 1)
        {
            u.push_back(action);
            new_belief = TBelief(x, u);
        }
        else
        {
            // Select o* as in the paper
            number selected_o = 0;
            double max_o = 0, tmp;

            for (number o = 0; o < this->mpomdp_->getObsSpace()->getNumItems(); o++)
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
            new_belief = this->nextState(belief, action, selected_o);
        }
        return new_belief;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteSpace<TAction>> SerializedBeliefMDP<TBelief, TAction, TObservation>::getActionSpaceAt(const TBelief &)
    {
        return this->mpomdp_->getActionSpace();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getReward(const TBelief &belief, const TAction &action) const
    {
        double r = 0;
        for (number s = 0; s < this->mpomdp_->getStateSpace()->getNumItems(); s++)
        {
            r += belief.at(s) * this->mpomdp_->getReward()->getReward(s, action);
        }
        return r;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &belief, const TAction &action, int t) const
    {
        double exp_next_v = 0;
        for (TObservation obs : this->mpomdp_->getObsSpace()->getAll())
        {
            auto next_belief = this->nextState(belief, action, obs);
            exp_next_v += this->getObservationProbability(action, obs, belief) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const
    {
        double proba = 0;
        auto &b = belief.getState();
        for (number s = 0; s < this->mpomdp_->getStateSpace()->getNumItems(); s++)
        {
            proba += this->mpomdp_->getObsDynamics()->getObservationProbability(action, obs, s) * b.at(s);
        }
        return proba;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteMDP> SerializedBeliefMDP<TBelief, TAction, TObservation>::toMDP()
    {
        return this->mpomdp_->toMDP();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> SerializedBeliefMDP<TBelief, TAction, TObservation>::toBeliefMDP()
    {
        return this->getptr();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> SerializedBeliefMDP<TBelief, TAction, TObservation>::getptr()
    {
        return SerializedBeliefMDP<TBelief, TAction, TObservation>::shared_from_this();
    }


} // namespace sdm
