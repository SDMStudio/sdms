#include <sdm/world/serialized_belief_mdp.hpp>

namespace sdm
{

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP()
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp) : serialized_mpomdp_(std::make_shared<SerializedMPOMDP<TBelief,TAction>>(underlying_dpomdp))
    {
        double proba = 0;
        for (const auto &s : this->serialized_mpomdp_->getStateSpaceAt(0)->getAll())
        {
            proba = this->serialized_mpomdp_->getStartDistrib().probabilities()[s.getState()];
            if (proba > 0)
            {
                this->istate_[s] = proba;
            }
        }
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedBeliefMDP<TBelief, TAction, TObservation>::SerializedBeliefMDP(std::string underlying_dpomdp) : SerializedBeliefMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp))
    {
    }

    template <typename TBelief, typename TAction, typename TObservation>
    bool SerializedBeliefMDP<TBelief, TAction, TObservation>::isSerialized() const
    {
        return this->serialized_mpomdp_->isSerialized();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    SerializedMPOMDP<TBelief,TAction> *SerializedBeliefMDP<TBelief, TAction, TObservation>::getUnderlyingProblem()
    {
        return this->serialized_mpomdp_.get();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::getInitialState()
    {
        return this->istate_;
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const
    {
        std::cout<<"\n ???? Next state 2 ";

        TBelief new_belief;
        number ag_id = belief.getCurrentAgentId();

        for (const auto &s_belief : belief )
        {
            auto s_belief_state = s_belief.first;
            auto proba = s_belief.second;
            auto x = belief.getHiddenState(s_belief_state);
            auto u = belief.getAction(s_belief_state);

            if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
            {
                std::cout<<"\n ???? Next state 2-1 ";

                u.push_back(action);
                typename TBelief::state_type s(x,u);
                new_belief[s] = proba;                
            }
            else
            {
                std::cout<<"\n ???? Next state 2-2 ";
                // double tmp, obs_proba;
                // for (const auto &nextState :this->serialized_mpomdp_->getStateSpace()->getAll())
                // {
                //     tmp = 0;
                //     for (const auto &s : this->serialized_mpomdp_->getStateSpace()->getAll())
                //     {
                //         tmp += this->serialized_mpomdp_->getStateDynamics()->getTransitionProbability(s, action, nextState) * belief.at(s);
                //     }
                //     obs_proba = this->serialized_mpomdp_->getObsDynamics()->getObservationProbability(action, obs, nextState);
                //     if (obs_proba && tmp)
                //     {
                //         new_belief[nextState] = obs_proba * tmp;
                //     }
                // }
            }
        }

        // Normalize the belief
        double sum = new_belief.norm_1();
        for (const auto &s_belief_state : new_belief)
        {
            new_belief[s_belief_state.first] = s_belief_state.second / sum;
        }

        return new_belief;
        // throw sdm::exception::NotImplementedException();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    TBelief SerializedBeliefMDP<TBelief, TAction, TObservation>::nextState(const TBelief &belief, const TAction &action, int t, HSVI<TBelief, TAction> *hsvi) const
    {
        std::cout<<"\n ???? Next state 1 !!!!! ";


        TBelief new_belief;
        number ag_id = belief.getCurrentAgentId();

        for (const auto &s_belief : belief )
        {
            auto s_belief_state = s_belief.first;
            auto proba = s_belief.second;
            auto x = belief.getHiddenState(s_belief_state);
            auto u = belief.getAction(s_belief_state);

            if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
            {
                std::cout<<"\n ???? Next state 1-1 ";
                u.push_back(action);
                typename TBelief::state_type s(x,u);
                new_belief[s] = proba;  
            }
            else
            {
                // Select o* as in the paper
                number selected_o = 0;
                double max_o = 0, tmp;

                std::cout<<"\n ???? Next state 1-2 ";

                // for (const auto o : this->serialized_mpomdp_->getObsSpace()->getAll())
                // {
                //     tmp = this->getObservationProbability(action, o, belief);
                //     auto tau = this->nextState(belief, action, o);
                //     tmp *= hsvi->do_excess(tau, t + 1);
                //     if (tmp > max_o)
                //     {
                //         max_o = tmp;
                //         selected_o = o;
                //     }
                // }
                // new_belief = this->nextState(belief, action, selected_o);
            }
        }
        return new_belief;
        // throw sdm::exception::NotImplementedException();
    }

    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<DiscreteSpace<TAction>> SerializedBeliefMDP<TBelief, TAction, TObservation>::getActionSpaceAt(const TBelief &belief)
    {
        return this->serialized_mpomdp_->getActionSpaceAt(belief.getCurrentAgentId());
    }

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getReward(const TBelief &belief, const TAction &action) const
    {
        double r = 0;
        for (const auto &belief_state : belief)
        {
            r += belief_state.second * this->serialized_mpomdp_->getReward(belief_state.first, action);
        }
        return r;
    }//Me paraît bien

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &belief, const TAction &action, int t) const
    {
        double exp_next_v = 0;
        for (const auto &obs : this->serialized_mpomdp_->getObsSpace()->getAll())
        {
            auto next_belief = this->nextState(belief, action, obs);
            exp_next_v += this->getObservationProbability(action, obs, belief) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    } // A vérifier, pas de parcourt de belief ? 

    template <typename TBelief, typename TAction, typename TObservation>
    double SerializedBeliefMDP<TBelief, TAction, TObservation>::getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const
    {
        double proba = 0, tmp;
        for (const auto &belief_state : belief)
        {
            tmp = 0;
            auto serialized_state = belief_state.first;
            for (auto next_serialized_state : this->serialized_mpomdp_->getNextSerializedStateSpace(serialized_state,action)->getAll())
            {
                tmp += this->serialized_mpomdp_->getObsDynamics(serialized_state, action, obs, next_serialized_state);
            }
            proba += tmp * belief_state.second;
        }
        return proba;
    } // A vérifier 


    template <typename TBelief, typename TAction, typename TObservation>
    std::shared_ptr<SerializedMMDP<>> SerializedBeliefMDP<TBelief, TAction, TObservation>::toMDP()
    {
        return this->serialized_mpomdp_->toMDP();
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
