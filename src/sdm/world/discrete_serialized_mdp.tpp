 #include <sdm/world/discrete_serialized_mdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
 

template <typename oState, typename oAction>
    DiscreteSerializedMDP<oState, oAction>::DiscreteSerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {
    }
/*
    template <typename oState, typename oAction>
    DiscreteSerializedMDP<oState, oAction>::DiscreteSerializedMDP(std::shared_ptr<DiscreteMDP> underlying_mdp, number hist_length) : dpomdp_(underlying_dpomdp)
    {

        typename oState::jhistory_type jhist;
        if (hist_length > 0)
        {
            jhist = std::make_shared<typename oState::jhistory_type::element_type>(this->dpomdp_->getNumAgents(), hist_length);
        }
        else
        {
            jhist = std::make_shared<typename oState::jhistory_type::element_type>(this->dpomdp_->getNumAgents());
        }

        for (typename oState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                Tuple<typename oState::state_type, typename oState::jhistory_type, std::vector<number>> p_x_h(s, jhist, {});
                this->istate_[p_x_h] = this->dpomdp_->getStartDistrib().probabilities()[s];
            }
        }
        this->cstate_ = this->istate_;
    }*/
    /*
    template <typename oState, typename oAction>
    SerializedOccupancyMDP<oState, oAction>::SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }*/

    template <typename oState, typename oAction>
    DiscreteSerializedMDP<oState, oAction>::DiscreteSerializedMDP(std::string underlying_mmdp) : DiscreteSerializedMDP(std::make_shared<DiscreteMDP>(underlying_mmdp))
    {
    }

    template <typename oState, typename oAction>
    oState DiscreteSerializedMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    oState &DiscreteSerializedMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> DiscreteSerializedMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        return std::make_shared<DiscreteSpace<oAction>>(this->mmdp_->getActionSpace()->getSpace(ag_id)->getAll());
    }

    template <typename oState, typename oAction>
    oState DiscreteSerializedMDP<oState, oAction>::nextState(const oState &ostate, const oAction &indiv_dr, int t, HSVI<oState, oAction> *hsvi) const
    {
        number ag_id = ostate.getCurrentAgentId();

        oState new_ostate;
        for (auto &p_x_u : ostate)
        {

            auto pair_x_u = p_x_u.first;
            auto x = pair_x_u.first;
            auto u = pair_x_u.second;

            if (ag_id != this->mmdp_->getNumAgents() - 1)
            {
                u.push_back(indiv_dr(ostate)); // cela n'est pas bon, mais pour le moment je ne sais pas le remplacer
                //u.push_back(indiv_dr) ? 
                new_ostate[std::make_pair(x, u)] = p_x_u.second;
            }
            else
            {
                u.push_back(indiv_dr(ostate));
                
                for (auto &y : this->mmdp_->getStateSpace()->getAll())
                {
                    Pair<typename oState::state_type, std::vector<number>> new_index(y, {});
                    
                    double proba = p_x_u.second * this->getStateDynamics()->getTransitionProbability(ostate, this->mmdp_->getActionSpace()->joint2single(u), y) * hsvi->do_excess(y, t + 1);
                    //cela non plus 
                    
                    if (proba > 0)
                    {
                        new_ostate[new_index] = new_ostate.at(new_index) + proba;
                    }
                }
            }
        }
        return new_ostate;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<Reward> DiscreteSerializedMDP<oState, oAction>::getReward() const
    {
        return this->mmdp_->getReward();
    }

    template <typename oState, typename oAction>
    double DiscreteSerializedMDP<oState, oAction>::getReward(const oState &ostate, const oAction &indiv_dr) const
    {
        double r = 0;
        number ag_id = ostate.getCurrentAgentId();

        if (ag_id != this->mmdp_->getNumAgents() - 1)
        {
            return 0;
        }

        for (auto &p_x_u : ostate)
        {
            auto pair_x_u = p_x_u.first;
            auto state = pair_x_u.first;
            auto actions = pair_x_u.second;

            std::vector<typename oAction::output_type> jaction(actions.begin(), actions.end());
            //Pas bon

            // Add the last selected action (the action of agent 0)
            //jaction.push_back(indiv_dr(jhistory->getIndividualHistory(ag_id))); 
            // Pas bon
            

            r += p_x_u.second * this->getReward()->getReward(state, this->mmdp_->getActionSpace()->joint2single(jaction));
        }
        return r;
    }

    template <typename oState, typename oAction>
    double DiscreteSerializedMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    int DiscreteSerializedMDP<oState, oAction>::getNumberAgent() const
    {
        return this->mmdp_->getNumAgents();
    }

    template <typename oState,typename oAction>
    double DiscreteSerializedMDP<oState,oAction>::getDiscount(int t) const
    {
                
        if(this->getNumberAgent() >1)
        {
            if(t%this->getNumberAgent() != this->getNumberAgent() -1)
            {
                return 1.0;
            }
        }
        return this->mmdp_->getDiscount();
    }

    // ************
    
    /*
    SerialDiscreteMDP::SerialDiscreteMDP()
    {
    }

    SerialDiscreteMDP::SerialDiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
    {
    }

    SerialDiscreteMDP::SerialDiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::discrete_distribution<number> start_distrib)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
    {
    }

    SerialDiscreteMDP::SerialDiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
    {
    }

    SerialDiscreteMDP::SerialDiscreteMDP(std::string &filename)
    {
        *this = *(parser::parse_file(filename.c_str())->toPOMDP()->toMDP());
    }

    // SolvableByHSVI interface implementation
    number SerialDiscreteMDP::getInitialState()
    {
        return this->getInternalState();
    }

    number SerialDiscreteMDP::nextState(const number &state, const number &action, int t, HSVI<number, number> *hsvi) const
    {
        double max = std::numeric_limits<double>::min();
        number amax = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            double tmp = this->getStateDynamics()->getTransitionProbability(state, action, state_) * hsvi->do_excess(state_, t + 1);
            if (tmp > max)
            {
                max = tmp;
                amax = state_;
            }
        }
        return amax;
    }

    std::shared_ptr<DiscreteSpace<number>> SerialDiscreteMDP::getActionSpaceAt(const number &state)
    {
        return this->getActionSpace();
    }

    std::shared_ptr<Reward> SerialDiscreteMDP::getReward() const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward();
    }

    double SerialDiscreteMDP::getReward(const number &state, const number &action) const
    {
        return this->getReward()->getReward(state, action);
    }

    double SerialDiscreteMDP::getExpectedNextValue(ValueFunction<number, number> *value_function, const number &state, const number &action, int t) const
    {
        double tmp = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            tmp += this->getStateDynamics()->getTransitionProbability(state, action, state_) * value_function->getValueAt(state_, t + 1);
        }
        return tmp;
    }

    double SerialDiscreteMDP::getDiscount()
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getDiscount();
    }

    void SerialDiscreteMDP::setDiscount(double discount)
    {
        FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::setDiscount(discount);
    }
    */
}