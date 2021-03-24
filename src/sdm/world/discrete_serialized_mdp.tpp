 #include <sdm/world/discrete_serialized_mdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
 

template <typename oState, typename oAction>
    DiscreteSerializedMDP<oState, oAction>::DiscreteSerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {
    }

    template <typename oState, typename oAction>
    DiscreteSerializedMDP<oState, oAction>::DiscreteSerializedMDP(std::string underlying_mmdp) : DiscreteSerializedMDP(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

    template <typename oState, typename oAction>
    oState DiscreteSerializedMDP<oState, oAction>::getInitialState()
    {
        if(this->istate_.first + 1 <  this->mmdp_->getStateSpace()->getNumItems())
        {
            this->istate_ = oState(this->istate_.first +1,{});
        }else
        {
            this->istate_ = oState(0,{});
        }
        return this->istate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> DiscreteSerializedMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        return std::make_shared<DiscreteSpace<oAction>>(this->mmdp_->getActionSpace()->getSpace(ag_id)->getAll());
    }

    template <typename oState, typename oAction>
    oState DiscreteSerializedMDP<oState, oAction>::nextState(const oState &ostate, const oAction &action, int t, HSVI<oState, oAction> *hsvi) const
    {
        number ag_id = ostate.getCurrentAgentId();

        oState new_ostate;

        auto x = ostate.getState();
        auto u = ostate.getAction();

        if (ag_id != this->mmdp_->getNumAgents() - 1)
        {
            u.push_back(action); 
             
            new_ostate = oState(x, u);
        }
        else
        {

            double max = std::numeric_limits<double>::min();
            number amax = 0;

            u.push_back(action); 

            for (const number & state_ : this->mmdp_->getStateSpace()->getAll())
            {
                double tmp = this->mmdp_->getStateDynamics()->getTransitionProbability(x, this->mmdp_->getActionSpace()->joint2single(u), state_);//* hsvi->do_excess(state_, t + 1);
                if (tmp > max)
                {
                    max = tmp;
                    amax = state_;
                }
            }
            new_ostate = oState(amax, {});
        }
        return new_ostate;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<Reward> DiscreteSerializedMDP<oState, oAction>::getReward() const
    {
        return this->mmdp_->getReward();
    }

    template <typename oState, typename oAction>
    double DiscreteSerializedMDP<oState, oAction>::getReward(const oState &ostate, const oAction &action) const
    {
        double r = 0;
        number ag_id = ostate.getCurrentAgentId();

        if (ag_id != this->mmdp_->getNumAgents() - 1)
        {
            return 0;
        }

        auto x = ostate.getState();
        auto u = ostate.getAction();
        
        
        u.push_back(action); 

        r = this->getReward()->getReward(x, this->mmdp_->getActionSpace()->joint2single(u));
        return r;
    }

    template <typename oState, typename oAction>
    double DiscreteSerializedMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &action, int t) const
    {
        oState ost = this->nextState(ostate, action);
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
}