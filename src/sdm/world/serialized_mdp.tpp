#include <sdm/world/serialized_mdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    SerializedMDP<oState, oAction>::SerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {   
        /*
        if(this->istate_.first + 1 <  this->mmdp_->getStateSpace()->getNumItems())
        {
            this->istate_ = oState(this->istate_.first +1,{});
        }else
        {
            this->istate_ = oState(0,{});
        }*/
    }

    template <typename oState, typename oAction>
    SerializedMDP<oState, oAction>::SerializedMDP(std::string underlying_mmdp) : SerializedMDP(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

    template <typename oState, typename oAction>
    oState SerializedMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    oState SerializedMDP<oState, oAction>::nextState(const oState &ostate, const oAction &action, int t, HSVI<oState, oAction> *hsvi) const
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

            for (number state_ = 0; state_<this->mmdp_->getStateSpace()->getNumItems(); state_++)
            {
                //std::cout<<"zzzz \n";
                //std::make_pair(0,{});
                //std::cout<<a;
                //std::cout<<"qqqq \n";
                //std::cout<<hsvi->do_excess(a, t+1);

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
    std::shared_ptr<DiscreteSpace<oAction>> SerializedMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        return std::make_shared<DiscreteSpace<oAction>>(this->mmdp_->getActionSpace()->getSpace(ag_id)->getAll());
    }

    template <typename oState, typename oAction>
    double SerializedMDP<oState, oAction>::getReward(const oState &ostate, const oAction &action) const
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

        r = this->mmdp_->getReward()->getReward(x, this->mmdp_->getActionSpace()->joint2single(u));
        // A voir si cela foncitonne mais cela ne me paraît pas faux non plus

        /*
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
        }*/

        return r;
    }

    template <typename oState, typename oAction>
    double SerializedMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &action, int t) const
    {
        oState ost = this->nextState(ostate, action);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    double SerializedMDP<oState, oAction>::getDiscount(int t) const
    {

        if (this->getNumberAgent() > 1)
        {
            if (t % this->getNumberAgent() != this->getNumberAgent() - 1)
            {
                return 1.0;
            }
        }
        return this->mmdp_->getDiscount();
    }

    template <typename oState, typename oAction>
    DiscreteMMDP *SerializedMDP<oState, oAction>::getUnderlyingProblem()
    {
        return this->mmdp_.get();
    }
    
    template <typename oState, typename oAction>
    bool SerializedMDP<oState, oAction>::isSerialized() const
    {
        return true;
    }
}