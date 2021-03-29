#include <sdm/world/serialized_mdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    SerializedMDP<oState, oAction>::SerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {
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
        oState new_ostate;

        number ag_id = ostate.getCurrentAgentId();
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

            for (number state_ = 0; state_ < this->mmdp_->getStateSpace()->getNumItems(); state_++)
            {
                oState next_serial_state(state_, {});
                double tmp = this->mmdp_->getStateDynamics()->getTransitionProbability(x, this->mmdp_->getActionSpace()->joint2single(u), state_) * hsvi->do_excess(next_serial_state, t + 1);
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
        return r;
    }

    template <typename oState, typename oAction>
    double SerializedMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &action, int t) const
    {

        number ag_id = ostate.getCurrentAgentId();

        auto x = ostate.getState();
        auto u = ostate.getAction();
        u.push_back(action);

        if (ag_id != this->mmdp_->getNumAgents() - 1)
        {
            return value_function->getValueAt(oState(ostate.getState(), u), t + 1);
        }
        else
        {
            double tmp = 0;
            for (auto &y : this->mmdp_->getStateSpace()->getAll())
            {
                tmp += this->mmdp_->getStateDynamics()->getTransitionProbability(x, this->mmdp_->getActionSpace()->joint2single(u), y) * value_function->getValueAt(oState(y, {}), t + 1);
            }
            return tmp;
        }
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
    std::shared_ptr<SerializedMDP<oState, oAction>> SerializedMDP<oState, oAction>::getptr()
    {
        return SerializedMDP<oState, oAction>::shared_from_this();
    }

    template <typename oState, typename oAction>
    std::shared_ptr<SerializedMDP<oState, oAction>> SerializedMDP<oState, oAction>::toMDP()
    {
        return this->getptr();
    }

    template <typename oState, typename oAction>
    bool SerializedMDP<oState, oAction>::isSerialized() const
    {
        return true;
    }
}