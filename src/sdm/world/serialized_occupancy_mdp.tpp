#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    SerializedOccupancyMDP<oState, oAction>::SerializedOccupancyMDP()
    {
    }

    template <typename oState, typename oAction>
    SerializedOccupancyMDP<oState, oAction>::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp) : dpomdp_(underlying_dpomdp)
    {
    }

    template <typename oState, typename oAction>
    SerializedOccupancyMDP<oState, oAction>::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
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
            auto x = s.getState();
            if (this->dpomdp_->getStartDistrib().probabilities()[x] > 0)
            {
                Pair<typename oState::state_type, typename oState::jhistory_type> p_s_o(s, jhist);
                this->istate_[p_s_o] = this->dpomdp_->getStartDistrib().probabilities()[x];
            }
        }
        this->cstate_ = this->istate_;
    }

    template <typename oState, typename oAction>
    SerializedOccupancyMDP<oState, oAction>::SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename oState, typename oAction>
    SerializedOccupancyMDP<oState, oAction>::SerializedOccupancyMDP(std::string underlying_dpomdp) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp))
    {
    }

    template <typename oState, typename oAction>
    oState &SerializedOccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    bool SerializedOccupancyMDP<oState, oAction>::isSerialized() const
    {
        return true;
    }

    template <typename oState, typename oAction>
    DiscreteDecPOMDP *SerializedOccupancyMDP<oState, oAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename oState, typename oAction>
    oState SerializedOccupancyMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> SerializedOccupancyMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        // Get the individual possible histories for the current agent (as vector)
        auto indiv_hist = sdm::tools::set2vector(ostate.getIndividualHistories(ag_id));

        // Generate all individual decision rules for agent 'ag_id' (the current agent)
        FunctionSpace<oAction> f_indiv_dr_space(indiv_hist, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());

        // Now we can return a discrete space of all indiv decision rules
        return std::make_shared<DiscreteSpace<oAction>>(f_indiv_dr_space.getAll());
    }

    template <typename oState, typename oAction>
    oState SerializedOccupancyMDP<oState, oAction>::nextState(const oState &ostate, const oAction &indiv_dr, number, HSVI<oState, oAction> *) const
    {
        number ag_id = ostate.getCurrentAgentId();

        oState new_ostate;

        for (auto &p_s_o : ostate)
        {
            auto pair_s_o = p_s_o.first;
            auto x = ostate.getHiddenState(pair_s_o);
            auto o = pair_s_o.second;
            auto u = ostate.getAction(pair_s_o);


            auto p_ihist = o->getIndividualHistory(ag_id);
            u.push_back(indiv_dr.act(p_ihist));

            if (ag_id != this->dpomdp_->getNumAgents() - 1)
            {
                typename oState::state_type s(x,u);
                Pair<typename oState::state_type, typename oState::jhistory_type> s_o(s, o);
                new_ostate[s_o] = p_s_o.second;
            }
            else
            {
                for (typename oState::state_type y : this->dpomdp_->getStateSpace()->getAll())
                {                    
                    for (auto &z : this->dpomdp_->getObsSpace()->getAll())
                    {
                        Pair<typename oState::state_type, typename oState::jhistory_type> new_index(y, o->expand(z));
                        double proba = p_s_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(u), this->dpomdp_->getObsSpace()->joint2single(z), y.getState());
                        if (proba > 0)
                        {
                            new_ostate[new_index] = new_ostate.at(new_index) + proba;
                        }
                    }
                }
            }
        }
        return new_ostate;
    }

    template <typename oState, typename oAction>
    double SerializedOccupancyMDP<oState, oAction>::getReward(const oState &ostate, const oAction &indiv_dr) const
    {
        double r = 0;
        number ag_id = ostate.getCurrentAgentId();
        

        if (ag_id != this->dpomdp_->getNumAgents() - 1)
        {
            return 0;
        }

        for (auto &p_s_o : ostate)
        {
            auto pair_s_o = p_s_o.first;
            auto x = pair_s_o.first.getState();
            auto u = pair_s_o.first.getAction();
            auto o = pair_s_o.second;

            std::vector<typename oAction::output_type> jaction(u.begin(), u.end());

            // Add the last selected action (the action of agent 0)
            jaction.push_back(indiv_dr.act(o->getIndividualHistory(ag_id)));

            r += p_s_o.second * this->dpomdp_->getReward()->getReward(x, this->dpomdp_->getActionSpace()->joint2single(jaction));
        }
        return r;
    }

    template <typename oState, typename oAction>
    double SerializedOccupancyMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, number t) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    double SerializedOccupancyMDP<oState, oAction>::getDiscount(number t) const
    {

        if (this->dpomdp_->getNumAgents() > 1)
        {
            if (t % this->dpomdp_->getNumAgents() != this->dpomdp_->getNumAgents() - 1)
            {
                return 1.0;
            }
        }
        return this->dpomdp_->getDiscount();
    }


    template <typename oState, typename oAction>
    std::shared_ptr<SerializedMDP<>> SerializedOccupancyMDP<oState, oAction>::toMDP()
    {
        return std::make_shared<SerializedMDP<>>(this->dpomdp_->toMMDP());
    }

    // template <typename oState, typename oAction>
    // std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedOccupancyMDP<oState, oAction>::toBeliefMDP()
    // {
    //     return std::make_shared<SerializedBeliefMDP<>>(this->dpomdp_->toMPOMDP());
    // }
} // namespace sdm
