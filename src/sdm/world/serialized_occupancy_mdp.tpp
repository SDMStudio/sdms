#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

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
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                Tuple<typename oState::state_type, typename oState::jhistory_type, std::vector<number>> p_x_h(s, jhist, {});
                this->istate_[p_x_h] = this->dpomdp_->getStartDistrib().probabilities()[s];
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
    oState SerializedOccupancyMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    oState &SerializedOccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> SerializedOccupancyMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        // Get the individual possible histories for the current agent (as vector)
        auto indiv_hist = ostate.getIndividualHistories(ag_id);
        std::vector<typename oState::jhistory_type::element_type::ihistory_type> v_inputs(indiv_hist.begin(), indiv_hist.end());

        // Generate all individual decision rules for agent 'ag_id' (the current agent)
        FunctionSpace<oAction> f_indiv_dr_space(v_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());

        // Now we can return a discrete space of all indiv decision rules
        return std::make_shared<DiscreteSpace<oAction>>(f_indiv_dr_space.getAll());
    }

    template <typename oState, typename oAction>
    oState SerializedOccupancyMDP<oState, oAction>::nextState(const oState &ostate, const oAction &indiv_dr, int t, HSVI<oState, oAction> *hsvi) const
    {
        number ag_id = ostate.getCurrentAgentId();

        oState new_ostate;
        for (auto &p_x_o : ostate)
        {

            auto tuple_x_o_u = p_x_o.first;
            auto x = sdm::get<0>(tuple_x_o_u);
            auto o = sdm::get<1>(tuple_x_o_u);
            auto u = sdm::get<2>(tuple_x_o_u);

            if (ag_id != this->dpomdp_->getNumAgents() - 1)
            {
                u.push_back(indiv_dr(o->getIndividualHistory(ag_id)));
                new_ostate[sdm::make_tuple(x, o, u)] = p_x_o.second;
            }
            else
            {
                auto p_ihist = o->getIndividualHistory(ag_id);
                u.push_back(indiv_dr(p_ihist));
                for (auto &y : this->dpomdp_->getStateSpace()->getAll())
                {
                    for (auto &z : this->dpomdp_->getObsSpace()->getAll())
                    {
                        Tuple<typename oState::state_type, typename oState::jhistory_type, std::vector<number>> new_index(y, o->expand(z), {});
                        double proba = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(u), this->dpomdp_->getObsSpace()->joint2single(z), y);
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
    std::shared_ptr<Reward> SerializedOccupancyMDP<oState, oAction>::getReward() const
    {
        return this->dpomdp_->getReward();
    }

    // A VERIFIER !!!!!!!!!!!!!!!!!!!!
    template <typename oState, typename oAction>
    double SerializedOccupancyMDP<oState, oAction>::getReward(const oState &ostate, const oAction &indiv_dr) const
    {
        double r = 0;
        number ag_id = ostate.getCurrentAgentId();

        if (ag_id != this->dpomdp_->getNumAgents() - 1)
        {
            return 0;
        }

        for (auto &p_x_o : ostate)
        {
            auto tuple_s_h_a = p_x_o.first;
            auto state = sdm::get<0>(tuple_s_h_a);
            auto jhistory = sdm::get<1>(tuple_s_h_a);
            auto actions = sdm::get<2>(tuple_s_h_a);

            std::vector<typename oAction::output_type> jaction(actions.begin(), actions.end());

            // Add the last selected action (the action of agent 0)
            jaction.push_back(indiv_dr(jhistory->getIndividualHistory(ag_id)));

            r += p_x_o.second * this->getReward()->getReward(state, this->dpomdp_->getActionSpace()->joint2single(jaction));
        }
        return r;
    }

    template <typename oState, typename oAction>
    double SerializedOccupancyMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

} // namespace sdm
