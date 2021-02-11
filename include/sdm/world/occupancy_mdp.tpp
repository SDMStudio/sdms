#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::shared_ptr<DecPOMDP> underlying_dpomdp) : dpomdp_(underlying_dpomdp)
    {
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::shared_ptr<DecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
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

        for (typename oState::state_type s : this->dpomdp_->getStateSpace().getAll())
        {
            if (this->dpomdp_->getStartDistrib()[s] > 0)
            {
                Pair<typename oState::state_type, typename oState::jhistory_type> p_x_h(s, jhist);
                this->istate_[p_x_h] = this->dpomdp_->getStartDistrib()[s];
            }
        }
        this->cstate_ = this->istate_;
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::string underlying_dpomdp, number hist_length) : OccupancyMDP(std::make_shared<DecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::string underlying_dpomdp) : OccupancyMDP(std::make_shared<DecPOMDP>(underlying_dpomdp))
    {
    }

    template <typename oState, typename oAction>
    oState &OccupancyMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }
    template <typename oState, typename oAction>
    oState &OccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    DiscreteSpace<oAction> OccupancyMDP<oState, oAction>::getActionSpace(oState ostate)
    {
        auto vect_i_hist = ostate.getIndividualHistories();
        std::vector<std::vector<typename oAction::value_type>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            std::vector<typename oState::jhistory_type::element_type::ihistory_type> v_inputs(vect_i_hist[ag_id].begin(), vect_i_hist[ag_id].end());
            FunctionSpace<typename oAction::value_type> f_indiv_dr_space(v_inputs, this->dpomdp_->getActionSpace().getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Now we can return a discrete space of all joint decision rules
        return DiscreteSpace(MultiDiscreteSpace<typename oAction::output_type>(vect_i_dr).getAll());
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::nextState(oState ostate, oAction joint_idr, int t, HSVI<oState, oAction> *hsvi) const
    {
        // std::cout << ostate << std::endl;
        // std::cout << joint_idr << std::endl;

        oState new_ostate;
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            for (auto &y : this->dpomdp_->getStateSpace().getAll())
            {
                // for (auto &u : this->dpomdp_->getActionSpace().getAll())
                // {
                for (auto &z : this->dpomdp_->getObsSpace().getAll())
                {
                    Pair<typename oState::state_type, typename oState::jhistory_type> new_index(y, o->expand(z));
                    std::vector<typename oAction::value_type::output_type> jaction;
                    for (int i = 0; i < joint_idr.size(); i++)
                    {
                        auto p_ihist = o->getIndividualHistory(i);
                        jaction.push_back(joint_idr[i](p_ihist));
                    }
                    double proba = p_x_o.second * this->dpomdp_->getDynamics(x, this->dpomdp_->getActionSpace().joint2single(jaction), this->dpomdp_->getObsSpace().joint2single(z), y);
                    if (proba > 0)
                    {
                        new_ostate[new_index] = new_ostate.at(new_index) + proba;
                    }
                }
            }
            // }
        }
        return new_ostate;
    }


    template <typename oState, typename oAction>
    Reward OccupancyMDP<oState, oAction>::getReward()
    {
        return this->dpomdp_->getReward();
    }

    template <typename oState, typename oAction>
    double OccupancyMDP<oState, oAction>::getReward(oState ostate, oAction joint_idr) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto state = p_x_o.first.first;
            auto jhistory = p_x_o.first.second;
            std::vector<typename oAction::value_type::output_type> jaction;
            for (int i = 0; i < joint_idr.size(); i++)
            {
                jaction.push_back(joint_idr[i](jhistory->getIndividualHistory(i)));
            }
            r += p_x_o.second * this->dpomdp_->getReward(state, jaction);
        }
        return r;
    }

    template <typename oState, typename oAction>
    double OccupancyMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, oState ostate, oAction oaction, int t) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

} // namespace sdm