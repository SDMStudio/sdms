#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP()
    {
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp) : dpomdp_(underlying_dpomdp)
    {
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
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
                auto p_x_h = std::make_pair(s, jhist);
                this->istate_[p_x_h] = this->dpomdp_->getStartDistrib().probabilities()[s];
            }
        }
        this->cstate_ = this->istate_;
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::string underlying_dpomdp, number hist_length) : OccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename oState, typename oAction>
    OccupancyMDP<oState, oAction>::OccupancyMDP(std::string underlying_dpomdp) : OccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp))
    {
    }

    template <typename oState, typename oAction>
    void OccupancyMDP<oState, oAction>::compress()
    {
    }

    template <typename oState, typename oAction>
    oState &OccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    bool OccupancyMDP<oState, oAction>::isSerialized() const
    {
        return false;
    }

    template <typename oState, typename oAction>
    DiscreteDecPOMDP *OccupancyMDP<oState, oAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> OccupancyMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        using decision_rule_t = typename oAction::value_type;
        auto vect_i_hist = ostate.getAllIndividualHistories(); // get possible histories for all agents

        // Get individual decision rules for each agent
        std::vector<std::vector<decision_rule_t>> vect_i_dr = {};
        for (int ag_id = 0; ag_id < this->dpomdp_->getNumAgents(); ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            auto vect_inputs = sdm::tools::set2vector(vect_i_hist[ag_id]);
            FunctionSpace<decision_rule_t> f_indiv_dr_space(vect_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll());
            vect_i_dr.push_back(f_indiv_dr_space.getAll());
        }

        // Get joint decision rules for each agent
        std::vector<oAction> vect_j_dr = {};
        for (const auto &joint_idr : MultiDiscreteSpace<decision_rule_t>(vect_i_dr).getAll())
        {
            vect_j_dr.push_back(oAction(joint_idr));
        }

        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<oAction>>(vect_j_dr);
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::nextState(const oState &ostate, const oAction &joint_idr, int, HSVI<oState, oAction> *) const
    {
        oState new_ostate;
        // for all element in the support of the occupancy state
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            for (auto &y : this->dpomdp_->getStateSpace()->getAll())
            {
                // for (auto &u : this->dpomdp_->getActionSpace().getAll())
                // {
                for (auto &z : this->dpomdp_->getObsSpace()->getAll())
                {
                    Pair<typename oState::state_type, typename oState::jhistory_type> new_index(y, o->expand(z));
                    auto jaction = joint_idr.act(o->getIndividualHistories());
                    double proba = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(jaction), this->dpomdp_->getObsSpace()->joint2single(z), y);
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
    double OccupancyMDP<oState, oAction>::getReward(const oState &ostate, const oAction &joint_idr) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto x = p_x_o.first.first;
            auto o = p_x_o.first.second;
            auto jaction = joint_idr.act(o->getIndividualHistories());
            r += p_x_o.second * this->dpomdp_->getReward()->getReward(x, this->dpomdp_->getActionSpace()->joint2single(jaction));
        }
        return r;
    }

    template <typename oState, typename oAction>
    double OccupancyMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteMDP> OccupancyMDP<oState, oAction>::toMDP()
    {
        return this->dpomdp_->toMDP();
    }

    template <typename oState, typename oAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> OccupancyMDP<oState, oAction>::toBeliefMDP()
    {
        return this->dpomdp_->toBeliefMDP();
    }

} // namespace sdm
