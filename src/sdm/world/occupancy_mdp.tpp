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

        this->ihistory_ = jhist;

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
    oState &OccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    oState OccupancyMDP<oState, oAction>::reset()
    {
        this->chistory_ = this->ihistory_;
        this->cstate_ = this->istate_;
        this->dpomdp_->reset();
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    std::tuple<oState, std::vector<double>, bool> OccupancyMDP<oState, oAction>::step(oAction joint_idr)
    {
        auto jaction = joint_idr.act(this->chistory_->getIndividualHistories()); // Select action based on joint separable decision rule
        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);           // Do step and get next observation and rewards
        this->chistory_ = this->chistory_->expand(next_obs);                     // Update the history based on the observation
        this->cstate_ = this->nextState(this->cstate_, joint_idr);               // Update the occupancy state
        return std::make_tuple(this->cstate_, rewards, done);                    // return the new occupancy state and the perceived rewards
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

        // Get possible histories for all agents
        auto vect_i_hist = ostate.getAllIndividualHistories();

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
    oState OccupancyMDP<oState, oAction>::nextState(const oState &ostate, const oAction &joint_idr, number, std::shared_ptr<HSVI<oState, oAction>>) const
    {
        oState new_ostate, tmp;
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
                    // Compute de proba of the next couple (state, joint history)
                    double proba = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(x, this->dpomdp_->getActionSpace()->joint2single(jaction), this->dpomdp_->getObsSpace()->joint2single(z), y);

                    // -> TO REPLACE WITH
                    // double proba = p_x_o.second * this->dpomdp_->getDynamics(x, jaction, z, y);

                    if (proba > 0)
                    {
                        new_ostate[new_index] = new_ostate.at(new_index) + proba;
                    }
                }
                // }
            }
        }

        // Compress the occupancy state
        // tmp = new_ostate;
        // new_ostate = new_ostate.compress();
        // if (tmp != new_ostate)
        // {
        //     std::cout << "\n\nBefore : " << tmp << std::endl;
        //     std::cout << "\nAfter : " << new_ostate << std::endl;
        // }
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

            // REPLACE BY IN LATER VERSION
            // r += p_x_o.second * this->dpomdp_->getReward(x, jaction);
        }
        return r;
    }

    template <typename oState, typename oAction>
    double OccupancyMDP<oState, oAction>::getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, number t) const
    {
        oState ost = this->nextState(ostate, oaction);
        // std::cout << "OState in Exp -->" << ost << std::endl;
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
