#include <sdm/world/private_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    PrivateOccupancyMDP<oState, oAction>::PrivateOccupancyMDP()
    {
    }

    template <typename oState, typename oAction>
    PrivateOccupancyMDP<oState, oAction>::PrivateOccupancyMDP(
        std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
    {   

        typename oState::jhistory_type hist;
        if (hist_length > 0)
        {
            hist = std::make_shared<typename oState::jhistory_type::element_type>(
                this->dpomdp_->getNumAgents() - 1, hist_length);
        }
        else
        {
            hist = std::make_shared<typename oState::jhistory_type::element_type>(this->dpomdp_->getNumAgents() - 1);
        }

        this->ihistory_ = hist;

        for (typename oState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                Pair<typename oState::state_type, typename oState::jhistory_type> p_x_h(s, hist);
                this->istate_[p_x_h] = this->dpomdp_->getStartDistrib().probabilities()[s];
            }
        }
        this->cstate_ = this->istate_;
    }

    template <typename oState, typename oAction>
    PrivateOccupancyMDP<oState, oAction>::PrivateOccupancyMDP(std::string underlying_dpomdp, number hist_length) : PrivateOccupancyMDP(
        std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length
    )
    {
    }

    template <typename oState, typename oAction>
    oState &PrivateOccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    oState PrivateOccupancyMDP<oState, oAction>::reset()
    {
        this->chistory_ = this->ihistory_;
        this->cstate_ = this->istate_;
        this->dpomdp_->reset();
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    std::tuple<oState, std::vector<double>, bool> PrivateOccupancyMDP<oState, oAction>::step(oAction oaction)
    {   
        
        std::vector<typename oAction::first_type::value_type::output_type> jaction;
        for (number i = 0; i < oaction.first.size(); i++)
        {
            auto p_ihist = this->chistory_->getIndividualHistory(i);
            auto idr = oaction.first.at(i);
            jaction.push_back(idr(p_ihist));

        }
        jaction.push_back(oaction.second);

        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);

        next_obs.pop_back();
        this->chistory_ = this->chistory_->expand(next_obs);
        this->cstate_ = this->nextState(this->cstate_, oaction);

        return std::make_tuple(this->cstate_, rewards, done);
    }

    template <typename oState, typename oAction>
    bool PrivateOccupancyMDP<oState, oAction>::isSerialized() const
    {
        return false;
    }

    template <typename oState, typename oAction>
    DiscreteDecPOMDP *PrivateOccupancyMDP<oState, oAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename oState, typename oAction>
    oState PrivateOccupancyMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> PrivateOccupancyMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {
        auto vect_i_hist = ostate.getAllIndividualHistories();
        std::vector<std::vector<typename oAction::first_type::value_type>> vect_idr = {};
        for (number ag_id = 0; ag_id < this->dpomdp_->getNumAgents() - 1; ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            std::vector<typename oState::jhistory_type::element_type::ihistory_type> v_inputs(
                vect_i_hist[ag_id].begin(), 
                vect_i_hist[ag_id].end()
            );
            FunctionSpace<typename oAction::first_type::value_type> f_indiv_dr_space(
                v_inputs, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll()
            );
            vect_idr.push_back(f_indiv_dr_space.getAll());
        }
        std::vector<oAction> v_oaction;
        // For all joint decision rules of other agents (except N)
        for (const auto&jdr : MultiDiscreteSpace<typename oAction::first_type::output_type>(vect_idr).getAll()){
            // For all actions of player N
            for (const auto & action_n: this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll()){
                    v_oaction.push_back(std::make_pair(jdr, action_n));
            }
        }  
        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<oAction>>(v_oaction);
    }

    template <typename oState, typename oAction>
    oState PrivateOccupancyMDP<oState, oAction>::nextState(
        const oState &ostate, const oAction &oaction, number, HSVI<oState, oAction> *) const
    {
        oState new_ostate;
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
                    auto zn = z.back();
                    z.pop_back();
                    Pair<typename oState::state_type, typename oState::jhistory_type> new_index(y, o->expand(z));
                    std::vector<typename oAction::first_type::value_type::output_type> jaction;
                    for (number i = 0; i < oaction.first.size(); i++)
                    {
                        auto p_ihist = o->getIndividualHistory(i);
                        auto idr = oaction.first.at(i);
                        jaction.push_back(idr(p_ihist));
                    }
                    jaction.push_back(oaction.second);
                    z.push_back(zn);
                    double proba = p_x_o.second * this->dpomdp_->getObsDynamics()->getDynamics(
                        x, 
                        this->dpomdp_->getActionSpace()->joint2single(jaction), 
                        this->dpomdp_->getObsSpace()->joint2single(z), 
                        y
                    );
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
    double PrivateOccupancyMDP<oState, oAction>::getReward(const oState &ostate, const oAction &oaction) const
    {
        double r = 0;
        for (auto &p_x_o : ostate)
        {
            auto state = p_x_o.first.first;
            auto jhistory = p_x_o.first.second;
            std::vector<typename oAction::first_type::value_type::output_type> jaction;
            Joint<typename oAction::first_type::value_type::output_type> actions;
            actions.push_back(oaction.second);
            for (number i = oaction.first.size() - 1; i > 1; i--)
            {
                auto idr = oaction.first.at(i);
                actions.push_back(idr(jhistory->getIndividualHistory(i)));
            }
            for (number i = 0; i < oaction.first.size() + 1; i++)
            {
                jaction.push_back(actions[i]);
            }
            r += p_x_o.second * this->dpomdp_->getReward()->getReward(state, this->dpomdp_->getActionSpace()->joint2single(jaction));
        }
        return r;
    }

    template <typename oState, typename oAction>
    double PrivateOccupancyMDP<oState, oAction>::getExpectedNextValue(
        ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, number t
    ) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteMDP> PrivateOccupancyMDP<oState, oAction>::toMDP()
    {
        return this->dpomdp_->toMDP();
    }

    template <typename oState, typename oAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> PrivateOccupancyMDP<oState, oAction>::toBeliefMDP()
    {
        return this->dpomdp_->toBeliefMDP();
    }

} // namespace sdm
