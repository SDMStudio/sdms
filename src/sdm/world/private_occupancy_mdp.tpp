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

        typename oState::jjhistory_type jjhist;
        for (number agent = 0; agent < this->dpomdp_->getNumAgents() - 1; agent++){
            if (hist_length > 0)
            {
                jjhist.push_back(std::make_shared<typename oState::jhistory_type::element_type>(
                    this->dpomdp_->getNumAgents() - agent, hist_length
                ));
            }
            else
            {
                jjhist.push_back(std::make_shared<typename oState::jhistory_type::element_type>(
                    this->dpomdp_->getNumAgents() - agent
                ));
            }
        }

        this->ihistory_ = jjhist;

        for (typename oState::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                Pair<typename oState::state_type, typename oState::jjhistory_type> p_x_h(s, jjhist);
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
        Joint<typename oAction::first_type::value_type::output_type> actions;
        actions.push_back(oaction.second);
        for (int agent = this->dpomdp_->getNumAgents() - 2; agent >= 0; agent--)
        {   
            auto p_ihist = this->chistory_.at(agent);
            auto idr = oaction.first.at(agent);
            actions.push_back(idr(std::make_pair(p_ihist, actions)));
        }

        for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++)
        {
            jaction.push_back(actions[agent]);
        }

        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);

        std::vector<Joint<number>> next_obs_vector(this->dpomdp_->getNumAgents());
        // For all agents from 0 to n-1 (1 to n)
        for (number i = 0; i < this->dpomdp_->getNumAgents(); i++){
            // For all agents from 0 to i (1 to i)
            for (number j = 0; j <= i; j++){
                next_obs_vector[j].push_back(next_obs[i]);
            }
        }

        for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++){
            this->chistory_.at(agent) = this->chistory_.at(agent)->expand(next_obs_vector[agent]);
        }

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
        auto vect_ij_hist = ostate.getAllIndividualJointHistories();
        std::vector<std::vector<typename oAction::first_type::value_type>> vect_idr = {};
        for (number ag_id = 0; ag_id < this->dpomdp_->getNumAgents() - 1; ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            std::vector<typename oState::jhistory_type> v_inputs(
                vect_ij_hist[ag_id].begin(), 
                vect_ij_hist[ag_id].end()
            );
            std::vector<Pair<typename oState::jhistory_type, Joint<number>>> v_inputs_;
            for (auto &v_input: v_inputs){
                for (const auto & action_n: this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll()){
                    Joint<typename oAction::first_type::value_type::output_type> actions;
                    actions.push_back(action_n); /////// this only works for N=2 btw
                    v_inputs_.push_back(make_pair(v_input, actions));
                }
            }
            FunctionSpace<typename oAction::first_type::value_type> f_indiv_dr_space(
                v_inputs_, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll()
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
                    std::vector<Joint<number>> z_vector(this->dpomdp_->getNumAgents());
                    // For all agents from 0 to n-1 (1 to n)
                    for (number i = 0; i < this->dpomdp_->getNumAgents(); i++){
                        // For all agents from 0 to i (1 to i)
                        for (number j = 0; j <= i; j++){
                            z_vector[j].push_back(z[i]);
                        }
                    }
                    auto zn = z_vector.back();
                    z_vector.pop_back();
                    
                    auto new_o = o;
                    for (number agent = 0; agent < this->dpomdp_->getNumAgents() - 1; agent++){
                        new_o.at(agent) = new_o.at(agent)->expand(z_vector.at(agent));
                    }

                    Pair<typename oState::state_type, typename oState::jjhistory_type> new_index(y, new_o);
                    std::vector<typename oAction::first_type::value_type::output_type> jaction;
                    Joint<typename oAction::first_type::value_type::output_type> actions;
                    actions.push_back(oaction.second);

                    for (int agent = this->dpomdp_->getNumAgents() - 2; agent >= 0; agent--)
                    {
                        auto p_ihist = o.at(agent);
                        auto idr = oaction.first.at(agent);
                        actions.push_back(idr(std::make_pair(p_ihist, actions)));
                    }

                    for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++)
                    {
                        jaction.push_back(actions[agent]);
                    }

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
            auto jjhistory = p_x_o.first.second;
            std::vector<typename oAction::first_type::value_type::output_type> jaction;
            Joint<typename oAction::first_type::value_type::output_type> actions;
            actions.push_back(oaction.second);
            for (int agent = this->dpomdp_->getNumAgents() - 2; agent >= 0; agent--)
            {
                auto idr = oaction.first.at(agent);
                actions.push_back(idr(std::make_pair(jjhistory.at(agent), actions)));
            }
            for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++)
            {
                jaction.push_back(actions[agent]);
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
