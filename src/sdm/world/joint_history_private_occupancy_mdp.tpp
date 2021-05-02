#include <sdm/world/joint_history_private_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>
// #include <typeinfo>
// #include <cxxabi.h>

namespace sdm
{

    template <typename oState, typename oAction>
    JointHistoryPrivateOccupancyMDP<oState, oAction>::JointHistoryPrivateOccupancyMDP()
    {
    }

    template <typename oState, typename oAction>
    JointHistoryPrivateOccupancyMDP<oState, oAction>::JointHistoryPrivateOccupancyMDP(
        std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : dpomdp_(underlying_dpomdp)
    {   

        typename oState::second_type jjhist_full_reversed;
        std::vector<typename oState::first_type::history_type> hists_reversed;
        for (int agent = this->dpomdp_->getNumAgents() - 1; agent >= 0; agent--){
            if (hist_length > 0)
            {
                hists_reversed.push_back(std::make_shared<typename oState::first_type::history_type::element_type>(hist_length));
            } else {
                hists_reversed.push_back(std::make_shared<typename oState::first_type::history_type::element_type>());
            }
            std::vector<typename oState::first_type::history_type> hists;
            for (int i = hists_reversed.size() - 1; i >= 0; i--){
                hists.push_back(hists_reversed[i]);
            }
            typename oState::first_type::jhistory_type jhist = std::make_shared<typename oState::first_type::jhistory_type::element_type>(
                hists
            );
            jjhist_full_reversed.push_back(jhist);
        }

        typename oState::first_type::jjhistory_type jjhist;
        typename oState::second_type jjhist_full;
        for (int j = jjhist_full_reversed.size() - 1; j >= 0; j--){
            if (j > 0){
                jjhist_full.push_back(jjhist_full_reversed[j]);
                jjhist.push_back(jjhist_full_reversed[j]);
            } else {
                jjhist_full.push_back(jjhist_full_reversed[j]);
            }
        }

        for (typename oState::first_type::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                Pair<typename oState::first_type::state_type, typename oState::first_type::jjhistory_type> p_x_h(s, jjhist);
                this->istate_.first[p_x_h] = this->dpomdp_->getStartDistrib().probabilities()[s];
            }
        }

        this->istate_.second = jjhist_full;

        this->cstate_ = this->istate_;
    }

    template <typename oState, typename oAction>
    JointHistoryPrivateOccupancyMDP<oState, oAction>::JointHistoryPrivateOccupancyMDP(std::string underlying_dpomdp, number hist_length) : JointHistoryPrivateOccupancyMDP(
        std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length
    )
    {
    }

    template <typename oState, typename oAction>
    oState &JointHistoryPrivateOccupancyMDP<oState, oAction>::getState()
    {
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    oState JointHistoryPrivateOccupancyMDP<oState, oAction>::reset()
    {
        this->cstate_ = this->istate_;
        this->dpomdp_->reset();
        return this->cstate_;
    }

    template <typename oState, typename oAction>
    std::tuple<oState, std::vector<double>, bool> JointHistoryPrivateOccupancyMDP<oState, oAction>::step(oAction oaction)
    {           
        std::vector<typename oAction::first_type::value_type::output_type> jaction;
        typename oAction::first_type::value_type::input_type::second_type actions;
        actions.push_back(oaction.second);
        for (int agent = this->dpomdp_->getNumAgents() - 2; agent >= 0; agent--)
        {   
            auto p_ihist = this->cstate_.second.at(agent);
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
        this->cstate_ = this->nextState(this->cstate_, oaction);
        for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++){
            this->cstate_.second.at(agent) = this->cstate_.second.at(agent)->expand(next_obs_vector[agent]);
        }
        return std::make_tuple(this->cstate_, rewards, done);
    }

    template <typename oState, typename oAction>
    bool JointHistoryPrivateOccupancyMDP<oState, oAction>::isSerialized() const
    {
        return false;
    }

    template <typename oState, typename oAction>
    DiscreteDecPOMDP *JointHistoryPrivateOccupancyMDP<oState, oAction>::getUnderlyingProblem()
    {
        return this->dpomdp_.get();
    }

    template <typename oState, typename oAction>
    oState JointHistoryPrivateOccupancyMDP<oState, oAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> JointHistoryPrivateOccupancyMDP<oState, oAction>::getActionSpaceAt(const oState &ostate)
    {   
        auto vect_ij_hist = ostate.first.getAllIndividualJointHistories();
        std::vector<std::vector<typename oAction::first_type::value_type>> vect_jdr = {};
        for (number agent = 0; agent < this->dpomdp_->getNumAgents() - 1; agent++)
        {
            // Generate all individual decision rules for agent 'agent'
            std::vector<typename oState::first_type::jhistory_type> v_ij_hists(
                vect_ij_hist[agent].begin(), 
                vect_ij_hist[agent].end()
            );
            std::vector<Pair<typename oState::first_type::jhistory_type, 
                             typename oAction::first_type::value_type::input_type::second_type
                            >> v_inputs;

            int n_actions = 1;
            std::vector<std::vector<typename oAction::first_type::value_type::input_type::second_type>> vec_vec_jactions(
                this->dpomdp_->getNumAgents() - 1 - agent
            );
            for (int agent_ = this->dpomdp_->getNumAgents() - 1; agent_ > agent; agent_--){
                number num_actions_agent_ = 0;
                for (const auto & action_agent_: this->dpomdp_->getActionSpace()->getSpace(agent_)->getAll()){
                    num_actions_agent_++;
                }
                n_actions = n_actions * num_actions_agent_;
                std::vector<typename oAction::first_type::value_type::input_type::second_type> vec_jactions(n_actions);
                vec_vec_jactions.push_back(vec_jactions);
            }
            // First we need to do it for agent_ N.
            for (const auto & u_n: this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll()){
                typename oAction::first_type::value_type::input_type::second_type jaction;
                jaction.push_back(u_n);
                vec_vec_jactions[0].push_back(jaction);
            }
            int i = 0;
            // Then other agent_s, starting with agent_ N-1 until the last agent_ before agent.
            for (int agent_ = this->dpomdp_->getNumAgents() - 2; agent_ > agent; agent_--){
                i++;
                std::vector<typename oAction::first_type::value_type::input_type::second_type> vec_jactions;
                for(const auto & jaction_prev: vec_vec_jactions[i - 1]){
                    for(const auto & u_agent_: this->dpomdp_->getActionSpace()->getSpace(agent_)->getAll()){
                        typename oAction::first_type::value_type::input_type::second_type jaction = jaction_prev;
                        jaction.push_back(u_agent_);
                        vec_vec_jactions[i].push_back(jaction);
                    }
                }
            }
            
            for (auto &ij_hist: v_ij_hists){
                for(const auto & jaction: vec_vec_jactions[i]){
                    v_inputs.push_back(make_pair(ij_hist, jaction));
                }
            }

            FunctionSpace<typename oAction::first_type::value_type> f_indiv_dr_space(
                v_inputs, this->dpomdp_->getActionSpace()->getSpace(agent)->getAll()
            );
            vect_jdr.push_back(f_indiv_dr_space.getAll());
        }
        std::vector<oAction> v_oaction;
        // For all joint decision rules of other agents (except N)
        for (const auto&jdr : MultiDiscreteSpace<typename oAction::first_type::output_type>(vect_jdr).getAll()){
            // For all actions of agent N
            for (const auto & u_n: this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll()){
                    v_oaction.push_back(std::make_pair(jdr, u_n));
            }
        }  
        // Now we can return a discrete space of all joint decision rules
        return std::make_shared<DiscreteSpace<oAction>>(v_oaction);
    }


    template <typename oState, typename oAction>
    oState JointHistoryPrivateOccupancyMDP<oState, oAction>::nextState(
        const oState &ostate, const oAction &oaction, number, HSVI<oState, oAction> *) const
    {
        oState new_ostate;
        for (auto &p_x_o : ostate.first)
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

                    Pair<typename oState::first_type::state_type, typename oState::first_type::jjhistory_type> new_index(y, new_o);
                    std::vector<typename oAction::first_type::value_type::input_type::second_type::value_type> jaction;
                    typename oAction::first_type::value_type::input_type::second_type actions;
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
                        new_ostate.first[new_index] = new_ostate.first.at(new_index) + proba;
                    }
                }
            }
            // }
        }
        new_ostate.second = ostate.second;
        return new_ostate;
    }

    template <typename oState, typename oAction>
    double JointHistoryPrivateOccupancyMDP<oState, oAction>::getReward(const oState &ostate, const oAction &oaction) const
    {
        double r = 0;
        for (auto &p_x_o : ostate.first)
        {
            auto state = p_x_o.first.first;
            auto jjhistory = p_x_o.first.second;
            std::vector<typename oAction::first_type::value_type::input_type::second_type::value_type> jaction;
            typename oAction::first_type::value_type::input_type::second_type actions;
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
    double JointHistoryPrivateOccupancyMDP<oState, oAction>::getExpectedNextValue(
        ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, number t
    ) const
    {
        oState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteMDP> JointHistoryPrivateOccupancyMDP<oState, oAction>::toMDP()
    {
        return this->dpomdp_->toMDP();
    }

    template <typename oState, typename oAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> JointHistoryPrivateOccupancyMDP<oState, oAction>::toBeliefMDP()
    {
        return this->dpomdp_->toBeliefMDP();
    }

} // namespace sdm
