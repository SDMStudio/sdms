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

        // std::cout << "sss" << std::endl;
        typename oState::second_type jjhist_full_reversed;
        std::vector<typename oState::first_type::history_type> hists_reversed;
        for (int agent = this->dpomdp_->getNumAgents() - 1; agent >= 0; agent--){
            // std::cout << "agent " << agent << std::endl;
            if (hist_length > 0)
            {
                hists_reversed.push_back(std::make_shared<typename oState::first_type::history_type::element_type>(hist_length));
            } else {
                hists_reversed.push_back(std::make_shared<typename oState::first_type::history_type::element_type>());
            }
            std::vector<typename oState::first_type::history_type> hists;
            for (int i = hists_reversed.size() - 1; i >= 0; i--){
                // std::cout << "i " << i << std::endl;
                hists.push_back(hists_reversed[i]);
            }
            typename oState::first_type::jhistory_type jhist = std::make_shared<typename oState::first_type::jhistory_type::element_type>(
                hists
            );
            // std::cout << "jhist " << jhist << std::endl;
            // std::cout << "jhist->getDepth() " << jhist->getDepth() << std::endl;
            // std::cout << "jhist->getData() " << jhist->getData() << std::endl;
            // std::cout << "jhist->isOrigin() " << jhist->isOrigin() << std::endl;
            jjhist_full_reversed.push_back(jhist);
        }
        // std::cout << "sss" << std::endl;

        typename oState::first_type::jjhistory_type jjhist;
        typename oState::second_type jjhist_full;
        for (int j = jjhist_full_reversed.size() - 1; j >= 0; j--){
            // std::cout << "j " << j << std::endl;
            if (j > 0){
                // std::cout << "yoooo " << std::endl;
                // std::cout << "jjhist_full_reversed[j] " << jjhist_full_reversed[j] << std::endl;
                // std::cout << "jjhist_full_reversed[j]->getDepth() " << jjhist_full_reversed[j]->getDepth() << std::endl;
                // std::cout << "jjhist_full_reversed[j]->getData() " << jjhist_full_reversed[j]->getData() << std::endl;
                // std::cout << "jjhist_full_reversed[j]->isOrigin() " << jjhist_full_reversed[j]->isOrigin() << std::endl;
                jjhist_full.push_back(jjhist_full_reversed[j]);
                jjhist.push_back(jjhist_full_reversed[j]);
            } else {
                // std::cout << "breeeh " << std::endl;
                // std::cout << "jjhist_full_reversed[j] " << jjhist_full_reversed[j] << std::endl;
                // std::cout << "jjhist_full_reversed[j]->getDepth() " << jjhist_full_reversed[j]->getDepth() << std::endl;
                // std::cout << "jjhist_full_reversed[j]->getData() " << jjhist_full_reversed[j]->getData() << std::endl;
                // std::cout << "jjhist_full_reversed[j]->isOrigin() " << jjhist_full_reversed[j]->isOrigin() << std::endl;
                jjhist_full.push_back(jjhist_full_reversed[j]);
            }
        }
        // std::cout << "sss" << std::endl;

        // typename oState::first_type::jjhistory_type jjhist;
        // for (number agent = 0; agent < this->dpomdp_->getNumAgents() - 1; agent++){
        //     if (hist_length > 0)
        //     {
        //         jjhist.push_back(std::make_shared<typename oState::first_type::jhistory_type::element_type>(
        //             this->dpomdp_->getNumAgents() - agent, hist_length
        //         ));
        //     }
        //     else
        //     {
        //         jjhist.push_back(std::make_shared<typename oState::first_type::jhistory_type::element_type>(
        //             this->dpomdp_->getNumAgents() - agent
        //         ));
        //     }
        // }

        for (typename oState::first_type::state_type s : this->dpomdp_->getStateSpace()->getAll())
        {
            if (this->dpomdp_->getStartDistrib().probabilities()[s] > 0)
            {
                Pair<typename oState::first_type::state_type, typename oState::first_type::jjhistory_type> p_x_h(s, jjhist);
                this->istate_.first[p_x_h] = this->dpomdp_->getStartDistrib().probabilities()[s];
            }
        }

        // typename oState::second_type jjhist_full;
        // for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++){
        //     if (agent < this->dpomdp_->getNumAgents() - 1)
        //     {
        //         jjhist_full.push_back(jjhist[agent]);
        //     } else {
        //         if (hist_length > 0)
        //         {
        //             jjhist_full.push_back(std::make_shared<typename oState::first_type::jhistory_type::element_type>(
        //                 this->dpomdp_->getNumAgents() - agent, hist_length
        //             ));
        //         }
        //         else
        //         {
        //             jjhist_full.push_back(std::make_shared<typename oState::first_type::jhistory_type::element_type>(
        //                 this->dpomdp_->getNumAgents() - agent
        //             ));
        //         }
        //     }
            
        // }

        this->istate_.second = jjhist_full;

        this->cstate_ = this->istate_;
        // JointHistoryTree_p<number> o2 = this->cstate_.second.at(1);
        // std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
        // std::cout << "o2->getIndividualHistory(0) " << o2->getIndividualHistory(0) << std::endl;
        // std::cout << "o2->getIndividualHistory(0)->getDepth() " << o2->getIndividualHistory(0)->getDepth() << std::endl;
        // std::cout << "o2->getIndividualHistory(0)->getData() " << o2->getIndividualHistory(0)->getData() << std::endl;
        // std::cout << "*o2->getIndividualHistory(0).get() " << std::endl << *o2->getIndividualHistory(0).get() << std::endl;
        // std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
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
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;
        std::vector<typename oAction::first_type::value_type::output_type> jaction;
        Joint<typename oAction::first_type::value_type::output_type> actions;
        actions.push_back(oaction.second);
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;

        for (int agent = this->dpomdp_->getNumAgents() - 2; agent >= 0; agent--)
        {   
            // std::cout << "agent " << agent << std::endl;
            auto p_ihist = this->cstate_.second.at(agent);
            auto idr = oaction.first.at(agent);
            std::cout << "p_ihist " << p_ihist << std::endl;
            std::cout << "p_ihist->getDepth() " << p_ihist->getDepth() << std::endl;
            std::cout << "p_ihist->getData() " << p_ihist->getData() << std::endl;
            std::cout << "p_ihist->isOrigin() " << p_ihist->isOrigin() << std::endl;
            std::cout << "idr " << idr << std::endl;
            actions.push_back(idr(std::make_pair(p_ihist, actions)));
        }
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;

        for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++)
        {
            jaction.push_back(actions[agent]);
        }
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;

        auto [next_obs, rewards, done] = this->dpomdp_->step(jaction);
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;

        std::vector<Joint<number>> next_obs_vector(this->dpomdp_->getNumAgents());
        // For all agents from 0 to n-1 (1 to n)
        for (number i = 0; i < this->dpomdp_->getNumAgents(); i++){
            // For all agents from 0 to i (1 to i)
            for (number j = 0; j <= i; j++){
                next_obs_vector[j].push_back(next_obs[i]);
            }
        }
        this->cstate_ = this->nextState(this->cstate_, oaction);
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;

        for (number agent = 0; agent < this->dpomdp_->getNumAgents(); agent++){
            this->cstate_.second.at(agent) = this->cstate_.second.at(agent)->expand(next_obs_vector[agent]);
        }
        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;

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
        std::cout << "bbbbbbbbbbbbbbbbbbbb" << std::endl;
        auto vect_ij_hist = ostate.first.getAllIndividualJointHistories();
        std::vector<std::vector<typename oAction::first_type::value_type>> vect_idr = {};
        std::cout << "bbbbbbbbbbbbbbbbbbbb" << std::endl;
        for (number ag_id = 0; ag_id < this->dpomdp_->getNumAgents() - 1; ag_id++)
        {
            // Generate all individual decision rules for agent 'ag_id'
            std::vector<typename oState::first_type::jhistory_type> v_inputs(
                vect_ij_hist[ag_id].begin(), 
                vect_ij_hist[ag_id].end()
            );
            std::vector<Pair<typename oState::first_type::jhistory_type, Joint<number>>> v_inputs_;
            for (auto &v_input: v_inputs){
                for (const auto & u2: this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll()){
                    std::cout << "u2 " << u2 << std::endl;
                    Joint<typename oAction::first_type::value_type::output_type> actions;
                    actions.push_back(u2); /////// this only works for N=2 btw
                    v_inputs_.push_back(make_pair(v_input, actions));
                    // std::cout << "v_input " << v_input << std::endl;
                    // std::cout << "v_input->getDepth() " << v_input->getDepth() << std::endl;
                    // std::cout << "v_input->getData() " << v_input->getData() << std::endl;
                    // std::cout << "v_input->isOrigin() " << v_input->isOrigin() << std::endl;
                }
            }
            FunctionSpace<typename oAction::first_type::value_type> f_indiv_dr_space(
                v_inputs_, this->dpomdp_->getActionSpace()->getSpace(ag_id)->getAll()
            );
            vect_idr.push_back(f_indiv_dr_space.getAll());
        }
        std::cout << "bbbbbbbbbbbbbbbbbbbb" << std::endl;
        std::vector<oAction> v_oaction;
        std::cout << "b" << std::endl;
        // For all joint decision rules of other agents (except N)
        for (const auto&jdr : MultiDiscreteSpace<typename oAction::first_type::output_type>(vect_idr).getAll()){
            std::cout << "jdr " << jdr << std::endl;
            // For all actions of player N
            for (const auto & action_n: this->dpomdp_->getActionSpace()->getSpace(this->dpomdp_->getNumAgents() - 1)->getAll()){
                    std::cout << "action_n " << action_n << std::endl;
                    v_oaction.push_back(std::make_pair(jdr, action_n));
            }
        }  
        std::cout << "bbbbbbbbbbbbbbbbbbbb" << std::endl;
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
