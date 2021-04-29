#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/space/function_space.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP()
    {
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp) : serialized_mpomdp_(std::make_shared<SerializedMPOMDP<TState,TAction>>(underlying_dpomdp))
    {
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length) : serialized_mpomdp_(std::make_shared<SerializedMPOMDP<TState,TAction>>(underlying_dpomdp))
    {

        typename TState::jhistory_type jhist;
        if (hist_length > 0)
        {
            jhist = std::make_shared<typename TState::jhistory_type::element_type>(this->serialized_mpomdp_->getNumAgents(), hist_length);
        }
        else
        {
            jhist = std::make_shared<typename TState::jhistory_type::element_type>(this->serialized_mpomdp_->getNumAgents());
        }

        for (const auto s : this->serialized_mpomdp_->getSerializedStateSpaceAt(0)->getAll())
        {
            auto x = s.getState();
            if (this->serialized_mpomdp_->getStartDistrib().probabilities()[x] > 0) 
            {
                Pair<typename TState::state_type, typename TState::jhistory_type> p_s_o(s, jhist);
                this->istate_[p_s_o] = this->serialized_mpomdp_->getStartDistrib().probabilities()[x];
            }
        }
        //this->cstate_ = this->istate_;
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp), hist_length)
    {
    }

    template <typename TState, typename TAction>
    SerializedOccupancyMDP<TState, TAction>::SerializedOccupancyMDP(std::string underlying_dpomdp) : SerializedOccupancyMDP(std::make_shared<DiscreteDecPOMDP>(underlying_dpomdp))
    {
    }

    // template <typename TState, typename TAction>
    // TState &SerializedOccupancyMDP<TState, TAction>::getState()
    // {
    //     return this->cstate_;
    // }

    template <typename TState, typename TAction>
    SerializedMPOMDP<TState,TAction> *SerializedOccupancyMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->serialized_mpomdp_.get();
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::getInitialState()
    {
        return this->istate_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> SerializedOccupancyMDP<TState, TAction>::getActionSpaceAt(const TState &ostate)
    {
        // Get id of the current agent
        number ag_id = ostate.getCurrentAgentId();

        // Get the individual possible histories for the current agent (as vector)
        auto indiv_hist = ostate.getIndividualHistories(ag_id);

        std::vector<typename TState::jhistory_type::element_type::ihistory_type> v_inputs(indiv_hist.begin(), indiv_hist.end());

        // Generate all individual decision rules for agent 'ag_id' (the current agent)
        FunctionSpace<TAction> f_indiv_dr_space(v_inputs, this->serialized_mpomdp_->getActionSpace()->getSpace(ag_id)->getAll());

        // Now we can return a discrete space of all indiv decision rules
        return std::make_shared<DiscreteSpace<TAction>>(f_indiv_dr_space.getAll());
    }

    template <typename TState, typename TAction>
    TState SerializedOccupancyMDP<TState, TAction>::nextState(const TState &ostate, const TAction &indiv_dr, number, std::shared_ptr<HSVI<TState, TAction>>) const
    {
        number ag_id = ostate.getCurrentAgentId();

        TState new_ostate;

        for (auto &p_s_o : ostate)
        {
            auto pair_s_o = p_s_o.first;
            auto x = ostate.getHiddenState(pair_s_o);
            auto o = pair_s_o.second;
            auto u = ostate.getAction(pair_s_o);

            auto p_ihist = o->getIndividualHistory(ag_id);
            u.push_back(indiv_dr.act(p_ihist));

            if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
            {
                typename TState::state_type s(x,u);
                Pair<typename TState::state_type, typename TState::jhistory_type> s_o(s, o);
                new_ostate[s_o] = p_s_o.second;
            }
            else
            {
                for (const auto y : this->serialized_mpomdp_->getSerializedStateSpaceAt(0)->getAll())
                {    
                    for (auto &z : this->serialized_mpomdp_->getObsSpace()->getAll())
                    {  
                        Pair<typename TState::state_type, typename TState::jhistory_type> new_index(y, o->expand(z));
                        double proba = p_s_o.second * this->serialized_mpomdp_->getObsDynamics(pair_s_o.first, indiv_dr.act(p_ihist), z, y);
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

    template <typename TState, typename TAction>
    double SerializedOccupancyMDP<TState, TAction>::getReward(const TState &ostate, const TAction &indiv_dr) const
    {
        double r = 0;
        number ag_id = ostate.getCurrentAgentId();
        

        if (ag_id != this->serialized_mpomdp_->getNumAgents() - 1)
        {
            return 0;
        }

        for (auto &p_s_o : ostate)
        {
            auto pair_s_o = p_s_o.first;
            auto o = pair_s_o.second;

            r += p_s_o.second * this->serialized_mpomdp_->getReward(pair_s_o.first,indiv_dr.act(o->getIndividualHistory(ag_id)));
        }
        return r;
    }

    template <typename TState, typename TAction>
    double SerializedOccupancyMDP<TState, TAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &ostate, const TAction &oaction, number t) const
    {
        TState ost = this->nextState(ostate, oaction);
        return value_function->getValueAt(ost, t + 1);
    }

    template <typename TState, typename TAction>
    double SerializedOccupancyMDP<TState, TAction>::getDiscount(number t) const
    {
        return this->serialized_mpomdp_->getDiscount(t);
    }


    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDP<>> SerializedOccupancyMDP<TState, TAction>::toMDP()
    {
        return this->serialized_mpomdp_->toMDP();
    }

    template <typename TState, typename TAction>
    bool SerializedOccupancyMDP<TState, TAction>::isSerialized() const
    {
        return this->serialized_mpomdp_->isSerialized();
    }
    
} // namespace sdm
