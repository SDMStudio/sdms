
#include <sdm/world/zsposg.hpp>
#include <exception>

namespace sdm
{

    ZSPOSG::ZSPOSG()
    {
    }

    ZSPOSG::ZSPOSG(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp, MultiDiscreteSpace<number> obs_sp,
                   StateDynamics s_dyn, ObservationDynamics o_dyn, Reward rew, Vector start_distrib)
        : POSG(state_sp, agent_sp, action_sp, obs_sp, s_dyn, o_dyn, {rew}, start_distrib)
    {
        assert(agent_sp.getNumItems() == 2);

        if (this->getNumAgents() != 2)
        {
            throw std::invalid_argument("Wrong agent space. The number of agents must be equal to 2. Got : " + std::to_string(this->getNumAgents()));
        }
        this->rew_.push_back(Reward(this->getNumJActions(), this->getNumStates()));
        for (number s = 0; s < this->getNumStates(); s++)
        {
            for (number a = 0; a < this->getNumJActions(); a++)
            {
                this->rew_[1].setReward(s, a, 0 - this->rew_[0].getReward(s, a));
            }
        }
    }

    ZSPOSG::ZSPOSG(DecPOMDP dec_pomdp) : ZSPOSG(dec_pomdp.getStateSpace(), dec_pomdp.getAgentSpace(), dec_pomdp.getActionSpace(), dec_pomdp.getObsSpace(), dec_pomdp.getStateDynamics(), dec_pomdp.getObsDynamics(), dec_pomdp.getReward(), dec_pomdp.getStartDistrib())
    {
    }

    std::vector<double> ZSPOSG::getRewards(number state, number jaction) 
    {
        std::vector<double> v_r;
        for (number ag = 0; ag < this->getNumAgents(); ++ag)
        {
            v_r.push_back(this->getReward(state, jaction, ag));
        }
        return v_r;
    }

    std::vector<double> ZSPOSG::getRewards(number state, std::vector<number> jaction) 
    {
        return this->getRewards(state, this->action_space_.joint2single(jaction));
    }

    double ZSPOSG::getReward(number state, number jaction, number ag_id) 
    {
        return this->rew_[ag_id].getReward(state, jaction);
    }

    double ZSPOSG::getReward(number state, std::vector<number> jaction, number ag_id) 
    {
        return this->getReward(state, this->action_space_.joint2single(jaction), ag_id);
    }
} // namespace sdm