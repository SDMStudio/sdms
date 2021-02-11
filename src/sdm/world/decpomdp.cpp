
#include <sdm/world/decpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    DecPOMDP::DecPOMDP()
    {
    }

    DecPOMDP::DecPOMDP(std::string &filename)
    {
        *this = sdm::parser::parse_file(filename.c_str());
    }

    DecPOMDP::DecPOMDP(DiscreteSpace<number> &state_sp, DiscreteSpace<number> &agent_sp) : POSG(state_sp, agent_sp)
    {
    }

    DecPOMDP::DecPOMDP(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp, MultiDiscreteSpace<number> obs_sp,
                       StateDynamics s_dyn, ObservationDynamics o_dyn, Reward rew, Vector start_distrib)
        : POSG(state_sp, agent_sp, action_sp, obs_sp, s_dyn, o_dyn, {rew}, start_distrib)
    {
    }

    DecPOMDP::DecPOMDP(POSG &posg) : POSG(posg) {}

    Reward &DecPOMDP::getReward()
    {
        return this->rew_[0];
    }

    double DecPOMDP::getReward(number state, number jaction, number ag_id)
    {
        return this->getReward(state, jaction);
    }

    double DecPOMDP::getReward(number state, std::vector<number> jaction, number ag_id)
    {
        return this->getReward(state, jaction);
    }

    double DecPOMDP::getReward(number state, number jaction)
    {
        return this->rew_[0].getReward(state, jaction);
    }

    double DecPOMDP::getReward(number state, std::vector<number> jaction)
    {
        return this->getReward(state, this->action_space_.joint2single(jaction));
    }

    double DecPOMDP::getCost(number state, number jaction)
    {
        return std::abs((this->getReward().getMinReward() - this->getReward(state, jaction)) / (this->getReward().getMaxReward() - this->getReward().getMinReward()));
    }

    double DecPOMDP::getCost(number state, std::vector<number> jaction)
    {
        return this->getCost(state, this->getActionSpace().joint2single(jaction));
    }

} // namespace sdm