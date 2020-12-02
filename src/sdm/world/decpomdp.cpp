
#include <sdm/world/decpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    DecPOMDP::DecPOMDP()
    {
    }

    DecPOMDP::DecPOMDP(const std::string &filename)
    {
        *this = sdm::parser::parse_file(filename.c_str());
    }

    DecPOMDP::DecPOMDP(const DiscreteSpace &state_sp, const DiscreteSpace &agent_sp) : POSG(state_sp, agent_sp)
    {
    }

    DecPOMDP::DecPOMDP(const DiscreteSpace &state_sp, const DiscreteSpace &agent_sp, const MultiDiscreteSpace &action_sp, const MultiDiscreteSpace &obs_sp,
                       const StateDynamics &s_dyn, const ObservationDynamics &o_dyn, const Reward &rew, const Vector &start_distrib)
        : POSG(state_sp, agent_sp, action_sp, obs_sp, s_dyn, o_dyn, {rew}, start_distrib)
    {
    }

    DecPOMDP::DecPOMDP(const POSG& posg) : POSG(posg){}


    const Reward &DecPOMDP::getReward() const
    {
        return this->rew_[0];
    }

    double DecPOMDP::getReward(number state, number jaction, number ag_id) const
    {
        return this->getReward(state, jaction);
    }

    double DecPOMDP::getReward(number state, std::vector<number> jaction, number ag_id) const
    {
        return this->getReward(state, jaction);
    }

    double DecPOMDP::getReward(number state, number jaction) const
    {
        return this->rew_[0].getReward(state, jaction);
    }

    double DecPOMDP::getReward(number state, std::vector<number> jaction) const
    {
        return this->getReward(state, this->action_space_.joint2single(jaction));
    }

    double DecPOMDP::getCost(number state, number jaction) const
    {
        return std::abs((this->getReward().getMinReward() - this->getReward(state, jaction)) / (this->getReward().getMaxReward() - this->getReward().getMinReward()));
    }

    double DecPOMDP::getCost(number state, std::vector<number> jaction) const
    {
        return this->getCost(state, this->getActionSpace().joint2single(jaction));
    }

} // namespace sdm