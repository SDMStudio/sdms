
#include <sdm/world/pomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
    POMDP::POMDP() : DecPOMDP()
    {
    }

    POMDP::POMDP(DecPOMDP &decpomdp) : DecPOMDP(decpomdp)
    {
    }

    POMDP::POMDP(std::string &filename) : DecPOMDP(filename)
    {
    }

    POMDP::POMDP(DiscreteSpace<number> state_sp, MultiDiscreteSpace<number> action_sp, MultiDiscreteSpace<number> obs_sp,
                 StateDynamics s_dyn, ObservationDynamics o_dyn, Reward rew, Vector start_distrib) : DecPOMDP(state_sp, DiscreteSpace<number>({0}), action_sp, obs_sp, s_dyn, o_dyn, rew, start_distrib)
    {
    }

} // namespace sdm