#pragma once

#include <sdm/types.hpp>
#include <sdm/world/mdp.hpp>
#include <sdm/world/base/base_pomdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes.
     *
     */
    class POMDP : public BasePOMDP,
                  public SolvableByHSVI,
                  public std::enable_shared_from_this<POMDP>
    {
    public:
        POMDP();
        POMDP(std::shared_ptr<MDP> mdp, std::shared_ptr<ObsDynamics> obs_dyn);
        POMDP(std::shared_ptr<DiscreteSpace<number>>, std::shared_ptr<DiscreteSpace<number>>, std::shared_ptr<StateDynamics>, std::shared_ptr<Reward>, std::discrete_distribution<number>, number = 0, double = 0.9, Criterion = Criterion::REW_MAX);
        POMDP(std::string &filename);

        std::shared_ptr<POMDP> getptr();

        std::shared_ptr<MDP> toMDP();

        std::shared_ptr<BeliefMDP> toBeliefMDP();

    protected:
        std::shared_ptr<MDP> mdp = nullptr;
    };
} // namespace sdm