/**
 * @file hierarchical_mpomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file to transforme a MPOMDP in a hierarchical MPOMDP
 * @version 1.0
 * @date 27/07/2021
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sdm/types.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/world/transformed_mpomdp.hpp>

namespace sdm
{
    /**
     * @brief The Hierarchical MPOMDP is a transformation of a standard MPOMDP assuming there exists a hierarchy among agents.
     *
     * More precisely, each agent receive observations of its surbordinates in addition to its own observation.
     *
     * Example - tiger problem :
     *
     * (hear-left, hear-left) --> ((hear-left), (hear-left, hear-left))
     * (hear-left, hear-right) --> ((hear-left), (hear-left, hear-right))
     * (hear-right, hear-left) --> ((hear-right), (hear-right, hear-left))
     * (hear-right, hear-right) --> ((hear-right), (hear-right, hear-right))
     *
     */
    class HierarchicalMPOMDP : public TransformedMPOMDP
    {
    public:
        HierarchicalMPOMDP(Config config);
        HierarchicalMPOMDP(const std::shared_ptr<MPOMDPInterface> &mpomdp, Config = {});

        /**
         * @brief Get the reachable next states
         *
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        std::shared_ptr<Space> getObservationSpace(number t) const;

        /**
         * @brief Get ths observation space of agent i at timestep t.
         *
         * @param agent_id the identifier of the agent
         * @param t the timestep
         * @return the observation space
         */
        std::shared_ptr<Space> getObservationSpace(number agent_id, number t) const;

        /**
         * @brief Get the Reachablel Observations object
         *
         * @param state
         * @param action
         * @param next_state
         * @param t
         * @return std::set<std::shared_ptr<Observation>>
         */
        std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

        /**
         * @brief Get the Obs Probability object
         *
         * @param state
         * @param action
         * @param next_state
         * @param observation
         * @param t
         * @return double
         */
        double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

        /**
         * @brief Get ths action space at timestep t.
         *
         * @param t the timestep
         * @return the action space
         */
        // virtual std::shared_ptr<Space> getSubordinateActionSpace(number agent_id) const = 0;

        std::shared_ptr<Observation> getClassicObservation(const std::shared_ptr<Observation> &hierarchical_joint_observation) const;
        std::shared_ptr<Observation> getHierarchicalObservation(const std::shared_ptr<Observation> &classic_joint_observation) const;

        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action, bool increment_timestep);

    protected:
        /** @brief The transformed joint observation space */
        std::shared_ptr<DiscreteSpace> joint_observation_space_;

        /** @brief Keep transformed individual observation space for each agent */
        Joint<std::shared_ptr<DiscreteSpace>> indiv_observation_spaces_;

        using obs_bimap = boost::bimaps::bimap<std::shared_ptr<Observation>, std::shared_ptr<Observation>>;
        using obs_bimap_value = obs_bimap::value_type;

        /** @brief The mapping from joint observation to hierarchical observation **/
        obs_bimap bimap_classic_to_hierarchical_obs;

        /**
         * @brief This function will transform the observation space of the MPOMDP as if it was a hierarchical problem.
         */
        void setupObservationSpace();
    };

} // namespace sdm