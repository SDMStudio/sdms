/**
 * @file discrete_decpomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/po_decision_process.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

// #include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Decentralized Partially Observable Markov Decision Processes. 
     * This class is central in SDMS since the actual parser can only parse files that are conform to the .dpomdp format (see [masplan page](http://masplan.org/problem_domains) ).
     *   
     * 
     */
    class DiscreteDecPOMDP : public PartiallyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, ObservationDynamics, Reward, std::discrete_distribution<number>>,
                             public std::enable_shared_from_this<DiscreteDecPOMDP>

    {
    public:
        // using occupancy_mdp = OccupancyMDP<OccupancyState<number, JointHistoryTree_p<number>>, JointDeterministicDecisionRule<HistoryTree_p<number>, number>>;

        DiscreteDecPOMDP();
        DiscreteDecPOMDP(std::string &);
        DiscreteDecPOMDP(DiscreteDecPOMDP &);
        DiscreteDecPOMDP(std::shared_ptr<DiscreteSpace<number>> , std::shared_ptr<MultiDiscreteSpace<number>> , std::shared_ptr<MultiDiscreteSpace<number>> , std::shared_ptr<StateDynamics> , std::shared_ptr<ObservationDynamics> , std::shared_ptr<Reward>, std::discrete_distribution<number> , number  = 0, double  = 0.9, Criterion  = Criterion::REW_MAX);

        std::shared_ptr<DiscreteDecPOMDP> getptr();

        /**
         * @brief Get the corresponding Markov Decision Process. The induced MDP is the DecPOMP problem transformed as it was a single agent and fully observable problem. The joint actions are represented as single actions.
         * 
         * @return The corresponding MDP.
         */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Partially Observable Markov Decision Process. The induced POMDP is the DecPOMP problem transformed as it was a single agent problem. The joint actions and observations are represented as single actions.
         * 
         * @return The corresponding POMDP.
         */
        std::shared_ptr<DiscretePOMDP> toPOMDP();

        /**
         * @brief Get the corresponding Partially Observable Markov Decision Process. The induced POMDP is the DecPOMP problem transformed as it was a single agent problem. The joint actions and observations are represented as single actions.
         * 
         * @return The corresponding POMDP.
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();

        /**
         * @brief Get the corresponding Multi-agent Markov Decision Process. It corresponds to the relaxation of the original DecPOMP assuming that all agents can observation the entire state of the environment. 
         * 
         * @return a MMDP.
         */
        std::shared_ptr<DiscreteMMDP> toMMDP();


        // std::shared_ptr<SerializedOccupancyMDP> toSerializedOccupancyMDP();

        /**
         * @brief Encodes DiscreteDecPOMDP class into a string (standard .posg or .dpomdp or .zsposg format).
         * 
         * @return the process as XML
         */
        std::string toStdFormat();

        /**
         * @brief Encodes DiscreteDecPOMDP class into a string (XML format).
         * 
         * @return the process as XML
         */
        std::string toXML();

        /**
         * @brief Encodes DiscreteDecPOMDP class into a string (JSON format).
         * 
         */
        std::string toJSON();

        /**
         * @brief Save problem in file with given format (.xml, .json or .{dpomdp, posg, zsposg}).
         * 
         * @param filename the file name
         */
        void generateFile(std::string);

        friend std::ostream &operator<<(std::ostream &os, DiscreteDecPOMDP &model)
        {
            os << model.toXML();
            return os;
        }
    };
} // namespace sdm
