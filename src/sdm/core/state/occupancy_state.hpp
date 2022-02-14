#pragma once
#include <sdm/types.hpp>
#include <sdm/macros.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    class PrivateOccupancyState;

    /**
     * @brief An occupancy state refers to the complete knowledge the central planner have access to take decisions.
     *
     * Occupancy states are firstly defined by Dibangoye, Amato, Buffet and Charpillet
     * in [Optimally Solving Dec-POMDPs as Continuous-State MDPs](https://hal.inria.fr/hal-01279444/document).
     * An occupancy state is defined as a posterior distribution over states and histories, given a complete information state
     * (i.e. \$\\xi_t (x_{t}, o_{t} ) = p(x_{t}, o_t \\mid i_{t})\$ ) .
     *
     */
    class OccupancyState : public Belief,
                           public OccupancyStateInterface
    {
    public:
        static double PRECISION;

        OccupancyState();
        OccupancyState(number num_agents, number h);
        OccupancyState(number num_agents, number h, StateType stateType);
        OccupancyState(const OccupancyState &copy);
        ~OccupancyState();

        virtual std::shared_ptr<OccupancyState> make(number h);
        virtual std::shared_ptr<OccupancyState> copy();

        virtual size_t hash(double precision = PRECISION) const;
        virtual bool operator==(const OccupancyState &other) const;
        virtual bool isEqual(const OccupancyState &other, double precision = PRECISION) const;
        virtual bool isEqual(const std::shared_ptr<State> &other, double precision = PRECISION) const;
        virtual bool isEqualNorm1(const std::shared_ptr<BeliefInterface> &other, double precision = PRECISION) const;
        virtual bool isEqualNormInf(const std::shared_ptr<BeliefInterface> &other, double precision = PRECISION) const;

        virtual double getProbability(const std::shared_ptr<State> &joint_history) const;
        virtual double getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history) const;
        virtual double getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &state) const;

        virtual void setProbability(const std::shared_ptr<State> &joint_history, double proba);
        virtual void setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

        virtual void addProbability(const std::shared_ptr<State> &joint_history, double proba);
        virtual void addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);
        virtual std::shared_ptr<Action> applyDR(const std::shared_ptr<DecisionRule> &dr, const std::shared_ptr<JointHistoryInterface> &joint_history) const;

        virtual bool checkCompatibility(const std::shared_ptr<Observation> &, const std::shared_ptr<Observation> &);
        virtual Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);
        virtual void updateOccupancyStateProba(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double probability);
        virtual Pair<std::shared_ptr<OccupancyStateInterface>, double> finalizeNextState(const std::shared_ptr<OccupancyStateInterface> &one_step_occupancy_state, number t);

        virtual double getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t);

        virtual Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> sampleJointHistoryBelief();

        /**
         * @brief Get the set of joint histories that are in the support of the occupancy state.
         * @return the possible joint hitories
         */
        virtual const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistories() const;

        /**
         * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

         * @return the possible states
         */
        virtual const std::set<std::shared_ptr<BeliefInterface>> &getBeliefs() const;

        /**
         * @brief Get the belief corresponding to the given joint historiy.

         * @return the belief
         */
        virtual std::shared_ptr<BeliefInterface> getBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const;

        /**
         * @brief Set the belief corresponding to the given joint historiy.
         *
         * @param jhistory the joint history
         * @param belief the corresponding belief
         */
        virtual void setBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory, const std::shared_ptr<BeliefInterface> &belief);

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
         * @param number the agent identifier
         */
        virtual const std::set<std::shared_ptr<HistoryInterface>> &getIndividualHistories(number ag_id) const;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
         */
        virtual const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &getAllIndividualHistories() const;

        virtual void finalize();
        virtual void finalize(bool do_compression);

        /**
         * @brief Get the fully uncompressed occupancy state.
         */
        virtual std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy();

        /**
         * @brief Set the fully uncompressed occupancy state.
         */
        virtual void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

        /**
         * @brief Get the one step uncompressed occupancy state.
         */
        virtual std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy();

        /**
         * @brief Set the one step uncompressed occupancy state
         */
        virtual void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

        /**
         * @brief Get the compressed occupancy state
         */
        virtual std::shared_ptr<OccupancyStateInterface> getCompressedOccupancy();

        /**
         * @brief Set the compressed occupancy state
         */
        virtual void setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate);

        /**
         * @brief Get the label that corresponds to the ihistory.
         */
        virtual std::shared_ptr<HistoryInterface> getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const;

        /**
         * @brief Get the list of labels that corresponds to the list of ihistories.
         */
        virtual Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &) const;

        /**
         * @brief Get the probability over individual histories and precise agent
         *
         * @param number Agent Id
         * @param typename jhistory_type::element_type::ihistory_type : Individual History
         */
        virtual double getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryInterface> &) const;

        /**
         * @brief Update the label of a specific individual history
         */
        virtual void updateLabel(number, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<HistoryInterface> &);

        /**
         * @brief Update the labels of multiple individual histories
         */
        virtual void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &, const Joint<std::shared_ptr<HistoryInterface>> &);

        /**
         * @brief Get the Compressed Joint History.
         */
        virtual std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Check probabilistic equivalence
         *
         * @return true if histories are equivalent
         * @return false else
         */
        virtual bool areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<HistoryInterface> &, number);

        /**
         * @brief Compression for occupancy states based on belief state representation.
         * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.
         *
         * @return the compressed occupancy state
         */
        virtual std::shared_ptr<OccupancyStateInterface> compress();

        /**
         * @brief Get the Private Occupancy States object
         *
         * @return const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>>&
         */
        virtual const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &getPrivateOccupancyStates() const;

        /**
         * @brief Get the
         *
         * @param agent_id
         * @param ihistory
         * @return const std::shared_ptr<PrivateOccupancyState>&
         */
        virtual const std::shared_ptr<PrivateOccupancyState> &getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const;

        virtual std::shared_ptr<OccupancyState> getptr();

        virtual std::string str() const;

        virtual double product(const std::shared_ptr<AlphaVector> &alpha);
        virtual double product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action);

        virtual bool operator==(const std::shared_ptr<BeliefInterface> &other) const;

        virtual std::shared_ptr<Space> getActionSpaceAt(number t);
        virtual void setActionSpaceAt(number t, std::shared_ptr<Space> action_space);
        virtual void setup();
        virtual void normalize();

        virtual std::shared_ptr<JointHistoryInterface> getJointHistory(std::shared_ptr<JointHistoryInterface> candidate_jhistory);

        /** @brief Keep relation between list of individual histories and joint histories */
        static RecursiveMap<Joint<std::shared_ptr<HistoryInterface>>, std::shared_ptr<JointHistoryInterface>> jhistory_map_;

    protected:
        /** @brief the number of agents */
        number num_agents_ = 2, h;

        /** @brief This representation of occupancy states consists of private occupancy states for each agent */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> tuple_of_maps_from_histories_to_private_occupancy_states_;

        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, double>> weight_of_private_occupancy_state_;

        /** @brief Keep in memory the uncompressed occupancy states */
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state;

        std::weak_ptr<OccupancyStateInterface> compressed_occupancy_state;

        /** @brief Keep relations between all private ihistories and labels */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<HistoryInterface>>> private_ihistory_map_;

        /** @brief probability of a private history space for a precise agent */
        std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, double>> probability_ihistories;

        /**
         * @brief space of all reachable states, those in the support of the occupancy state
         * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
         */
        std::set<std::shared_ptr<BeliefInterface>> list_beliefs_;

        /**
         * @brief space of joint histories
         */
        std::set<std::shared_ptr<JointHistoryInterface>> list_joint_histories_;

        /**
         * @brief tuple of private history spaces, one private history space per agent
         */
        std::vector<std::set<std::shared_ptr<HistoryInterface>>> all_list_ihistories_;

        /**
         * @brief mapping from joint history to belief
         */
        std::unordered_map<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> map_joint_history_to_belief_;

        /**
         * @brief
         */
        std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, std::set<std::shared_ptr<JointHistoryInterface>>>> ihistories_to_jhistory_;

        virtual Pair<std::shared_ptr<State>, double> computeNext(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);
        virtual Pair<std::shared_ptr<State>, double> computeNextKeepAll(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);
        virtual Pair<std::shared_ptr<OccupancyStateInterface>, double> finalizeKeepAll(const std::shared_ptr<OccupancyStateInterface> &one_step_occupancy_state, const std::shared_ptr<OccupancyStateInterface> &fully_uncompressed_occupancy_state, number t);

        virtual void setupIndividualHistories();
        virtual void setupBeliefsAndHistories();
        virtual void setProbabilityOverIndividualHistories();
        virtual void setupPrivateOccupancyStates();

        std::shared_ptr<std::unordered_map<number, std::shared_ptr<Space>>> action_space_map;

        // necessary for now for phoMDP
        std::vector<std::shared_ptr<std::unordered_map<number, std::vector<std::shared_ptr<JointHistoryInterface>>>>> individual_hierarchical_history_vector_map_vector;
        //
        std::shared_ptr<std::unordered_map<number, std::vector<std::shared_ptr<JointHistoryInterface>>>> joint_history_map_vector;
    };
} // namespace sdm

DEFINE_STD_HASH(sdm::OccupancyState, sdm::OccupancyState::PRECISION);