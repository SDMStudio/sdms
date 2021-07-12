#pragma once
#include <sdm/types.hpp>
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
     * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
     * 
     * @tparam TState refers to a number
     * @tparam TJointHistory_p refers to a joint histories
     */
    class OccupancyState : public OccupancyStateInterface,
                           public Belief
    {
    public:
        static double PRECISION;

        OccupancyState();
        OccupancyState(number num_agents);
        OccupancyState(const OccupancyState &copy);

        bool operator==(const OccupancyState &other) const;

        double getProbability(const std::shared_ptr<State> &joint_history) const;
        double getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history) const;
        double getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &state) const;

        void setProbability(const std::shared_ptr<State> &joint_history, double proba);
        void setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

        void addProbability(const std::shared_ptr<State> &joint_history, double proba);
        void addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

        Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> sampleJointHistoryBelief();

        /**
         * @brief Get the set of joint histories that are in the support of the occupancy state.
         * @return the possible joint hitories
         */
        std::set<std::shared_ptr<JointHistoryInterface>> getJointHistories() const;

        /**
         * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

         * @return the possible states
         */
        std::set<std::shared_ptr<BeliefInterface>> getBeliefs() const;

        /**
         * @brief Get the belief corresponding to the given joint historiy.

         * @return the belief
         */
        std::shared_ptr<BeliefInterface> getBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const;

        /**
         * @brief Set the belief corresponding to the given joint historiy.
         * 
         * @param jhistory the joint history
         * @param belief the corresponding belief 
         */
        void setBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory, const std::shared_ptr<BeliefInterface> &belief);

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
         * @param number the agent identifier
         */
        std::set<std::shared_ptr<HistoryInterface>> getIndividualHistories(number ag_id) const;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
         */
        std::vector<std::set<std::shared_ptr<HistoryInterface>>> getAllIndividualHistories() const;

        void finalize();

        /**
         * @brief Get the fully uncompressed occupancy state.
         */
        std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy() const;

        /**
         * @brief Set the fully uncompressed occupancy state.
         */
        void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

        /**
         * @brief Get the one step uncompressed occupancy state. 
         */
        std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy() const;

        /**
         * @brief Set the one step uncompressed occupancy state
         */
        void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &);

        std::shared_ptr<OccupancyStateInterface> getCompressedOccupancy() const;

        /**
         * @brief Set the compressed occupancy state
         */
        void setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate);

        /**
         * @brief Get the label that corresponds to the ihistory.
         */
        std::shared_ptr<HistoryInterface> getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const;

        /**
         * @brief Get the list of labels that corresponds to the list of ihistories.
         */
        Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &) const;

        /**
         * @brief Get the probability over individual histories and precise agent
         * 
         * @param number Agent Id
         * @param typename jhistory_type::element_type::ihistory_type : Individual History
         */
        double getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryInterface> &) const;

        /**
         * @brief Update the label of a specific individual history
         */
        void updateLabel(number, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<HistoryInterface> &);

        /**
         * @brief Update the labels of multiple individual histories
         */
        void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &, const Joint<std::shared_ptr<HistoryInterface>> &);

        /**
         * @brief Get the Compressed Joint History. 
         */
        std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Check probabilistic equivalence
         * 
         * @return true 
         * @return false 
         */
        bool areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<HistoryInterface> &, number);

        /**
         * @brief Compression for occupancy states based on belief state representation. 
         * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
         * 
         * @return the compressed occupancy state 
         */
        std::shared_ptr<OccupancyStateInterface> compress();

        TypeState getTypeState() const;

        std::shared_ptr<OccupancyState> getptr();

        std::string str() const;

        double operator^(const std::shared_ptr<BeliefInterface> &other) const;
        bool operator==(const std::shared_ptr<BeliefInterface> &other) const;

        std::shared_ptr<Space> getActionSpaceAt(number t);
        void setActionSpaceAt(number t, std::shared_ptr<Space> action_space);
        void setup();
        void normalize();

    protected:
        /**
         * @brief the number of agents 
         */
        number num_agents_ = 2;

        /** @brief This representation of occupancy states consists of private occupancy states for each agent */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> tuple_of_maps_from_histories_to_private_occupancy_states_;
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, double>> weight_of_private_occupancy_state_;

        /** @brief Keep in memory the uncompressed occupancy states */
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state, compressed_occupancy_state;

        /** @brief Keep relations between all private ihistories and labels */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<std::shared_ptr<HistoryInterface>>>> private_ihistory_map_;

        /** @brief */
        Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<std::shared_ptr<HistoryInterface>>>> map_label_to_pointer;

        /** @brief Keep relation between list of individual histories and joint histories */
        RecursiveMap<Joint<std::shared_ptr<HistoryInterface>>, std::shared_ptr<JointHistoryInterface>> jhistory_map_;

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
        RecursiveMap<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> map_joint_history_to_belief_;

        std::unordered_map<number, std::unordered_map<std::shared_ptr<HistoryInterface>, std::set<std::shared_ptr<JointHistoryInterface>>>> ihistories_to_jhistory_;

        /**
         * @brief Get the Private Occupancy States object
         * 
         * @return const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>>& 
         */
        const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &getPrivateOccupancyStates() const;

        /**
         * @brief Get the 
         * 
         * @param agent_id 
         * @param ihistory 
         * @return const std::shared_ptr<PrivateOccupancyState>& 
         */
        const std::shared_ptr<PrivateOccupancyState> &getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const;

        void setupIndividualHistories();
        void setupBeliefsAndHistories();
        void setProbabilityOverIndividualHistories();
        void setupPrivateOccupancyStates();

        std::shared_ptr<std::unordered_map<number, std::shared_ptr<Space>>> action_space_map;
    };
} // namespace sdm

namespace std
{

    template <>
    struct hash<sdm::OccupancyState>
    {
        typedef sdm::OccupancyState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {

            size_t seed = 0;
            std::map<std::shared_ptr<sdm::State>, double> ordered(in.begin(), in.end());
            std::vector<int> rounded;
            for (const auto &pair_jhist_proba : ordered)
            {
                sdm::hash_combine(seed, pair_jhist_proba.first);
                sdm::hash_combine(seed, in.getBeliefAt(pair_jhist_proba.first->toHistory()->toJointHistory()));
                rounded.push_back(lround(sdm::OccupancyState::PRECISION * pair_jhist_proba.second));
            }
            for (const auto &v : rounded)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
            return seed;
            // return std::hash<sdm::MappedVector<std::shared_ptr<sdm::State>>>()(in, sdm::OccupancyState::PRECISION);
        }
    };
}