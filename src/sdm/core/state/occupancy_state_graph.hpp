#pragma once

#include <sdm/core/state/belief_state_graph.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    /**
     * @brief A graph that keep all beliefs
     * 
     */
    class OccupancyStateGraph : public OccupancyStateInterface, public BeliefStateGraph
    {
    public:
        using TransitionFunction = Pair<std::shared_ptr<BeliefInterface>, double>(const std::shared_ptr<POMDPInterface> &,
                                                                                  const std::shared_ptr<BeliefInterface> &,
                                                                                  const std::shared_ptr<Action> &,
                                                                                  const std::shared_ptr<Observation> &,
                                                                                  number);

        OccupancyStateGraph();
        // OccupancyStateGraph(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);
        OccupancyStateGraph(const std::shared_ptr<BeliefInterface> &data);

        /**
         * @brief Construct a new belief 
         * 
         * @param predecessor 
         * @param belief 
         */
        OccupancyStateGraph(const std::shared_ptr<OccupancyStateGraph> &predecessor, const std::shared_ptr<BeliefInterface> &belief);

        void initialize();

        /**
         * @brief Expands the graph
         * 
         * @param action the action
         * @param observation the observation
         * @param backup if true, we store the expanded belief in the graph.
         * @return the next belief
         */
        std::shared_ptr<BeliefStateGraph> next(const std::shared_ptr<BeliefInterface>&belief, double probability , const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t, bool backup = true);

        /**
         * @brief Return a 
         */
        std::string str() const;

        /**
         * @brief Return a the pointer on this object
         */
        std::shared_ptr<BeliefStateGraph> getptr();

        TypeState getTypeState() const;


        // std::shared_ptr<BeliefInterface> ->
        // std::unordered_map<Belief,std::shared_ptr<BeliefInterface>>
        

        double getProbability(const std::shared_ptr<State> &state) const;
        void setProbability(const std::shared_ptr<State> &state, double proba);
        void addProbability(const std::shared_ptr<State> &, double proba);

        double getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief) const;

        void setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

        void addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba);

        /**
         * @brief Get the set of joint histories that are in the support of the occupancy state.
         * @return the possible joint hitories
         */
        const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistories() const;

        /**
         * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

         * @return the possible states
         */
        const std::set<std::shared_ptr<BeliefInterface>> &getBeliefs() const;

        /**
         * @brief Get the set of beliefs at a given joint history
         * 
         * @param jhistory 
         * @return const std::set<std::shared_ptr<BeliefInterface>>& 
         */
        const std::set<std::shared_ptr<BeliefInterface>> &getBeliefsAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
         * @param number the agent identifier
         */
        const std::set<std::shared_ptr<HistoryInterface>> &getIndividualHistories(number ag_id) const;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
         */
        const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &getAllIndividualHistories() const;

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

        /**
         * @brief Get the list of labels that corresponds to the list of ihistories.
         */
        Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &) const;

        /**
         * @brief Get all the probability conditionning to a Joint History
         * 
         * @param std::shared_ptr<JointHistoryInterface> : Joint History
         */
        const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Update the labels of multiple individual histories
         */
        void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &, const Joint<std::shared_ptr<HistoryInterface>> &);

        /**
         * @brief Get the Compressed Joint History. 
         */
        std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &) const;

        /**
         * @brief Get the probability over individual histories and precise agent
         * 
         * @param number Agent Id
         * @param typename jhistory_type::element_type::ihistory_type : Individual History
         */
        const double &getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryInterface> &) const;

        /**
         * @brief Compression for occupancy states based on belief state representation. 
         * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
         * 
         * @return the compressed occupancy state 
         */
        std::shared_ptr<OccupancyStateInterface> compress();

        void finalize();
    };

} // namespace sdm
