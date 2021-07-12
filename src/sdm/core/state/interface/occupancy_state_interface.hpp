#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/space/space.hpp>


namespace sdm
{
    class OccupancyStateInterface : virtual public BeliefInterface
    {
    public:
        virtual double getProbability(const std::shared_ptr<State> &pair_history_belief) const = 0;
        virtual double getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief) const = 0;

        virtual void setProbability(const std::shared_ptr<State> &pair_history_belief, double proba) = 0;
        virtual void setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba) = 0;

        virtual void addProbability(const std::shared_ptr<State> &pair_history_belief, double proba) = 0;
        virtual void addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba) = 0;

        virtual Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> sampleJointHistoryBelief() = 0;

        /**
         * @brief Get the set of joint histories that are in the support of the occupancy state.
         * @return the possible joint hitories
         */
        virtual const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistories() const = 0;

        /**
         * @brief Get the set of states that are in the support of the occupancy state for a precise joint historiy.

         * @return the possible states
         */
        virtual const std::set<std::shared_ptr<BeliefInterface>> &getBeliefs() const = 0;

        /**
         * @brief Get the set of beliefs at a given joint history
         * 
         * @param jhistory 
         * @return const std::set<std::shared_ptr<BeliefInterface>>& 
         */
        virtual const std::set<std::shared_ptr<BeliefInterface>> &getBeliefsAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const = 0;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
         * @param number the agent identifier
         */
        virtual const std::set<std::shared_ptr<HistoryInterface>> &getIndividualHistories(number ag_id) const = 0;

        /**
         * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
         */
        virtual const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &getAllIndividualHistories() const = 0;

        /**
         * @brief Get the fully uncompressed occupancy state.
         */
        virtual std::shared_ptr<OccupancyStateInterface> getFullyUncompressedOccupancy() const = 0;

        /**
         * @brief Set the fully uncompressed occupancy state.
         */
        virtual void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;

        /**
         * @brief Get the one step uncompressed occupancy state. 
         */
        virtual std::shared_ptr<OccupancyStateInterface> getOneStepUncompressedOccupancy() const = 0;

        /**
         * @brief Set the one step uncompressed occupancy state
         */
        virtual void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &) = 0;

        /**
         * @brief Get the list of labels that corresponds to the list of ihistories.
         */
        virtual Joint<std::shared_ptr<HistoryInterface>> getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &) const = 0;

        /**
         * @brief Get all the probability conditionning to a Joint History
         * 
         * @param std::shared_ptr<JointHistoryInterface> : Joint History
         */
        virtual const double &getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &) const = 0;

        /**
         * @brief Update the labels of multiple individual histories
         */
        virtual void updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &, const Joint<std::shared_ptr<HistoryInterface>> &) = 0;

        /**
         * @brief Get the Compressed Joint History. 
         */
        virtual std::shared_ptr<JointHistoryInterface> getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &) const = 0;

        /**
         * @brief Get the probability over individual histories and precise agent
         * 
         * @param number Agent Id
         * @param typename jhistory_type::element_type::ihistory_type : Individual History
         */
        virtual const double &getProbabilityOverIndividualHistories(number, const std::shared_ptr<HistoryInterface> &) const = 0;

        /**
         * @brief Compression for occupancy states based on belief state representation. 
         * To be in this representation, the type 'TState' have to be a derivation of the interface BeliefState.  
         * 
         * @return the compressed occupancy state 
         */
        virtual std::shared_ptr<OccupancyStateInterface> compress() = 0;

        virtual void finalize() = 0;

        virtual std::shared_ptr<Space> getActionSpaceAt(number t) = 0;
        virtual void setActionSpaceAt(number t, std::shared_ptr<Space> action_space) = 0;
        //
        virtual std::vector<std::shared_ptr<JointHistoryInterface>> getIndividualHierarchicalHistoryVectorFor(number t, number agent) = 0;
        virtual void pushToIndividualHierarchicalHistoryVectorFor(number t, number agent, std::shared_ptr<JointHistoryInterface>& individual_hierarchical_history) = 0;
    };
}