/**
 * @file jhistory_tree.hpp
 * @author Jilles S. Dibangoye
 * @author David Albert
 * @brief History Tree data structure
 * @version 0.1
 * @date 14/12/2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <sdm/core/state/history_tree.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/types.hpp>
#include <sdm/core/observation/default_observation.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>

namespace sdm
{

    /**
     * @class JointHistoryTree
     *
     * @brief Joint history class that use a representation by tree.
     *
     * It can be viewed as a history over joint observations and as a list of individual histories.
     *
     */
    class JointHistoryTree : public JointHistoryInterface,
                             public HistoryTree,
                             public Joint<std::shared_ptr<HistoryInterface>>,
                             public BoostSerializable<JointHistoryTree>

    {
    public:
        using ihistory_type = std::shared_ptr<HistoryTree>;
        using value_type = typename HistoryTree::value_type;

        /*!
         *  @fn     JointHistoryTree()
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        JointHistoryTree();

        /**
         * @brief Construct a new joint history tree object (the origin)
         *
         * @param n_agents the number of agents
         */
        JointHistoryTree(number n_agents);

        /**
         * @brief Construct a new truncated joint history tree object (the origin)
         *
         * @param n_agents the number of agents
         * @param max_depth the maximal depth of the truncated history
         */
        JointHistoryTree(number n_agents, number max_depth);

        /**
         *  @brief  This constructor build a child node given its parent's node and a new joint item.
         *  @param  parent   the parent tree
         *  @param  item     the item
         */
        JointHistoryTree(std::shared_ptr<HistoryTree> parent, const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &item);

        /**
         * @brief Construct a new joint history based on individual histories
         * @warning This will build a well defined Joint<std::shared_ptr<HistoryTree>> structure but wrong HistoryTree<std::shared_ptr<Joint<T>>> !!
         *
         * @param ihistories the list of individual histories
         */
        JointHistoryTree(const Joint<std::shared_ptr<HistoryInterface>> &ihistories);

        number getNumAgents() const;

        std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Action> &joint_action = nullptr, bool backup = true);
        std::shared_ptr<JointHistoryInterface> expand(const std::shared_ptr<JointObservation> &joint_observation, const std::shared_ptr<JointAction> &joint_action = nullptr, bool = true);

        /**
         * @brief Get the address of the individual history of agent 'agent_id'
         *
         * @param agent_id the agent id
         * @return the address of the individual history of agent 'agent_id'
         */
        std::shared_ptr<HistoryInterface> getIndividualHistory(number agent_id) const;

        /**
         * @brief Get the address of the individual histories of all agents
         *
         * @return a vector that contains all individual histories
         */
        Joint<std::shared_ptr<HistoryInterface>> getIndividualHistories() const;

        std::string str() const;

        std::shared_ptr<JointHistoryTree> getptr();

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

        std::shared_ptr<JointObservation> getDefaultObs();
        void setDefaultObs(const std::shared_ptr<JointObservation> &default_observation);

        std::shared_ptr<JointHistoryTree> getParent() const;
        std::shared_ptr<JointHistoryTree> getOrigin();
        std::vector<std::shared_ptr<JointHistoryTree>> getChildren() const;
        std::shared_ptr<JointHistoryTree> getChild(const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &child_item) const;

        friend std::ostream &operator<<(std::ostream &os, JointHistoryTree &j_hist)
        {
            os << j_hist.str();
            return os;
        }

    protected:
        void addIndividualHistory(std::shared_ptr<HistoryInterface> ihist);

        void setupDefaultObs(number num_agents, const std::shared_ptr<Observation> &default_observation = sdm::NO_OBSERVATION);
        std::shared_ptr<JointObservation> default_observation_;

        /*!
         *  @brief  Expands the tree
         *  @param  data the data of the expanded node
         *  @return the expanded tree
         *
         *  If child leading from the item previously exists, the method return
         *  that child. Otherwise, it expands the tree by adding an item at the
         *  current leaf of the tree and creating if necessary a corresponding
         *  child. The constructed child is returned.
         */
        std::shared_ptr<JointHistoryTree> expandJointHistoryTree(const std::shared_ptr<JointObservation> &joint_observation, const std::shared_ptr<JointAction> &joint_action = nullptr, bool backup = true);
        virtual std::shared_ptr<HistoryTree> expandHistoryTree(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Action> &joint_action, bool backup);
    };

} // namespace sdm