/**
 * @file history_tree.hpp
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

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/utils/struct/tree.hpp>
#include <sdm/core/state/interface/history_interface.hpp>

namespace sdm
{

    /**
     * @class HistoryTree
     *
     * @brief History class that use a representation by tree.
     *
     * Each node in the tree represent an action-observation pair.
     * Let consider nodes above a given node as the list of actions
     * and observations at previous timesteps.
     *
     */
    class HistoryTree : virtual public HistoryInterface,
                        public Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>
    {
    public:
        using value_type = typename Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::value_type;

        /**
         *  @brief  Default constructor.
         *  This constructor builds a default and empty tree.
         */
        HistoryTree();

        /**
         * @brief Construct a new truncated tree object
         *
         * @param data the value of the origin
         */
        HistoryTree(number max_depth);

        /**
         *  @brief Construct a history tree iteratively.
         *
         *  @param  parent   the parent tree
         *  @param  item     the item
         *  @param  backup wheter the node is marked or not
         *
         *  This constructor builds a tree with a given parent and item.
         */
        HistoryTree(std::shared_ptr<HistoryTree> parent, const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &item);

        std::shared_ptr<HistoryInterface> getPreviousHistory();

        std::shared_ptr<Observation> getLastObservation();

        number getHorizon() const;

        /**
         * @brief Expands the history
         *
         * @param observation the observation of the expanded node
         * @return the expanded history
         */
        std::shared_ptr<HistoryInterface> expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true);

        std::string str() const;
        std::string short_str() const;

        std::shared_ptr<HistoryTree> getptr();
        std::shared_ptr<HistoryTree> getParent() const;
        std::shared_ptr<HistoryTree> getOrigin();
        std::vector<std::shared_ptr<HistoryTree>> getChildren() const;
        std::shared_ptr<HistoryTree> getChild(const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &obs_act) const;

        const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &getData() const;

        friend std::ostream &operator<<(std::ostream &os, HistoryTree &i_hist)
        {
            os << i_hist.str();
            return os;
        }

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

    protected:
        /**
         *  @brief  Expands the tree using truncated expand method
         *
         *  @param  data the data of the expanded node
         *  @param  backup wheter the node is marked or not
         *
         *  @return the truncated expanded tree
         */
        template <typename output>
        std::shared_ptr<output> truncatedExpand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, bool backup)
        {
            std::list<value_type> items;
            const auto &obs_action = std::make_pair(observation, action);

            //<! fill in the vector of observation to simulate
            items.push_front(obs_action);
            auto parent = this;
            while (items.size() < this->getMaxDepth())
            {
                items.push_front(parent->getData());
                parent = parent->getParent().get();
            }

            //<! iteratively expands the base_graph
            auto trace = parent->getOrigin();

            for (auto it = items.begin(); it != items.end(); ++it)
            {
                trace = trace->expandHistoryTree(it->first, it->second, backup);
            }

            return std::static_pointer_cast<output>(trace);
        }

        /**
         *  @brief  Expands the history
         *  @param  data the data of the expanded node
         *  @return the expanded history
         *
         *  If child leading from the item previously exists, the method return
         *  that child. Otherwise, it expands the tree by adding an item at the
         *  current leaf of the tree and creating if necessary a corresponding
         *  child. The constructed child is returned.
         */
        template <typename output = HistoryTree>
        std::shared_ptr<output> expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true)
        {
            auto obs_action = std::make_pair(observation, action);
            if (backup && (this->children_.find(obs_action) != this->children_.end()))
            {
                return std::static_pointer_cast<output>(this->getChild(obs_action));
            }
            if (backup && (this->getDepth() >= this->getMaxDepth()))
            {
                if (this->getMaxDepth() == 0)
                    return std::static_pointer_cast<output>(this->getptr());
                else
                    return this->truncatedExpand<output>(observation, action, backup);
            }
            if (backup)
            {
                auto child = std::make_shared<output>(this->getptr(), obs_action);
                this->children_.emplace(obs_action, child);
                return child;
            }
            return std::make_shared<output>(this->getptr(), obs_action);
        }

        virtual std::shared_ptr<HistoryTree> expandHistoryTree(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action = nullptr, bool backup = true);
    };

} // namespace sdm
  // #include <sdm/core/state/history_tree.tpp>