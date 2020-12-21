/**
 * @file history.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains the generic history class
 * @version 0.1
 * @date 14/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <vector>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
// #include <torch/torch.h>

/**
 * @namespace sdms
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    /**
     * @brief History seq alias
     * 
     * @tparam item_t 
     */
    template <typename item_t>
    using History = State<std::vector<item_t>>;

    using int_history = History<int>;
    using char_history = History<char>;
    // using tensor_history = History<Tensor>;

    using int_jhistory = JointHistory<int>;
    using char_jhistory = JointHistory<char>;
    // using tensor_jhistory = JointHistory<Tensor>;

    /**
     * @brief History tree alias
     * 
     * @tparam item_t 
     */
    template <typename item_t>
    using HistoryTree = State<Tree<item_t>>;

    template <typename item_t>
    using history_tree_p = std::make_shared<HistoryTree<item_t>>;

    template <typename obs_t>
    using obs_history_tree_p = history_tree_p<obs_t>;

    template <typename action_t, typename obs_t>
    using act_obs_history_tree_p = history_tree_p<std::pair<action_t, obs_t>>;

    using int_history_tree = HistoryTree<int>;

    using char_history_tree = HistoryTree<char>;
    // using tensor_history_tree = HistoryTree<Tensor>;

    using int_jhistory_tree = JointHistoryTree<int>;
    using char_jhistory_tree = JointHistoryTree<char>;
    // using tensor_jhistory_tree = JointHistoryTree<Tensor>;
    

} // namespace sdm