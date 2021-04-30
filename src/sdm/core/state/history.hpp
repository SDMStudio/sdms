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
#include <sdm/core/state/history_tree.hpp>
#include <sdm/core/state/jhistory_tree.hpp>

/**
 * @namespace sdms
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    /**
     * @brief History seq alias
     * 
     * @tparam T 
     */
    template <typename T>
    using History = std::vector<T>;

    using int_history = History<int>;
    using char_history = History<char>;
    // using tensor_history = History<Tensor>;

    template <typename T>
    using HistoryTree_p = std::shared_ptr<HistoryTree<T>>;

    template <typename T>
    using JointHistoryTree_p = std::shared_ptr<JointHistoryTree<T>>;

    template <typename T>
    using JointJointHistoryTree_p = std::shared_ptr<Joint<std::shared_ptr<JointHistoryTree<T>>>>;

    template <typename obs_t>
    using ObsHistoryTree_p = HistoryTree_p<obs_t>;

    template <typename action_t, typename obs_t>
    using ActObsHistoryTree_p = HistoryTree_p<std::pair<action_t, obs_t>>;

    // using tensor_jhistory_tree = JointHistoryTree<Tensor>;
    

} // namespace sdm