/**
 * @file incremental_vf.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 11/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <map>
#include <assert.h>

#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/world/posg.hpp>

namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TState 
     * @tparam TAction 
     */
    template <typename TState, typename TAction, typename TValueFunction>
    class IncrementalValueFunction : public TValueFunction<TState, TAction, TValue>
    {
    protected:
        /**
         * @brief The problem which incremental value function is evaluated 
         * 
         */

    public:
        /**
         * @brief Construct a new Incremental Value Function object
         * 
         * @param problem 
         * @param default_value 
         */
        IncrementalValueFunction(std::shared_ptr<POSG> problem);

        std::shared_ptr<POSG> getWorld();

        void updateValueAt(const TState &s, int t = 0){
            TBackupOperator backup_op(this, world);
            backup_op.updateAt(s, t);
        }
    };

} // namespace sdm

#include <sdm/utils/value_function/incremental_value_function.tpp>
