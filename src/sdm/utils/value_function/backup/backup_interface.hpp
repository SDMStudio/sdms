#pragma once

#include <sdm/core/function.hpp>

namespace sdm
{
    class ValueFunction;

    class BackupInterfaceForValueFunction
    {
    };

    template <class TData>
    class BackupInterface : public BackupInterfaceForValueFunction
    {
    public:

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param const std::shared_ptr<Action>& : action
         * @param number t : time step
         * @return TData 
         */
        virtual TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, number t) = 0;
    };
}