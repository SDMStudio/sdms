#pragma once

// #include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{

    class QValueBackupInterface
    {
    public:  
        /**
         * @brief 
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        virtual double backup(number t) = 0;
    };

} // namespace sdm
