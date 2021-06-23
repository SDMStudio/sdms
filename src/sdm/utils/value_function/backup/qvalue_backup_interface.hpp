#pragma once

#include <sdm/types.hpp>

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
