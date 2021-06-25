#pragma once

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{

    template<typename TData>
    class BackupBase : public BackupInterface<TData>
    {
    public:

        BackupBase();
        BackupBase(const std::shared_ptr<SolvableByHSVI>& world);
        
        virtual ~BackupBase();

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
    protected : 
        std::shared_ptr<SolvableByHSVI> world_;
    };

} // namespace sdm
#include <sdm/utils/value_function/backup/backup_base.tpp>
