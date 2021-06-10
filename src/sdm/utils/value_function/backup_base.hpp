#pragma once
#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

#include <sdm/core/function.hpp>

namespace sdm
{
    class SolvableByHSVI;
    class ValueFunction;

    class BackupInterface
    {
    public:
        virtual std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
    };


    template<typename TData>
    class BackupBase : public BackupInterface
    {
    public:
        virtual std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
        virtual TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
    protected : 
        std::shared_ptr<SolvableByHSVI> world_;
    };

} // namespace sdm
