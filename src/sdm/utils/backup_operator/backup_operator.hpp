#pragma once
#include <math.h>

#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TData>
    class BackupOperator
    {
    public:
        using data_type = TData;

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& s, number t) = 0;
    };

    class ClassicBellmanBackupOperator : public BackupOperator<double>
    {
    public:
        double backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& s, number t)
        {
            // return vf->getQValueAt(s, t)->max();
        }
    };

    // template <typename TState, typename TAction>
    // class ClassicQBackupOperator : public BackupOperator<TState, TAction, double>
    // {
    // public:
    //     double backup(std::shared_ptr<ValueFunction<TState, TAction>>vf, TState s, int t)
    //     {
    //         return vf->getQValueAt(s, t)->max();
    //     }
    // };
} // namespace sdm
