#pragma once
#include <math.h>

#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TData>
    class BackupOperator
    {
    public:
        using data_type = TData;

        virtual TData backup(ValueFunction<TState, TAction> *vf, TState s, int t) = 0;
    };

    template <typename TState, typename TAction>
    class ClassicBellmanBackupOperator : public BackupOperator<TState, TAction, double>
    {
    public:
        double backup(ValueFunction<TState, TAction> *vf, TState s, int t)
        {
            return vf->getQValueAt(s, t)->max();
        }
    };

    // template <typename TVector, typename TAction>
    // class MacPlanBackupOperator : public BackupOperator<TVector, TAction, TVector>
    // {
    // public:
    //     TVector backup(ValueFunction<TVector, TAction> *vf, TVector s, int t)
    //     {

    //         return vf->getQValueAt(s, t)->max();
    //     }
    // };
} // namespace sdm
