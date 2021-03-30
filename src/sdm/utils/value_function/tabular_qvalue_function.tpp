#include <sdm/utils/value_function/tabular_qvalue_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::TabularQValueFunction(int horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
        : QValueFunction<TState, TAction, TValue>(horizon), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_, Container());
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::TabularQValueFunction(int horizon, TValue default_value) : TabularQValueFunction(horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::initialize()
    {
        this->initializer_->init(this);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::initialize(TValue default_value, int t)
    {
        if (this->isInfiniteHorizon())
        {
            this->representation[0] = Container(default_value);
        }
        else
        {
            assert(t < this->getHorizon());
            this->representation[t] = Container(default_value);
        }
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TValue TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getValueAt(const TState &state, int t)
    {
        if (this->isInfiniteHorizon())
        {
            return this->representation[0].at(state);
        }
        else
        {
            return (t >= this->getHorizon()) ? 0 : this->representation[t].at(state);
        }
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::updateQValueAt(const TState &state, const TAction &action, int t, TValue target)
    {
        if (this->isInfiniteHorizon())
        {
            this->representation[0][Pair({state, action})] = target;
        }
        else
        {
            assert(t < this->horizon_);
            this->representation[t][Pair({state, action})] = target;
        }
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::updateQValueAt(const TState &state, const TAction &action, int t)
    {
        this->updateValueAt(state, t, this->getBackupOperator().backup(this, state, t));
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    typename TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::backup_operator_type TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getBackupOperator()
    {
        return this->backup_op_;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::string TabularQValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::str()
    {
        std::ostringstream res;
        res << "<tabular_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (int i=0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (auto pair_st_val : this->representation[i])
            {
                res << "\t\t<state id=\"" << pair_st_val.first << "\">" << std::endl;
                res << "\t\t\t" << pair_st_val.second << std::endl;
                res << "\t\t</state>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }

        res << "</tabular_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm