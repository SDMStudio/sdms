
namespace sdm
{
    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
        : ValueFunction<TState, TAction, TValue>(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_, Container());
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value) : TabularValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::initialize()
    {
        this->initializer_->init(this);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::initialize(TValue default_value, number t)
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
    TValue TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getValueAt(const TState &state, number t)
    {
        if (t < this->getHorizon())
        {
            bool found = false;
            auto first = this->representation[t].begin();
            auto last = this->representation[t].end();

            while (first != last && !found)
            {
                if (first->first == state)
                {
                    found = true;
                }
                ++first;
            }
            if (!found)
            {
                if (this->init_function_ != nullptr)
                {
                    TValue i_value = this->init_function_->operator()(state, t);
                    this->updateValueAt(state, t, i_value);
                    return i_value;
                }
            }
        }

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
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::updateValueAt(const TState &state, number t, TValue target)
    {
        if (this->isInfiniteHorizon())
        {
            this->representation[0][state] = target;
        }
        else
        {
            assert(t < this->horizon_);
            // std::cout << " / Found ?  "<< (this->representation[t].find(state) != this->representation[t].end()) << " -- V(" << state << "," << t << ") = " << this->representation[t][state] <<std::endl;
            this->representation[t][state] = target;
        }
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::updateValueAt(const TState &state, number t)
    {
        this->updateValueAt(state, t, this->getBackupOperator().backup(this, state, t));
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    typename TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::backup_operator_type TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getBackupOperator()
    {
        return this->backup_op_;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::string TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::str()
    {
        std::ostringstream res;
        res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
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

        res << "</tabular_value_function>" << std::endl;
        return res.str();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::vector<TState> TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getSupport(number t)
    {
        if (this->isInfiniteHorizon())
        {
            return this->representation[0].getIndexes();
        }
        else
        {
            return (t >= this->getHorizon()) ? std::vector<TState>{} : this->representation[t].getIndexes();
        }
    }

} // namespace sdm