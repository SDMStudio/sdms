
namespace sdm
{
    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
        : ValueFunction<TState, TAction, TValue>(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value) : TabularValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::initialize(TValue default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TValue TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getValueAt(const TState &state, number t)
    {
        if (t < this->getHorizon() && this->init_function_ != nullptr)
        {
            if ((this->representation[t].find(state) == this->representation[t].end()))
            {
                TValue i_value = this->init_function_->operator()(state, t);
                this->updateValueAt(state, t, i_value);
                return i_value;
            }
        }

        return this->representation[this->isInfiniteHorizon() ? 0 : t].at(state);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TAction TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getBestAction(const TState &state, number t)
    {
        TAction best_action;
        TValue max = -std::numeric_limits<TValue>::max(), tmp;

        for (const auto &action : this->getWorld()->getActionSpaceAt(state)->getAll())
        {
            if (max < (tmp = this->getQValueAt(state, action, t)))
            {
                best_action = action;
                max = tmp;
            }
        }
        return best_action;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::updateValueAt(const TState &state, number t, TValue target)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = target;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    size_t TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getSize(number t) const
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].size();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::updateValueAt(const TState &state, number t)
    {
        this->updateValueAt(state, t, this->getBackupOperator().backup(this->getptr(), state, t));
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    typename TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::backup_operator_type TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getBackupOperator()
    {
        return this->backup_op_;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::save(std::string filename)
    {
        BoostSerializable<TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>>::save(filename);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::load(std::string filename)
    {
        BoostSerializable<TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>>::load(filename);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::string TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::str()
    {
        std::ostringstream res;
        res << "<tabular_value_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (std::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (const auto &pair_st_val : this->representation[i])
            {
                // res << "\t\t<state id=\"" << pair_st_val.first << "\">" << std::endl;
                // res << "\t\t</state>" << std::endl;
                std::ostringstream state_str;
                state_str << pair_st_val.first;
                res << "\t\t<state>" << std::endl;
                res << tools::addIndent(state_str.str(), 3) << std::endl;
                res << "\t\t</state>" << std::endl;
                res << "\t\t<value>" << std::endl;
                res << "\t\t\t" << pair_st_val.second << std::endl;
                res << "\t\t</value>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }

        res << "</tabular_value_function>" << std::endl;
        return res.str();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::vector<TState> TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>::getSupport(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].getIndexes();
    }

} // namespace sdm