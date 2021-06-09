
namespace sdm
{
    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularValueFunction<TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, std::shared_ptr<Initializer> initializer)
        : ValueFunction(problem, horizon), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    TabularValueFunction<TBackupOperator, TStruct>::TabularValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, double default_value)
        : TabularValueFunction(problem, horizon, std::make_shared<ValueInitializer>(default_value))
    {
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TBackupOperator, TStruct>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TBackupOperator, TStruct>::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    double TabularValueFunction<TBackupOperator, TStruct>::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        if (t < this->getHorizon() && this->init_function_ != nullptr)
        {
            if ((this->representation[t].find(state) == this->representation[t].end()))
            {
                double i_value = this->init_function_->operator()(state, t);
                this->updateValueAt(state, t, i_value);
                return i_value;
            }
        }

        return this->representation[this->isInfiniteHorizon() ? 0 : t].at(state);
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::shared_ptr<Action> TabularValueFunction<TBackupOperator, TStruct>::getBestAction(const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto &action : *this->getWorld()->getActionSpaceAt(state, t))
        {
            auto casted_action = std::static_pointer_cast<Action>(action);
            if (max < (tmp = this->getQValueAt(state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }

        return best_action;
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TBackupOperator, TStruct>::updateValueAt(const std::shared_ptr<State> &state, number t, double target)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t][state] = target;
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TBackupOperator, TStruct>::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        this->updateValueAt(state, t, this->getBackupOperator().backup(this->getptr(), state, t));
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    typename TabularValueFunction<TBackupOperator, TStruct>::backup_operator_type TabularValueFunction<TBackupOperator, TStruct>::getBackupOperator()
    {
        return this->backup_op_;
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TBackupOperator, TStruct>::save(std::string filename)
    {
        BoostSerializable<TabularValueFunction<TBackupOperator, TStruct>>::save(filename);
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    void TabularValueFunction<TBackupOperator, TStruct>::load(std::string filename)
    {
        BoostSerializable<TabularValueFunction<TBackupOperator, TStruct>>::load(filename);
    }

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::string TabularValueFunction<TBackupOperator, TStruct>::str()
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

    template <class TBackupOperator, template <typename TI, typename TV> class TStruct>
    std::vector<std::shared_ptr<State>> TabularValueFunction<TBackupOperator, TStruct>::getSupport(number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].getIndexes();
    }

} // namespace sdm