
namespace sdm
{
    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TabularQValueFunction<TState, TAction, TValue, TMatrix>::TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<TState, TAction>> initializer)
        : QValueFunction<TState, TAction, TValue>(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TabularQValueFunction<TState, TAction, TValue, TMatrix>::TabularQValueFunction(number horizon, double learning_rate, TValue default_value) : TabularQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void TabularQValueFunction<TState, TAction, TValue, TMatrix>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void TabularQValueFunction<TState, TAction, TValue, TMatrix>::initialize(TValue default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    std::shared_ptr<VectorImpl<TAction, TValue>> TabularQValueFunction<TState, TAction, TValue, TMatrix>::getQValueAt(const TState &state, number t)
    {
        using v_type = typename TMatrix<TState, TAction, TValue>::value_type::second_type;

        return std::make_shared<v_type>(this->representation[this->isInfiniteHorizon() ? 0 : t].at(state));
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    TValue TabularQValueFunction<TState, TAction, TValue, TMatrix>::getQValueAt(const TState &state, const TAction &action, number t)
    {
        return this->representation[this->isInfiniteHorizon() ? 0 : t].at(state).at(action);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void TabularQValueFunction<TState, TAction, TValue, TMatrix>::updateQValueAt(const TState &state, const TAction &action, number t, TValue target)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        this->representation[h][state][action] = this->representation[h].at(state).at(action) + this->learning_rate_ * target;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    void TabularQValueFunction<TState, TAction, TValue, TMatrix>::updateQValueAt(const TState &, const TAction &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TS, typename TA, typename TV> class TMatrix>
    std::string TabularQValueFunction<TState, TAction, TValue, TMatrix>::str()
    {
        std::ostringstream res;
        res << "<tabular_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (auto pair_st_val : this->representation[i])
            {
                res << "\t\t<state id=\"" << pair_st_val.first << "\">" << std::endl;
                for (auto pair_act_val : pair_st_val.second)
                {
                    res << "\t\t\t<action id=\"" << pair_act_val.first << "\">" << std::endl;
                    res << "\t\t\t\t" << pair_act_val.second << std::endl;
                    res << "\t\t\t</action>" << std::endl;
                }
                // res << "\t\t\t" << pair_st_val.second << std::endl;
                res << "\t\t</state>" << std::endl;
            }
            res << "\t</value>" << std::endl;
        }

        res << "</tabular_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm