#include <sdm/utils/value_function/tabular_qvalue_function.hpp>

namespace sdm
{
    TabularQValueFunction::TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer)
        : QValueFunction(horizon), learning_rate_(learning_rate), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    TabularQValueFunction::TabularQValueFunction(number horizon, double learning_rate, double default_value) : TabularQValueFunction(horizon, learning_rate, std::make_shared<ValueInitializer>(default_value))
    {
    }

    void TabularQValueFunction::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    void TabularQValueFunction::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> TabularQValueFunction::getQValuesAt(const std::shared_ptr<State> &state, number t)
    {
        using v_type = typename MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>::value_type::second_type;
        return std::make_shared<v_type>(this->representation[this->isInfiniteHorizon() ? 0 : t].at(state));
    }

    double TabularQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValuesAt(state, t)->at(action);
    }

    void TabularQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number step, double delta)
    {
        auto h = this->isInfiniteHorizon() ? 0 : step;
        this->representation[h][state][action] = this->representation[h][state][action] + this->learning_rate_ * delta;
    }

    void TabularQValueFunction::updateQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    int TabularQValueFunction::getNumStates() const
    {
        number num_states = 0;
        for (number h = 0; h < getHorizon(); h++)
        {
            num_states += this->representation[h].size();
        }
        return num_states;
    }

    std::string TabularQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<tabular_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (auto state__actions_values : this->representation[i])
            {
                res << "\t\t<state id=\"" << state__actions_values.first << "\">" << std::endl;
                tools::indentedOutput(res, state__actions_values.first->str().c_str(), 3);
                res << std::endl;
                res << "\t\t</state>" << std::endl;
                res << "\t\t<actions>" << std::endl;
                for (auto action_value : state__actions_values.second)
                {
                    res << "\t\t\t<action id=\"" << action_value.first << "\" value=" << action_value.second << ">" << std::endl;
                    tools::indentedOutput(res, action_value.first->str().c_str(), 4);
                    res << std::endl
                        << "\t\t\t</action>" << std::endl;
                }
                res << "\t\t</actions>" << std::endl;
            }
            res << "\t</timestep>" << std::endl;
        }

        res << "</tabular_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm