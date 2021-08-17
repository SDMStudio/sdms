#include <iterator>
#include <sdm/utils/value_function/initializer/initializer.hpp>

namespace sdm
{
    template <class TInput>
    TabularQValueFunction<TInput>::TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<TInput>> initializer, bool active_learning)
        : QValueFunction<TInput>(horizon), learning_rate_(learning_rate), active_learning_(active_learning), initializer_(initializer)
    {
        this->representation = std::vector<Container>(this->isInfiniteHorizon() ? 1 : this->horizon_ + 1, Container());
    }

    template <class TInput>
    TabularQValueFunction<TInput>::TabularQValueFunction(number horizon, double learning_rate, double default_value, bool active_learning) : TabularQValueFunction<TInput>(horizon, learning_rate, std::make_shared<ValueInitializer<TInput>>(default_value),active_learning)
    {
    }

    template <class TInput>
    void TabularQValueFunction<TInput>::initialize()
    {
        this->initializer_->init(this->getptr());
    }

    template <class TInput>
    void TabularQValueFunction<TInput>::initialize(double default_value, number t)
    {
        this->representation[this->isInfiniteHorizon() ? 0 : t] = Container(default_value);
    }

    template <class TInput>
    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> TabularQValueFunction<TInput>::getQValuesAt(const TInput &state, number t)
    {
        using v_type = typename MappedMatrix<TInput, std::shared_ptr<Action>, double>::value_type::second_type;
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return std::make_shared<v_type>(this->representation[h].at(state));
    }

    template <class TInput>
    double TabularQValueFunction<TInput>::getQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t)
    {
        // std::cout << "-------- TabularQValueFunction::getQValueAt() ---------" << std::endl;
        return this->getQValuesAt(state, t)->at(action);
    }

    // double TabularQValueFunction::getValueAt(const TInput &state, number t)
    // {
    //     return this->getQValuesAt(state, t)->max();
    // }

    // std::shared_ptr<Action> TabularQValueFunction::getBestAction(const TInput &state, number t)
    // {
    //     return this->getQValuesAt(state, t)->argmax();
    // }

    template <class TInput>
    void TabularQValueFunction<TInput>::updateQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;

        double new_value = (this->active_learning_) ? this->representation[h].at(state).at(action) + this->learning_rate_ * delta : delta;
        this->representation[h][state][action] = new_value;
    }

    template <class TInput>
    void TabularQValueFunction<TInput>::updateQValueAt(const TInput &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <class TInput>
    bool TabularQValueFunction<TInput>::isNotSeen(const TInput &state, number t)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return (this->representation[h].find(state) == this->representation[h].end());
    }

    template <class TInput>
    int TabularQValueFunction<TInput>::getNumStates() const
    {
        return 0;
    }

    // void TabularQValueFunction::printNumberOfActions()
    // {
    //     std::cout << "Printing number of actions" << std::endl;
    //     for (sdm::size_t i = 0; i < this->representation.size(); i++)
    //     {
    //         std::cout << "\ttimestep=" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << std::endl;
    //         for (auto state__actions_values : this->representation[i])
    //         {
    //             std::cout << "\t\tstate=" << state__actions_values.first << " number of actions = " << state__actions_values.second.size();
    //             bool ok = true;
    //             for (auto i = state__actions_values.second.begin(); i != state__actions_values.second.end(); i++)
    //             {
    //                 for (auto j = std::next(i); j != state__actions_values.second.end(); j++)
    //                 {
    //                     // ok = (!false);
    //                 }
    //             }
    //             std::cout << std::endl;
    //         }
    //     }
    // }

    template <class TInput>
    std::string TabularQValueFunction<TInput>::str() const
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
                    res << std::endl << "\t\t\t</action>" << std::endl;
                }
                res << "\t\t</actions>" << std::endl;
            }
            res << "\t</timestep>" << std::endl;
        }

        res << "</tabular_qvalue_function>" << std::endl;
        return res.str();
    }
} // namespace sdm