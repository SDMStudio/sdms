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

    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> TabularQValueFunction::getQValuesAt(const std::shared_ptr<Observation> &observation, number t)
    {
        using v_type = typename MappedMatrix<std::shared_ptr<Observation>, std::shared_ptr<Action>, double>::value_type::second_type;
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return std::make_shared<v_type>(this->representation[h].at(observation));
    }

    double TabularQValueFunction::getQValueAt(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, number t)
    {
        return this->getQValuesAt(observation, t)->at(action);
    }

    double TabularQValueFunction::getValueAt(const std::shared_ptr<Observation> &observation, number t)
    {
        return this->getQValuesAt(observation, t)->max();
    }

    std::shared_ptr<Action> TabularQValueFunction::getBestAction(const std::shared_ptr<Observation> &observation, number t)
    {
        return this->getQValuesAt(observation, t)->argmax();
    }

    void TabularQValueFunction::updateQValueAt(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, number t, double delta)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        this->representation[h][observation][action] = this->representation[h].at(observation).at(action) + this->learning_rate_ * delta;
    }

    void TabularQValueFunction::updateQValueAt(const std::shared_ptr<Observation> &, const std::shared_ptr<Action> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    bool TabularQValueFunction::isNotSeen(const std::shared_ptr<Observation> &observation, number t)
    {
        auto h = this->isInfiniteHorizon() ? 0 : t;
        return (this->representation[h].find(observation) == this->representation[h].end());
    }

    std::string TabularQValueFunction::str() const
    {
        std::ostringstream res;
        res << "<tabular_qvalue_function horizon=\"" << ((this->isInfiniteHorizon()) ? "inf" : std::to_string(this->getHorizon())) << "\">" << std::endl;
        for (sdm::size_t i = 0; i < this->representation.size(); i++)
        {
            res << "\t<value timestep=\"" << ((this->isInfiniteHorizon()) ? "all" : std::to_string(i)) << "\" default=\"" << this->representation[i].getDefault() << "\">" << std::endl;
            for (auto pair_st_val : this->representation[i])
            {
                res << "\t\t<state id=\"" << pair_st_val.first << "\"" << std::endl;
                tools::indentedOutput(res, pair_st_val.first->str().c_str(), 3);
                res << "\t\t/>" << std::endl;
                for (auto pair_act_val : pair_st_val.second)
                {
                    res << "\t\t\t<action id=\"" << pair_act_val.first << "\"" << std::endl;
                    tools::indentedOutput(res, pair_act_val.first->str().c_str(), 4);
                    res << "\t\t\t/>" << std::endl;
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