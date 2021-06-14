#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    QValueFunction::QValueFunction()
    {
    }

    QValueFunction::QValueFunction(number horizon) : BaseValueFunction(horizon)
    {
    }

    double QValueFunction::getValueAt(const std::shared_ptr<Observation> &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    std::shared_ptr<Action> QValueFunction::getBestAction(const std::shared_ptr<Observation> &observation, number t)
    {
        auto qvalues = this->getQValuesAt(observation, t);
        return qvalues->argmax();
    }


    std::shared_ptr<QValueFunction> QValueFunction::getptr()
    {
        return std::static_pointer_cast<QValueFunction>(this->shared_from_this());
    }

} // namespace sdm