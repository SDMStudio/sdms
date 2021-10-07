#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    QValueFunction::QValueFunction()
    {
    }

    QValueFunction::QValueFunction(number horizon) : QValueFunctionBase(horizon)
    {
    }

    std::shared_ptr<QValueFunction> QValueFunction::getptr()
    {
        return std::static_pointer_cast<QValueFunction>(this->shared_from_this());
    }

    double QValueFunction::getValueAt(const TGlobalInput &input, number t)
    {
        return this->getQValueAt(input.first, input.second, t);
    }

    void QValueFunction::updateValueAt(const TGlobalInput &input, number t)
    {
        this->updateQValueAt(input.first, input.second, t);
    }

    std::shared_ptr<Action> QValueFunction::getBestAction(const TGlobalInput &, number)
    {
        //Pas de BestAction dans ton code Baris ?
        throw sdm::exception::NotImplementedException();
    }

} // namespace sdm