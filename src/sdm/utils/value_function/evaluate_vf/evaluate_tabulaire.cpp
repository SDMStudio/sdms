#include <sdm/utils/value_function/evaluate_vf/evaluate_tabulair.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    EvaluateTabulaire::EvaluateTabulaire() {}
    EvaluateTabulaire::~EvaluateTabulaire(){}

    Pair<std::shared_ptr<State>,double> EvaluateTabulaire::evaluate(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        double value;

        // If the element state doesn't exits, we determine his value by using the initilisation function
        if (std::find(vf->getSupport(t).begin(),vf->getSupport(t).end(),state) == vf->getSupport(t).end() && vf->getInitFunction() != nullptr)
        {
            value = vf->getInitFunction()->operator()(state,t);
        }
        else
        {
            value = vf->getValueAt(state,t);
        }
        return std::make_pair(state,value);
    }
}