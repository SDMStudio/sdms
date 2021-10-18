#include <sdm/utils/value_function/vfunction/value_function_approximation_interface.hpp>

namespace sdm
{

    ValueFunctionApproximationInterface::ValueFunctionApproximationInterface(number horizon, const std::shared_ptr<Initializer> &initializer,
                                                                             const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                                                             const std::shared_ptr<UpdateOperatorInterface> &update_operator,
                                                                             int freq_pruning) : ValueFunction(horizon, initializer, action_selection, update_operator),
                                                                                                 freq_pruning(freq_pruning)
    {
    }
    
    void ValueFunctionApproximationInterface::doPruning(number trial)
    {
        if (trial % this->getPruningFrequency() == 0)
        {
            for (number time = 0; time < this->getHorizon(); time++)
            {
                this->prune(time);
            }
        }
    }

    int ValueFunctionApproximationInterface::getPruningFrequency() const
    {
        return this->freq_pruning;
    }

} // namespace sdm
