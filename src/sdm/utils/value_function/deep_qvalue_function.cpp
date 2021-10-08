#include <algorithm>

#include <sdm/types.hpp>
#include <sdm/utils/nn/mlpnet.hpp>
#include <sdm/utils/value_function/deep_qvalue_function.hpp>

namespace sdm
{
    DeepQValueFunction::DeepQValueFunction(number num_actions, number num_observations, number memory, number horizon)
    {
        unsigned long input, output;
        for (number step = 0; step < horizon; ++step)
        {
            output = num_actions;
            input = 2 * std::pow(num_observations, std::min(memory, step));
            this->qnetworks[step] = nn::MlpNet(input, output);
        }
    }

    void DeepQValueFunction::initialize()
    {
        
    }

    double DeepQValueFunction::getQValueAt(const std::shared_ptr<OccupancyStateInterface> &state, const std::shared_ptr<JointHistoryInterface> &observation, const std::shared_ptr<Action> &action, number t)
    {
    }

    double DeepQValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {

    }

    void DeepQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        // definir le tenseur torch
    }

    void DeepQValueFunction::updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta)
    {
    }

    void DeepQValueFunction::updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &state, const std::shared_ptr<JointHistoryInterface> &observation, const std::shared_ptr<Action> &action, number t, double delta)
    {
    }

    std::string DeepQValueFunction::str() const
    {
    }

} // namespace sdm
