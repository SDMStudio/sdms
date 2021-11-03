
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/algorithms/planning/rsvi.hpp>

namespace sdm
{
    RSVI::RSVI(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, number num_samples, double max_time, std::string name)
        : DFSVI(world, value_function, error, max_time, name), num_samples(num_samples)
    {
    }

    //  SELECT ACTION IN PERSEUS
    std::shared_ptr<Space> RSVI::selectActions(const std::shared_ptr<State> &state, number t)
    {
        throw exception::NotImplementedException();
        // return std::make_shared<DiscreteSpace>(getWorld()->getActionSpaceAt(state, t)->sample(num_samples), false);
    }

    //  SELECT ACTION IN PERSEUS
    std::shared_ptr<Space> RSVI::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        throw exception::NotImplementedException();
        // return std::make_shared<DiscreteSpace>(getWorld()->getNextObservationDistribution(state, action, t)->sample(num_samples), false);
    }

    // COMPUTE NEXT STATE IN PERSEUS
    std::shared_ptr<Space> RSVI::selectNextStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return DFSVI::selectNextStates(state, action, observation, t);
    }

    std::string RSVI::getAlgorithmName()
    {
        return "RandomSearchValueIteration";
    }

}
