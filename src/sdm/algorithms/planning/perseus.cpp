
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/algorithms/planning/perseus.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
// #include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    Perseus::Perseus(std::shared_ptr<SolvableByHSVI> world, std::shared_ptr<ValueFunction> value_function, double error, number num_samples, double max_time, std::string name)
        : PBVI(world, value_function, error, max_time, name), num_samples(num_samples)
    {
    }

    //  SELECT ACTION IN PERSEUS
    std::shared_ptr<Space> Perseus::selectActions(const std::shared_ptr<State> &state, number t)
    {
        throw exception::NotImplementedException();
        // return std::make_shared<DiscreteSpace>(getWorld()->getActionSpaceAt(state, t)->sample(num_samples), false);
    }

    //  SELECT ACTION IN PERSEUS
    std::shared_ptr<Space> Perseus::selectObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        throw exception::NotImplementedException();
        // return std::make_shared<DiscreteSpace>(getWorld()->getNextObservationDistribution(state, action, t)->sample(num_samples), false);
    }

    // COMPUTE NEXT STATE IN PERSEUS
    std::shared_ptr<Space> Perseus::selectNextStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return PBVI::selectNextStates(state, action, observation, t);
    }

}
