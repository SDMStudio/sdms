#include <sdm/world/posg.hpp>

namespace sdm
{

    POSG::POSG() {}

    POSG::POSG(const DecisionProcess &stochastic_game) : DecisionProcess(stochastic_game.getStateSpace(), stochastic_game.getAgentSpace(), stochastic_game.getActionSpace(), stochastic_game.getStartDistrib())
    {
    }

    // POSG::POSG(const std::string &filename) {}

    // POSG::POSG(number state_sp, number action_sp, const std::vector<double> & start_distrib) : SG(state_sp, action_sp, start_distrib) {}

    POSG::POSG(const DiscreteSpace &state_sp, const DiscreteSpace &agent_sp, const MultiDiscreteSpace &action_sp, const MultiDiscreteSpace &obs_sp,
               const StateDynamics &s_dyn, const ObservationDynamics &o_dyn, const std::vector<Reward> &rews, const Vector &start_distrib)
        : StochasticProcess(state_sp, start_distrib), DecisionProcess(state_sp, agent_sp, action_sp, s_dyn, rews, start_distrib), POProcess(state_sp, obs_sp),
          o_dynamics_(o_dyn) {}

    const ObservationDynamics& POSG::getObsDynamics() const
    {
        return this->o_dynamics_;
    }

    double POSG::getObservationProbability(number jaction, number jobservation, number state) const
    {
        return this->o_dynamics_.getObservationProbability(jaction, jobservation, state);
    }

    double POSG::getObservationProbability(std::vector<number> jaction, std::vector<number> jobservation, number state) const
    {
        return this->getObservationProbability(this->action_space_.joint2single(jaction), this->obs_spaces_.joint2single(jobservation), state);
    }

    const Matrix &POSG::getObservations(number jaction) const
    {
        return this->o_dynamics_.getObservations(jaction);
    }

    const Matrix &POSG::getObservations(std::vector<number> jaction) const
    {
        return this->getObservations(this->action_space_.joint2single(jaction));
    }

    double POSG::getDynamics(number cstate, number jaction, number jobservation, number nstate) const
    {
        return this->o_dynamics_.getDynamics(cstate, jaction, jobservation, nstate);
    }

    const Matrix &POSG::getDynamics(number jaction, number jobservation) const
    {
        return this->o_dynamics_.getDynamics(jaction, jobservation);
    }
} // namespace sdm