#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
    class BeliefInterface : public State
    {
    public:
        virtual std::set<std::shared_ptr<State>> getStates() const = 0;

        virtual size_t size() const = 0;

        virtual double getProbability(const std::shared_ptr<State> &state) const = 0;

        virtual void setProbability(const std::shared_ptr<State> &state, double proba) = 0;

        virtual void addProbability(const std::shared_ptr<State> &, double proba) = 0;

        virtual bool operator==(const std::shared_ptr<BeliefInterface> &other) const = 0;

        virtual std::string str() const = 0;
    };
}