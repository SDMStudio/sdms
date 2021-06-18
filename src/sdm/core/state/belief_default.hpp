#pragma once

#include <sdm/core/state/belief_interface.hpp>

namespace sdm
{
    class BeliefDefault : public BeliefInterface
    {
    public:
        BeliefDefault();

        /**
         * @brief Default Belief 
         * 
         * @param double : default value
         * 
         */
        BeliefDefault(double);

        double getProbability(const std::shared_ptr<State> &state) const;
        void setDefaultValue(double); 
        double getDefaultValue() const; 

        std::vector<std::shared_ptr<State>> getStates() const;

        size_t size() const;

        void setProbability(const std::shared_ptr<State> &state, double proba);

        void addProbability(const std::shared_ptr<State> &, double proba);

        bool operator==(const std::shared_ptr<BeliefInterface> &other) const;
        double operator^(const std::shared_ptr<BeliefInterface> &other) const;

        std::string str() const;

    protected : 
        double default_value_;

    };
}