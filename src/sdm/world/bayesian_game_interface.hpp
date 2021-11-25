#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/space/space.hpp>

namespace sdm
{

    class BayesianGameInterface
    {

    public:
        virtual ~BayesianGameInterface() {}

        virtual number getNumAgents() const = 0;

        virtual std::vector<int> getGameDimensions() = 0;

        virtual std::vector<int> getTypesNumbers() = 0;

        virtual float getPayoff(std::vector<std::shared_ptr<State>> types, std::vector<std::shared_ptr<Action>> actions, int idAgent) = 0;

        virtual float getJointTypesProba(std::vector<std::shared_ptr<State>> types) = 0;
    };

    class BayesianGameInterfaceTMP
    {
    public:
        virtual ~BayesianGameInterfaceTMP() {}
        virtual number getNumAgents() const = 0;

        // Dans TwoPlayerBayesianGame la structure de donnee pour cela est un MultiDiscreteSpace
        // composé de N espaces d'actions individuels (un pour chaque joueur)
        virtual std::shared_ptr<Space> getActionSpace() const = 0;

        // Dans TwoPlayerBayesianGame la structure de donnee pour cela est un MultiDiscreteSpace
        // composé de N espaces de types individuels (un pour chaque joueur)
        virtual std::shared_ptr<Space> getTypeSpace() const = 0;

        // Les MultiDiscreteSpace ci-dessus te génèrent des std:shared_ptr<Joint<std::shared_ptr<State>>>,
        // C'est grossomodo la même chose que des pointeurs sur std::vector<std::shared_ptr<State>> mais qui sont reconnus
        // comme des std::shared<State> (Joint<std::shared_ptr<State>> hérite de State).
        // C'est ces derniers qui seront passés en argument de tes fonctions
        virtual double getPayoff(std::shared_ptr<State> joint_type, std::shared_ptr<Action> joint_action, int idAgent) = 0;

        virtual double getJointTypesProba(std::shared_ptr<State> joint_type) = 0;
    };
}