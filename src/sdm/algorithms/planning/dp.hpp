#pragma once

#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

namespace sdm
{
    class DynamicProgramming : public Algorithm
    {
    public:
        /**
         * @brief Construct a dynamic programming algorithm.
         * 
         * @param world the world to be solved
         * @param error the error used by the stop criterion
         * @param name the name of the algorithm
         * 
         */
        DynamicProgramming(std::shared_ptr<SolvableByHSVI> world, double error, std::string name);

        /**
         * @brief Get the world.
         * 
         * This function returns a pointer on the world the algorithm is solving.
         */
        std::shared_ptr<SolvableByHSVI> getWorld() const;

        /**
         * @brief Get the number of trials.
         */
        number getTrial();

        /**
         * @brief Print starting informations on the output screen.
         */
        void printStartInfo();

        /**
         * @brief Print ending informations on the output screen.
         */
        void printEndInfo();

    protected:
        /**
         * @brief Initialize the logger
         */
        virtual void initLogger() = 0;

        /** @brief The problem to be solved */
        std::shared_ptr<SolvableByHSVI> world;

        /** @brief The allowed error */
        double error;

        /** @brief The number of trials */
        number trial;

        /** @brief The logger */
        std::shared_ptr<MultiLogger> logger;

    };
}