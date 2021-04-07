#pragma once

namespace sdm
{

    class Exploration
    {
        virtual void reset(int nb_timesteps) = 0;
        virtual void update(int nb_timesteps) = 0;
        // virtual void getActionFrom() = 0;
    };

    class EpsGreedy : public Exploration
    {
    };
} // namespace sdm