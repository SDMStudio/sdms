#pragma once

namespace sdm
{
    namespace config
    {
        /** Precision used to compare elements */
        const double PRECISION_SDMS_VECTOR = 1e-9;
        const double PRECISION_MAPPED_VECTOR = 0.00001;
        const double PRECISION_VECTO_INTERFACE = PRECISION_MAPPED_VECTOR;

        const double PRECISION_BELIEF = 0.0001;
        const double PRECISION_OCCUPANCY_STATE = 0.0001;

        /** Number of decimal to display */
        const unsigned short BELIEF_DECIMAL_PRINT = 5;
        const unsigned short OCCUPANCY_DECIMAL_PRINT = 5;
        const unsigned short VALUE_DECIMAL_PRINT = 4;

    } // namespace config

}