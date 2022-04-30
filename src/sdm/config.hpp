#pragma once

#include <string>

namespace sdm
{
    /**
     * @brief Namespace grouping a set of configurations.
     * 
     */
    namespace config
    {
        
        // ----------------------------------------------------
        // -------------- PRECISION PARAMETERS ----------------
        // ----------------------------------------------------

        /** @brief Precision used to compare SDMS vectors */
        const double PRECISION_SDMS_VECTOR = 0.0001;

        /** @brief Precision used to compare Mapped vectors */
        const double PRECISION_MAPPED_VECTOR = 0.0001;
        
        /** @brief Precision used to compare vectors interface */
        const double PRECISION_VECTO_INTERFACE = PRECISION_MAPPED_VECTOR;

        /** @brief Precision used to compare beliefs */
        const double PRECISION_BELIEF = 0.001;
        
        /** @brief Precision used to compare occupancy states */
        const double PRECISION_OCCUPANCY_STATE = 0.001;
        
        /** @brief Precision of the compression */
        const double PRECISION_COMPRESSION = 0.1;



        // --------------------------------------------------
        // ----------------- PATH PARAMETERS ----------------
        // --------------------------------------------------

        /** @brief Path to the directory where problems are stored. */
        const std::string INSTALL_PREFIX = "$HOME/.sdms";

        /** @brief Path to the directory where problems are stored. */
        const std::string PROBLEM_PATH = INSTALL_PREFIX + "/share/sdms/world/";

        /** @brief Path to the directory where configuration files are stored. */
        const std::string CONFIG_PATH = INSTALL_PREFIX + "/share/sdms/config/";

        /** @brief The name of the default world. */
        const std::string DEFAULT_WORLD = "mabc.dpomdp";

        // --------------------------------------------------
        // ---------------- THEME PARAMETERS ----------------
        // --------------------------------------------------

        /** @brief Color code for the main theme */
        const std::string SDMS_THEME_1 = "\033[1;36m";

        /** @brief Color code for the secondary theme */
        const std::string SDMS_THEME_2 = "\033[0;37m";

        /** @brief Color code for ***No Color*** */
        const std::string NO_COLOR = "\033[0m";

        /** @brief Custom parameter for logging */
        const std::string LOG_SDMS = SDMS_THEME_1 + "SDMS#>" + NO_COLOR + " ";


        // ----------------------------------------------------
        // ---------------- DISPLAY PARAMETERS ----------------
        // ----------------------------------------------------

        /** @brief Number of decimal used to display beliefs */
        const unsigned short BELIEF_DECIMAL_PRINT = 5;

        /** @brief Number of decimal used to display occupancy states */
        const unsigned short OCCUPANCY_DECIMAL_PRINT = 5;
        
        /** @brief Number of decimal used to display values */
        const unsigned short VALUE_DECIMAL_PRINT = 4;


    } // namespace config

}