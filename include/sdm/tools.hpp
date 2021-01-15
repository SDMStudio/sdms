/**
 * @file tools.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief A set a tools for SDMS platform
 * @version 0.1
 * @date 14/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <string>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>

namespace sdm
{
    namespace tools
    {

        /**
         * @brief Add indentation to input string
         * 
         * @param s input string
         * @param num_indents the number of indentations
         * @param indent the indentation format used
         */
        std::string addIndent(std::string s, int num_indents, std::string indent = "\t");

    } // namespace tools
} // namespace sdm