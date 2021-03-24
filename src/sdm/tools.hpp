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
    /**
     * @brief Namespace grouping other tools.
     */
    namespace tools
    {
        std::string getPathTo(std::string base, std::string world_name, std::string formalism_name);

        /**
         * @brief Compare the extension of a file. 
         * 
         * @param filename the filename
         * @param extension the extension
         * @return true if the filaname has the extension 'extension'
         * @return false if the filaname does not have the extension 'extension'
         */
        bool hasExtension(std::string filename, std::string extension);

        /**
         * @brief Add indentation to input string
         * 
         * @param s input string
         * @param num_indents the number of indentations
         * @param indent the indentation format used
         */
        std::string addIndent(std::string s, int num_indents, std::string indent = "\t");

        /**
         * @brief Add indentation to an output stream
         */
        void indentedOutput(std::ostream &outStream, const char *message);

        /**
         * @brief Repeat a string n times.
         * 
         * @param str the substring to replicate.
         * @param times the number of replication
         * @return the built string 
         */
        std::string repeatString(const std::string &str, size_t times);

    } // namespace tools
} // namespace sdm
