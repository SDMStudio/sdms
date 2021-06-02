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
#include <set>

#include <sdm/utils/struct/vector.hpp>

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
         * @brief Add indentation to an output stream
         */
        void indentedOutput(std::ostream &outStream, const char *message, int num_indent = 1);

        /**
         * @brief Add indentation to input string
         * 
         * @param s input string
         * @param num_indents the number of indentations
         * @param indent the indentation format used
         */
        // std::string addIndent(std::string s, int num_indents, std::string indent = "\t");
        std::string addIndent(std::string input_string, int num_indents = 1, std::string indent = "\t");

        /**
         * @brief Repeat a string n times.
         * 
         * @param str the substring to replicate.
         * @param times the number of replication
         * @return the built string 
         */
        std::string repeatString(const std::string &str, size_t times);

        template <typename T>
        std::vector<T> set2vector(const std::set<T> &set)
        {
            return std::vector<T>(set.begin(), set.end());
        }

        template <typename TKey, typename TValue>
        std::vector<TKey> extractKeys(const std::map<TKey, TValue>& input_map)
        {
            std::vector<TKey> retkey;
            for (auto const &element : input_map)
            {
                retkey.push_back(element.first);
            }
            return retkey;
        }

        template <typename TKey, typename TValue>
        std::vector<TValue> extractValues(const std::map<TKey, TValue>& input_map)
        {
            std::vector<TValue> retvalue;
            for (auto const &element : input_map)
            {
                retvalue.push_back(element.second);
            }
            return retvalue;
        }

    } // namespace tools
} // namespace sdm
