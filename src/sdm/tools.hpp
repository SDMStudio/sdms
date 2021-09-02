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
     * @brief Namespace grouping different kind of tools.
     */
    namespace tools
    {
        /**
         * @brief Compare the extension of a file with a given extension. 
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
         * @brief Add indentation to an input string.
         * 
         * @param input_string the string 
         * @param num_indents the number of indentations
         * @param indent the indentation format used
         */
        std::string addIndent(std::string input_string, int num_indents = 1, std::string indent = "\t");

        /**
         * @brief Repeat a string n times.
         * 
         * @param str the substring to replicate.
         * @param times the number of replication
         * @return the built string 
         * 
         * Example:
         * 
         * ```cpp
         * std::cout << repeatString("bla", 3) << std::endl;
         * // OUTPUT : blablabla
         * ```
         */
        std::string repeatString(const std::string &str, size_t times);

        /**
         * @brief Concatenate strings in order to create the path to a specific problem.
         * 
         * @param base the repository that contains problems definitions
         * @param world_name the name of the problem
         * @param formalism_name the formalism
         * @return std::string the complete path to the problem file
         * 
         * Exemple:
         * 
         * ```cpp
         * std::cout << getPathTo("/usr/local/share/sdms/world", "tiger", "pomdp") << std::endl;
         * // OUTPUT : /usr/local/share/sdms/world/pomdp/tiger.pomdp
         * ```
         * 
         */
        std::string getPathTo(std::string base, std::string world_name, std::string formalism_name);

        /**
         * @brief Copy values contained in a std::set into a std::vector.
         * 
         * @tparam T the type of items
         * @param set the set
         * @return the vector that contains items in the set
         */
        template <typename T>
        std::vector<T> set2vector(const std::set<T> &set)
        {
            return std::vector<T>(set.begin(), set.end());
        }

        /**
         * @brief Extract the keys contained in a map.
         * 
         * @tparam TKey the type of keys
         * @tparam TValue the type of values
         * @param input_map the map
         * @return the list of existing keys 
         * 
         */
        template <typename TKey, typename TValue>
        std::vector<TKey> extractKeys(const std::map<TKey, TValue> &input_map)
        {
            std::vector<TKey> retkey;
            for (auto const &element : input_map)
            {
                retkey.push_back(element.first);
            }
            return retkey;
        }

        /**
         * @brief Extract the values contained in a map.
         * 
         * @tparam TKey the type of keys
         * @tparam TValue the type of values
         * @param input_map the map
         * @return the list of existing values 
         * 
         */
        template <typename TKey, typename TValue>
        std::vector<TValue> extractValues(const std::map<TKey, TValue> &input_map)
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
