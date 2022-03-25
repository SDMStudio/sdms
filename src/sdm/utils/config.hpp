#pragma once

#include <map>
#include <iomanip>
#include <unordered_map>
#include <any>
#include <string>
#include <sstream>
#include <iostream>
#include <exception>
#include <fstream>
#include <sdm/utils/toml/tomlcpp.hpp>
#include <sdm/exception.hpp>
#include <sdm/tools.hpp>

namespace sdm
{
    /**
     * @brief The standard configuration object used in SDMS.
     *
     * SDM'Studio is a high level library requiring low level
     * configuration of its algorithms. This class is a simple
     * maneer to maintain control over these parameters.
     *
     */
    class Config : public std::unordered_map<std::string, std::any>
    {
    public:
        using value_type = std::pair<const std::string, std::any>;
        using value_list_type = value_type;

        Config();
        Config(std::initializer_list<value_type> vals);
        Config(const std::string &toml_filename);

        /**
         * @brief Get the configuration value. If no such configuration is set,
         * return default value.
         *
         * This functions can be seen as equivalent to the function get
         * in python dictionnary.
         *
         * @tparam the configuration type
         * @param key the key
         * @param defaultValue the default value
         * @return the value of the configuration
         */
        template <typename T>
        T get(const std::string &key, T defaultValue) const
        {
            auto it = this->find(key);
            if (it == this->end())
                return defaultValue;

            return std::any_cast<T>(it->second);
        };

        /**
         * @brief Get the configuration value.
         *
         * The key must exists in the object,
         * otherwise an error will be raised.
         *
         * @tparam the configuration type
         * @param key the key
         * @return the value of the configuration
         */
        template <typename T = std::string>
        T get(const std::string &key) const
        {
            auto it = this->find(key);
            if (it == this->end())
                throw std::invalid_argument("Key (" + key + ") not found ");

            return std::any_cast<T>(it->second);
        };

        template <typename T = std::string>
        std::optional<T> getOpt(const std::string &key) const
        {
            auto it = this->find(key);
            if (it != this->end())
            {
                const T *v = std::any_cast<T>(&it->second);
                if (v)
                    return std::optional<T>(*v);
            }
            return std::nullopt;
        };

        /**
         * @brief Set a new configuration.
         *
         * A configuration can be seen as a pair (key/value).
         *
         * @tparam the configuration type
         * @param key the key
         * @param value the value of the configuration
         */
        template <typename T>
        void set(const std::string &key, T value)
        {
            if (std::find(this->ordered_key_list.begin(), this->ordered_key_list.end(), key) == this->ordered_key_list.end())
                this->ordered_key_list.push_back(key);
            (*this)[key] = value;
        };

        /**
         * @brief Parse a configuration file (TOML format)
         * into a configuration object.
         *
         * @param toml_filename the configuration file
         * @return the resulting configuration object
         */
        static Config parseTOML(const std::string &toml_filename);

        std::string str(std::string name = "") const;

        friend std::ostream &operator<<(std::ostream &os, const Config &config)
        {
            os << config.str();
            return os;
        }
        
    protected:
        /**
         * @brief Parse a toml::Table into a configuration object.
         *
         * @param table the toml::Table
         * @return the resulting configuration object
         */
        static Config parseTable(const std::shared_ptr<toml::Table> &table);

        std::vector<std::string> ordered_key_list;
    };
}