#pragma once

#include <map>
#include <unordered_map>
#include <any>
#include <string>
#include <sstream>
#include <iostream>
#include <exception>
#include <fstream>
#include <sdm/utils/toml/tomlcpp.hpp>
#include <sdm/exception.hpp>

#define CHECK_AND_SET_ITEM(GetTYPE, VARIABLE)          \
    {                                                  \
        auto [okay, value] = table->GetTYPE(VARIABLE); \
        if (okay)                                      \
        {                                              \
            config.set(VARIABLE, value);               \
            continue;                                  \
        }                                              \
    }

namespace sdm
{
    class Config : public std::unordered_map<std::string, std::any>
    {
    public:
        using value_type = std::pair<const std::string, std::any>;
        using value_list_type = value_type;

        Config() {}
        Config(std::initializer_list<value_type> vals) : std::unordered_map<std::string, std::any>(vals)
        {
            for (const auto &pair_key_value : vals)
            {
                this->ordered_key_list.push_back(pair_key_value.first);
            }
        }

        Config(const std::string &toml_filename)
        {
            (*this) = Config::parseTOML(toml_filename);
        }

        template <typename T>
        T get(const std::string &key, T defaultValue) const
        {
            auto it = this->find(key);
            if (it == this->end())
                return defaultValue;

            return std::any_cast<T>(it->second);
        };

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
            const T *v = std::any_cast<T>(&it->second);
            if ((it != this->end()) && (v))
                return std::optional<T>(*v);
            else
                return std::nullopt;
        };

        template <typename T>
        void set(const std::string &key, T value)
        {
            this->ordered_key_list.push_back(key);
            (*this)[key] = value;
        };

        static Config parseTOML(const std::string &toml_filename)
        {
            auto res = toml::parseFile(toml_filename);
            auto res_parsing = toml::parseFile(toml_filename);
            auto table = res_parsing.table;
            if (!table)
                throw sdm::exception::ParsingException(res_parsing.errmsg);
            else
                return Config::parseTable(table);
        }

        std::string str(std::string name = "") const
        {
            std::ostringstream str_toml;
            std::string key;
            if (name != "")
                str_toml << "\n[" << name << "]" << std::endl;
            for (const auto &key : this->ordered_key_list)
            {
                auto opt_int = this->getOpt<int>(key);
                auto opt_double = this->getOpt<double>(key);
                auto opt_str = this->getOpt<std::string>(key);
                auto opt_bool = this->getOpt<bool>(key);
                auto opt_config = this->getOpt<Config>(key);
                if (opt_int.has_value())
                    str_toml << key << " = " << opt_int.value() << std::endl;
                if (opt_double.has_value())
                    str_toml << key << " = " << opt_double.value() << std::endl;
                if (opt_str.has_value())
                    str_toml << key << " = \"" << opt_str.value() << "\"" << std::endl;
                if (opt_bool.has_value())
                {
                    std::string boolean = opt_bool.value() ? "true" : "false";
                    str_toml << key << " = " << boolean << std::endl;
                }
                if (opt_config.has_value())
                {
                    if (name != "")
                        str_toml << opt_config.value().str(name + "." + key);
                    else
                        str_toml << opt_config.value().str(key);
                }
            }
            return str_toml.str();
        }

        int size()
        {
            return this->ordered_key_list.size();
        }

    protected:
        static Config parseTable(const std::shared_ptr<toml::Table> &table)
        {
            Config config;
            for (const auto &key : table->keys())
            {
                CHECK_AND_SET_ITEM(getString, key)
                CHECK_AND_SET_ITEM(getBool, key)
                CHECK_AND_SET_ITEM(getInt, key)
                CHECK_AND_SET_ITEM(getDouble, key)
                CHECK_AND_SET_ITEM(getTimestamp, key)

                auto sub_table = table->getTable(key);
                if (sub_table)
                {
                    config.set(key, Config::parseTable(sub_table));
                    continue;
                }
                throw sdm::exception::Exception("ParsingTable failed at " + key);
            }
            return config;
        }

        std::vector<std::string> ordered_key_list;
    };
}