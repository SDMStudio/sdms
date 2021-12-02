#pragma once

#include <map>
#include <any>
#include <string>
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
    class Config : public std::map<std::string, std::any>
    {
    public:
        using value_type = std::pair<const std::string, std::any>;
        using value_list_type = value_type;

        Config() {}
        Config(std::initializer_list<value_type> vals) : std::map<std::string, std::any>(vals) {}
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

        template <typename T = const char *>
        T get(const std::string &key) const
        {
            return this->getOpt<T>(key).value();
        };

        template <typename T = const char *>
        std::optional<T> getOpt(const std::string &key) const
        {
            auto it = this->find(key);
            if (it == this->end())
                throw std::invalid_argument("Key (" + key + ") not found ");

            if (const T *v = std::any_cast<T>(&it->second))
                return std::optional<T>(*v);
            else
                return std::nullopt;
        };

        template <typename T>
        void set(const std::string &key, T value)
        {
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
    };
}