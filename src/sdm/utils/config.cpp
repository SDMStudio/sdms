#include <sdm/utils/config.hpp>

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

    Config::Config() {}

    Config::Config(std::initializer_list<value_type> vals) : std::unordered_map<std::string, std::any>(vals)
    {
        for (const auto &pair_key_value : vals)
        {
            this->ordered_key_list.push_back(pair_key_value.first);
        }
    }

    Config::Config(const std::string &toml_filename)
    {
        (*this) = Config::parseTOML(toml_filename);
    }

    Config Config::parseTOML(const std::string &toml_filename)
    {
        auto res = toml::parseFile(toml_filename);
        auto res_parsing = toml::parseFile(toml_filename);
        auto table = res_parsing.table;
        if (!table)
            throw sdm::exception::ParsingException(res_parsing.errmsg);
        else
            return Config::parseTable(table);
    }

    std::string Config::str(std::string name) const
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
                str_toml << std::setprecision(10) << std::fixed << key << " = " << opt_double.value() << std::endl;
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

    Config Config::parseTable(const std::shared_ptr<toml::Table> &table)
    {
        Config config;
        for (const auto &key : table->keys())
        {
            CHECK_AND_SET_ITEM(getBool, key)
            CHECK_AND_SET_ITEM(getInt, key)
            CHECK_AND_SET_ITEM(getDouble, key)
            CHECK_AND_SET_ITEM(getTimestamp, key)

            // String case (we tell the difference between ".toml" files and other strings)
            auto [okay, value] = table->getString(key);
            if (okay)
            {
                if (tools::hasExtension(value, "toml"))
                    config.set(key, Config::parseTOML(config::CONFIG_PATH + value));
                else
                    config.set(key, value);
                continue;
            }

            // Sub table case
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

}