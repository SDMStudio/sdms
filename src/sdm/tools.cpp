#include <sdm/tools.hpp>
#include <sys/stat.h>
#include <regex>

namespace sdm
{

    namespace tools
    {
        std::string getPathTo(std::string base_dir, std::string world_name, std::string formalism_name)
        {
            if (base_dir[base_dir.size()] != '/')
            {
                base_dir = base_dir + "/";
            }
            return base_dir + formalism_name + "/" + world_name + "." + formalism_name;
        }

        std::string getWorldPath(std::string path)
        {
            struct stat buffer;
            if (stat(path.c_str(), &buffer) == 0)
            {
                return path;
            }
            else
            {
                std::string formalism_name, world_name;
                std::size_t pos = path.find(".");
                if (pos != std::string::npos)
                {
                    world_name = path.substr(0, pos);
                    formalism_name = path.substr(pos + 1);
                }
                else
                {
                    for (char separator : {'/', ':', '-', ',', ';'})
                    {
                        std::size_t pos = path.find(separator);
                        if (pos != std::string::npos)
                        {
                            world_name = path.substr(pos + 1);
                            formalism_name = path.substr(0, pos);
                            break;
                        }
                    }
                }
                return getPathTo(config::PROBLEM_PATH, world_name, formalism_name);
            }
        }

        bool hasExtension(std::string filename, std::string extension)
        {
            return regex_match(filename, std::regex(".*\\" + extension + "$"));
        }

        void indentedOutput(std::ostream &outStream, const char *message, int num_indents)
        {
            bool newline = true;
            while (char cur = *message)
            {
                if (newline)
                {
                    for (int indent = 0; indent < num_indents; ++indent)
                    {
                        outStream << "\t";
                    }
                    newline = false;
                }
                outStream << cur;
                if (cur == '\n')
                {
                    newline = true;
                }
                ++message;
            }
        }

        std::string addIndent(std::string input_string, int num_indents, std::string)
        {
            std::ostringstream res;
            indentedOutput(res, input_string.c_str(), num_indents);
            return res.str();
        }

        std::string repeatString(const std::string &str, size_t times)
        {
            std::stringstream stream;
            for (size_t i = 0; i < times; i++)
            {
                stream << str;
            }
            return stream.str();
        }

    } // namespace tools
} // namespace sdm