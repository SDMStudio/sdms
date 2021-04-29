#include <sdm/tools.hpp>
#include <regex>

namespace sdm
{

    namespace tools
    {
        std::string getPathTo(std::string base, std::string world_name, std::string formalism_name)
        {
            return base + "/" + formalism_name + "/" + world_name + "." + formalism_name;
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

        // std::string addIndent(std::string s, int num_indents, std::string indent)
        // {
        //     std::ostringstream res;
        //     for (int i = 0; i < num_indents; i++)
        //     {
        //         res << indent;
        //     }
        //     res << s;
        //     return res.str();
        // }

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