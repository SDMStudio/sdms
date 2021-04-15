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

        std::string addIndent(std::string s, int num_indents, std::string indent)
        {
            std::ostringstream res;
            for (int i = 0; i < num_indents; i++)
            {
                res << indent;
            }
            res << s;
            return res.str();
        }

        void indentedOutput(std::ostream &outStream, const char *message)
        {
            bool newline;
            while (char cur = *message)
            {
                if (newline)
                {
                    outStream << "\t";
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