#include <sdm/tools.hpp>

namespace sdm
{

    namespace tools
    {
        std::string sdm::tools::addIndent(std::string s, int num_indents, std::string indent)
        {
            std::ostringstream res;
            for (int i = 0; i < num_indents; i++)
            {
                res << indent;
            }
            res << s;
            return res.str();
        }

        void sdm::tools::indentedOutput(std::ostream &outStream, const char *message)
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
    } // namespace tools
} // namespace sdm