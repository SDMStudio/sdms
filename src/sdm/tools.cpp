#include <sdm/tools.hpp>

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