#include "sys/types.h"
#include "sys/sysinfo.h"
#include <sdm/parser/parser.hpp>

using namespace sdm;

long RanMemoryUsed(struct sysinfo memInfo)
{
    sysinfo (&memInfo);

    long long totalPhysMem = memInfo.totalram;
    //Multiply in next statement to avoid int overflow on right hand side...
    totalPhysMem *= memInfo.mem_unit;

    long long physMemUsed = memInfo.totalram - memInfo.freeram;
    //Multiply in next statement to avoid int overflow on right hand side...
    physMemUsed *= memInfo.mem_unit;

    std::cout<<"Total Ran Mem "<<totalPhysMem<<std::endl;
    std::cout<<"Ran Memory used  "<<physMemUsed<<std::endl;

    std::cout<<"Ratio : "<<100*physMemUsed/totalPhysMem<<std::endl;

    return physMemUsed;
}

int main(int argc, char **argv)
{
    
    struct sysinfo memInfo;

    auto start_memory = RanMemoryUsed(memInfo);

    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/Mars.dpomdp");

    auto end_memory = RanMemoryUsed(memInfo);

    std::cout<<"Used Memory : "<<end_memory-start_memory<<std::endl;

}