#include <iostream>
#include <cassert>
#include <sdm/common.hpp>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/core/multi_discrete_space.hpp>
#include <sdm/utils/decision_rules/joint.hpp>
#include <sdm/utils/decision_rules/variations.hpp>

int main(int argc, char **argv)
{

	// Construct DecPOMDP from filename (use parser)
	sdm::DecPOMDP dpomdp_1("../data/world/dpomdp/tiger.dpomdp");

	// Construct DecPOMDP using parser
	sdm::DecPOMDP dpomdp_2 = sdm::parser::parse_file("../data/world/dpomdp/mabc.dpomdp");

	// Copie DecPOMDP into another DecPOMDP
	sdm::DecPOMDP dpomdp_3;
	dpomdp_3 = dpomdp_2;

	// Construct ZSPOSG from DecPOMDP problem
	sdm::ZSPOSG zs_posg1(dpomdp_2);

	// Construct ZSPOSG from filename (use parser)
	// sdm::ZSPOSG zs_posg2("../data/world/dpomdp/mabc.zsposg");

	// Construct ZSPOSG using parser
	sdm::ZSPOSG zs_posg2 = sdm::parser::parse_file("../data/world/zsposg/fake_prisoners.zsposg");

	std::cout << "#> DPOMDP_1" << dpomdp_1 << "\n\n";
	std::cout << "#> DPOMDP_2" << dpomdp_2 << "\n\n";
	std::cout << "#> DPOMDP_3" << dpomdp_3 << "\n\n";

	std::cout << "#> ZSPOSG_1" << zs_posg << "\n\n";
	std::cout << "#> ZSPOSG_2" << zs_posg2 << "\n\n";

	return 0;
}
