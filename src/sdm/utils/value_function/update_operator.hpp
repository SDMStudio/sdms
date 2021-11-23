
//  ------------------------------------------------------------------------
// |            INCLUDE Update Operators interfaces                         |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

//  ------------------------------------------------------------------------
// |            INCLUDE Value Update Operators Implementations              |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/lb_tabular_update.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/pwlc_update.hpp>

//  ------------------------------------------------------------------------
// |            INCLUDE Q-Value Update Operators Implementations            |
//  ------------------------------------------------------------------------

#include <sdm/utils/value_function/update_operator/qupdate/tabular_qupdate.hpp>
#include <sdm/utils/value_function/update_operator/qupdate/pwlc_qupdate.hpp>


//  ------------------------------------------------------------------------
// |                        Add to the registry                             |
//  ------------------------------------------------------------------------
#include <sdm/macros.hpp>
#include <sdm/utils/value_function/update_operator/registry.hpp>

SDMS_REGISTRY(update)
SDMS_REGISTER("TabularUpdate", TabularUpdate)
SDMS_REGISTER("PWLCUpdate", PWLCUpdate)
SDMS_REGISTER("LBTabularUpdate", LowerBoundTabularUpdate)
SDMS_END_REGISTRY()
