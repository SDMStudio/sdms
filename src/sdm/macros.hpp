

#define SDMS_REGISTRY(name_namespace)     namespace sdm { namespace name_namespace { registry::map_type registry::container = {
#define SDMS_END_REGISTRY()      }; } }

#define SDMS_SUB_REGISTRY(name_namespace1, name_namespace2)     namespace sdm { namespace name_namespace1 { namespace name_namespace2 { registry::map_type registry::container = {
#define SDMS_END_SUB_REGISTRY()      }; } } }


#define SDMS_SUB_SUB_REGISTRY(name_namespace1, name_namespace2, name_namespace3)     namespace sdm { namespace name_namespace1 { namespace name_namespace2 { namespace name_namespace3 { registry::map_type registry::container = {
#define SDMS_END_SUB_SUB_REGISTRY()      }; } } } }

#define SDMS_REGISTER(name, class)     {name, &createInstance<class>},
