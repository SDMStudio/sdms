

#define SDMS_REGISTRY(name_namespace)     namespace sdm { namespace name_namespace { registry::map_type registry::container = {

#define SDMS_END_REGISTRY()      }; } }

#define SDMS_REGISTER(name, class)     {name, &createInstance<class>},
