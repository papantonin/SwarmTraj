#include "planning_bridge.hpp"
#include "swarmtraj/mission.hpp"
#include <exception>
using namespace swarmtraj;

PyObject* swarm_plan_mission(PyObject*,PyObject* args) {
    PyObject *snapshots,*options,*parameters;
    if(!PyArg_ParseTuple(args,"OOO",&snapshots,&options,&parameters))return nullptr;
    try {
        if(!PyList_Check(snapshots)||!PyList_Check(options)||!PyList_Check(parameters)
            ||PyList_Size(snapshots)!=PyList_Size(options)||PyList_Size(snapshots)!=PyList_Size(parameters))throw std::invalid_argument("snapshot/own configuration mismatch");
        std::vector<Designation> states;std::vector<PlanningSettings> config;
        for(Py_ssize_t k=0;k<PyList_Size(snapshots);++k) {
            states.push_back(bridge::designation(PyList_GetItem(snapshots,k)));
            config.push_back(bridge::planning_settings(PyList_GetItem(options,k),bridge::physics(PyList_GetItem(parameters,k))));
        }
        MissionResult result;std::exception_ptr error;
        Py_BEGIN_ALLOW_THREADS
        try{result=plan_mission(std::move(states),std::move(config));}catch(...){error=std::current_exception();}
        Py_END_ALLOW_THREADS
        if(error)std::rethrow_exception(error);
        auto* out=bridge::python(object({{"status",result.status},{"reason",result.reason},{"counts",values(result.counts)},
            {"minimum_distance_lower_bound",result.minimum_distance?Value(*result.minimum_distance):Value()},
            {"required_distance",result.required_distance?Value(*result.required_distance):Value()},
            {"search_nodes",result.search_nodes},{"diagnostics",result.diagnostics},{"timings",result.timings}}));
        if(!out)return nullptr;
        auto* plans=PyList_New(result.plans.size());if(!plans){Py_DECREF(out);return nullptr;}
        for(std::size_t k=0;k<result.plans.size();++k){auto* item=bridge::python(result.plans[k]);if(!item){Py_DECREF(plans);Py_DECREF(out);return nullptr;}PyList_SET_ITEM(plans,k,item);}
        if(PyDict_SetItemString(out,"plans",plans)<0){Py_DECREF(plans);Py_DECREF(out);return nullptr;}Py_DECREF(plans);return out;
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const std::runtime_error& e){PyErr_Clear();PyErr_SetString(PyExc_RuntimeError,e.what());return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}

PyObject* swarm_approach_angles(PyObject*,PyObject* args) {
    PyObject *snapshots,*options,*parameters;
    if(!PyArg_ParseTuple(args,"OOO",&snapshots,&options,&parameters))return nullptr;
    try {
        if(!PyList_Check(snapshots)||!PyList_Check(options)||!PyList_Check(parameters)||PyList_Size(snapshots)!=PyList_Size(options)||PyList_Size(snapshots)!=PyList_Size(parameters))throw std::invalid_argument("snapshot/own configuration mismatch");
        std::vector<Designation> states;std::vector<PlanningSettings> config;
        for(Py_ssize_t k=0;k<PyList_Size(snapshots);++k){states.push_back(bridge::designation(PyList_GetItem(snapshots,k)));config.push_back(bridge::planning_settings(PyList_GetItem(options,k),bridge::physics(PyList_GetItem(parameters,k))));}
        Value::Object result;for(auto [id,angle]:approach_angles(states,config))result[std::to_string(id)]=angle?Value(*angle):Value();return bridge::python(result);
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
