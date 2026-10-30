#include "python_bridge.hpp"
#include <exception>
using namespace swarmtraj;

PyObject* swarm_validate_candidate(PyObject*,PyObject* args) {
    PyObject *record,*options,*parameters,*snapshot,*cruise;double yaw;
    if(!PyArg_ParseTuple(args,"OOOOdO",&record,&options,&parameters,&snapshot,&yaw,&cruise))return nullptr;
    try {
        const auto p=bridge::physics(parameters);
        const auto path=bridge::candidate(record);const auto c=bridge::settings(options,p);
        const auto initial=bridge::vector<7>(snapshot);const auto requested=bridge::optional_number(cruise);
        if(!std::isfinite(yaw))throw std::invalid_argument("nonfinite initial yaw rate");
        ValidationResult result;std::exception_ptr error;
        Py_BEGIN_ALLOW_THREADS
        try {result=validate_candidate(path,initial,yaw,requested,c);}catch(...){error=std::current_exception();}
        Py_END_ALLOW_THREADS
        if(error)std::rethrow_exception(error);
        auto* time=result.contact_time?PyFloat_FromDouble(*result.contact_time):Py_NewRef(Py_None);
        return Py_BuildValue("(sN)",result.rejection.c_str(),time);
    } catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
      catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
