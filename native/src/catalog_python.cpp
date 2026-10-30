#include "planning_bridge.hpp"
#include <exception>
using namespace swarmtraj;

PyObject* swarm_extend_catalog(PyObject*,PyObject* args) {
    PyObject *record,*snapshot,*options,*physics,*angle,*hint,*windows;unsigned long long goal;const char* expansion;
    if(!PyArg_ParseTuple(args,"OOOOOKOOs",&record,&snapshot,&options,&physics,&angle,&goal,&hint,&windows,&expansion))return nullptr;
    try {
        auto catalog=bridge::catalog(record);const auto s=bridge::designation(snapshot);const auto c=bridge::planning_settings(options,bridge::physics(physics));
        const auto approach=bridge::optional_number(angle);std::optional<Repair> repair;std::optional<DepartureWindows> temporal;
        if(hint!=Py_None)repair=bridge::repair(hint);if(windows!=Py_None)temporal=bridge::windows(windows);
        if(goal>c.candidates||catalog.attempted>c.attempts)throw std::invalid_argument("catalog budget mismatch");
        std::exception_ptr error;
        Py_BEGIN_ALLOW_THREADS
        try {extend_catalog(catalog,s,c,goal,approach,repair?&*repair:nullptr,temporal?&*temporal:nullptr,expansion);}catch(...){error=std::current_exception();}
        Py_END_ALLOW_THREADS
        if(error)std::rethrow_exception(error);
        return bridge::python(catalog);
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
