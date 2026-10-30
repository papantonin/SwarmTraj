#include "planning_bridge.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
using namespace swarmtraj;

PyObject* swarm_physical_proposal(PyObject*,PyObject* args) {
    PyObject *snapshot,*options,*physics,*state,*angle,*hint,*departure,*hold;
    long long id;const char* mode;double extra;
    if(!PyArg_ParseTuple(args,"OOOLOOOsOOd",&snapshot,&options,&physics,&id,&state,&angle,
                         &hint,&mode,&departure,&hold,&extra))return nullptr;
    try {
        const auto s=bridge::designation(snapshot);
        if(!std::all_of(s.initial.begin(),s.initial.end(),[](double v){return std::isfinite(v);})
           ||!std::isfinite(s.yaw_rate)||s.initial[3]<=0.)throw std::invalid_argument("invalid proposal initial state");
        const auto c=bridge::planning_settings(options,bridge::physics(physics));
        Pcg64 rng(bridge::random_state(state));
        const auto approach=bridge::optional_number(angle);
        const auto release=bridge::optional_number(departure);
        const auto minimum_hold=bridge::optional_number(hold);
        if(id<0||!std::isfinite(extra)||extra<0.)throw std::invalid_argument("invalid proposal ID/clearance");
        std::optional<Repair> repair;
        if(hint!=Py_None)repair=bridge::repair(hint);
        // Copy all Python-owned input before releasing the GIL.
        const std::string repair_mode(mode);
        ProposalResult result;std::exception_ptr error;
        Py_BEGIN_ALLOW_THREADS
        try {result=physical_proposal(s,c,id,rng,approach,repair?&*repair:nullptr,
                    repair_mode,release,minimum_hold,extra);}catch(...){error=std::current_exception();}
        Py_END_ALLOW_THREADS
        if(error)std::rethrow_exception(error);
        auto* plan=result.plan?bridge::python(*result.plan):Py_NewRef(Py_None);
        auto* next=bridge::python(rng.state());
        auto* reason=PyUnicode_FromString(result.rejection.c_str());
        if(!plan||!next||!reason){Py_XDECREF(plan);Py_XDECREF(next);Py_XDECREF(reason);return nullptr;}
        auto* out=PyTuple_New(3);
        if(!out){Py_DECREF(plan);Py_DECREF(next);Py_DECREF(reason);return nullptr;}
        PyTuple_SET_ITEM(out,0,plan);PyTuple_SET_ITEM(out,1,next);PyTuple_SET_ITEM(out,2,reason);
        return out;
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}

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
