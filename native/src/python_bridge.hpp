#pragma once
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "swarmtraj/validation.hpp"
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <limits>

namespace swarmtraj::bridge {
inline PyObject* get(PyObject* d,const char* key) {
    if(!PyDict_Check(d))throw std::invalid_argument("expected record dictionary");
    auto* value=PyDict_GetItemString(d,key);
    if(!value)throw std::invalid_argument(std::string("missing record field: ")+key);
    return value;
}
inline double number(PyObject* obj,bool finite=true) {
    const double value=PyFloat_AsDouble(obj);
    if(PyErr_Occurred()||(finite&&!std::isfinite(value)))throw std::invalid_argument("expected finite numeric value");
    return value;
}
inline double real(PyObject* d,const char* key,bool finite=true) {return number(get(d,key),finite);}
inline long long integer(PyObject* obj) {
    if(!PyLong_Check(obj)||PyBool_Check(obj))throw std::invalid_argument("expected integer");
    auto value=PyLong_AsLongLong(obj);if(PyErr_Occurred())throw std::invalid_argument("integer out of range");return value;
}
inline std::size_t size(PyObject* obj) {
    const auto value=integer(obj);if(value<0)throw std::invalid_argument("negative size");return static_cast<std::size_t>(value);
}
inline bool boolean(PyObject* obj) {
    if(!PyBool_Check(obj))throw std::invalid_argument("expected boolean");return obj==Py_True;
}
inline std::string string(PyObject* obj) {
    if(!PyUnicode_Check(obj))throw std::invalid_argument("expected string");
    const char* value=PyUnicode_AsUTF8(obj);if(!value)throw std::invalid_argument("invalid string");return value;
}
inline PyObject* item(PyObject* sequence,Py_ssize_t index) {
    if(PyList_Check(sequence))return PyList_GetItem(sequence,index);
    if(PyTuple_Check(sequence))return PyTuple_GetItem(sequence,index);
    throw std::invalid_argument("expected list or tuple");
}
struct Owned {
    PyObject* object=nullptr;
    explicit Owned(PyObject* p):object(p){}
    ~Owned(){Py_XDECREF(object);}
    PyObject* release(){auto* p=object;object=nullptr;return p;}
};
struct Buffer {
    Py_buffer value{};
    explicit Buffer(PyObject* obj) {
        if(PyObject_GetBuffer(obj,&value,PyBUF_FORMAT|PyBUF_ND|PyBUF_STRIDES)<0)throw std::invalid_argument("expected float64 buffer");
        if(!value.format||std::strcmp(value.format,"d")||value.itemsize!=sizeof(double)||!PyBuffer_IsContiguous(&value,'C')) {
            PyBuffer_Release(&value);value.obj=nullptr;throw std::invalid_argument("expected contiguous native float64 buffer");
        }
    }
    ~Buffer(){if(value.obj)PyBuffer_Release(&value);}
};
template<std::size_t N> std::vector<std::array<double,N>> matrix(PyObject* obj,bool finite=true) {
    Buffer b(obj);
    if(b.value.ndim!=2||b.value.shape[1]!=N||b.value.shape[0]>1000001)throw std::invalid_argument("invalid matrix shape");
    std::vector<std::array<double,N>> result(b.value.shape[0]);
    if(!result.empty())std::memcpy(result.data(),b.value.buf,result.size()*sizeof(result[0]));
    if(finite)for(const auto& row:result)for(double v:row)if(!std::isfinite(v))throw std::invalid_argument("nonfinite buffer");
    return result;
}
template<std::size_t N> std::array<double,N> vector(PyObject* obj,bool finite=true) {
    Buffer b(obj);if(b.value.ndim!=1||b.value.shape[0]!=N)throw std::invalid_argument("invalid vector shape");
    std::array<double,N> result;std::memcpy(result.data(),b.value.buf,sizeof(result));
    if(finite)for(double v:result)if(!std::isfinite(v))throw std::invalid_argument("nonfinite buffer");return result;
}
inline std::vector<double> series(PyObject* obj,bool finite=true) {
    Buffer b(obj);if(b.value.ndim!=1||b.value.shape[0]>1000001)throw std::invalid_argument("invalid series shape");
    if(!b.value.shape[0])return {};
    const auto* first=static_cast<const double*>(b.value.buf);std::vector<double> result(first,first+b.value.shape[0]);
    if(finite)for(double v:result)if(!std::isfinite(v))throw std::invalid_argument("nonfinite buffer");return result;
}
inline std::optional<double> optional_number(PyObject* obj,bool finite=true) {return obj==Py_None?std::nullopt:std::optional<double>{number(obj,finite)};}
inline Parameters physics(PyObject* obj) {
    if(!PyTuple_Check(obj)||PyTuple_Size(obj)!=20)throw std::invalid_argument("expected 20 physical parameters");
    std::array<double,20> x;for(std::size_t k=0;k<20;++k)x[k]=number(PyTuple_GetItem(obj,k));
    Parameters p{x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],x[11],x[12],x[13],x[14],x[15],x[16],x[17],x[18],x[19]};validate(p);return p;
}
inline Candidate candidate(PyObject* d) {
    Candidate c;c.drone_id=integer(get(d,"drone_id"));c.candidate_id=integer(get(d,"candidate_id"));
    c.speed=real(d,"speed",false);c.dt=real(d,"dt",false);c.release=real(d,"release",false);c.arrival=real(d,"arrival",false);
    c.max_speed=real(d,"airspeed_limit",false);
    c.cruise=optional_number(get(d,"cruise"),false);c.approach=optional_number(get(d,"approach"),false);
    c.states=matrix<7>(get(d,"states"),false);c.controls=matrix<4>(get(d,"controls"),false);
    c.projected=matrix<6>(get(d,"projected"),false);c.acceleration=series(get(d,"acceleration"),false);
    c.vertical_acceleration=series(get(d,"vertical_acceleration"),false);c.levels=series(get(d,"levels"),false);
    auto* reference=get(d,"speed_reference");c.has_speed_reference=reference!=Py_None;
    if(c.has_speed_reference)c.speed_reference=series(reference,false);
    if(auto* family=PyDict_GetItemString(d,"family");family&&family!=Py_None)c.family=string(family);
    if(auto* routing=PyDict_GetItemString(d,"routing_mode"))c.routing_mode=string(routing);
    if(auto* phases=PyDict_GetItemString(d,"phases")) {
        if(!PyList_Check(phases)&&!PyTuple_Check(phases))throw std::invalid_argument("invalid phase list");
        for(Py_ssize_t k=0;k<PySequence_Size(phases);++k) {
            auto* phase=item(phases,k);
            if(!phase||!PyTuple_Check(phase)||PyTuple_Size(phase)!=3)throw std::invalid_argument("invalid phase");
            c.phases.push_back({string(PyTuple_GetItem(phase,0)),size(PyTuple_GetItem(phase,1)),size(PyTuple_GetItem(phase,2))});
        }
    }
    return c;
}
inline ValidationSettings settings(PyObject* d,const Parameters& p) {
    ValidationSettings c;c.physics=p;c.steps=size(get(d,"steps"));
    c.activation=real(d,"activation");c.min_delay=real(d,"min_delay");c.max_delay=real(d,"max_delay");c.latest_arrival=real(d,"latest_arrival");
    c.target=vector<2>(get(d,"target"));c.target_altitude=optional_number(get(d,"target_altitude"));
    c.target_radius=real(d,"target_radius");c.position_error=real(d,"position_error");c.descent_degrees=real(d,"descent_degrees");c.spread_degrees=real(d,"spread_degrees");
    auto* range=get(d,"cruise_range");if(range!=Py_None)c.cruise_range=vector<2>(range);
    c.cruise_levels=size(get(d,"cruise_levels"));c.smooth_speed=boolean(get(d,"smooth_speed"));
    c.speed_range=vector<2>(get(d,"speed_range"));c.speed_acceleration=real(d,"speed_acceleration");
    if(auto* option=PyDict_GetItemString(d,"operating_schedule"))c.operating_schedule=boolean(option);
    if(c.operating_schedule) {
        c.phase_speeds=vector<5>(get(d,"phase_speeds"));c.terminal_index=size(get(d,"terminal_index"));
        if(c.terminal_index<2||c.terminal_index>4||c.speed_acceleration<=0||c.smooth_speed||c.cruise_levels!=1)
            throw std::invalid_argument("invalid operating schedule settings");
        for(double speed:c.phase_speeds)if(speed<std::max(p.min_speed,p.stall_speed*p.stall_margin)||speed>p.max_speed)
            throw std::invalid_argument("operating speed outside physical envelope");
    }
    c.clock_error=real(d,"clock_error");c.aircraft_radius=real(d,"aircraft_radius");c.obstacle_clearance=real(d,"obstacle_clearance");
    auto solids=matrix<7>(get(d,"obstacles"));
    for(const auto& s:solids) {
        if((s[0]!=0&&s[0]!=1)||s[4]<=0||s[5]<=0||s[6]<=0||(s[0]==1&&s[4]!=s[5]))throw std::invalid_argument("invalid solid");
        c.obstacles.push_back({s[0]==1,{s[1],s[2],s[3]},{s[4],s[5],s[6]}});
    }
    if(!c.steps||c.steps>1000000||c.cruise_levels<1||c.cruise_levels>3||c.target_radius<=0||c.position_error<0
        ||c.target_radius<=c.position_error||c.clock_error<0||c.aircraft_radius<0||c.obstacle_clearance<0
        ||c.min_delay>c.max_delay||c.min_delay<0||c.activation<0
        ||(c.cruise_range&&c.cruise_range->at(0)>c.cruise_range->at(1)))throw std::invalid_argument("invalid validation settings");
    return c;
}
}
