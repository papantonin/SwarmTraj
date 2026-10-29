#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "swarmtraj/flight.hpp"
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <exception>
#include <limits>
#include <cmath>
#include "build_info.hpp"
using namespace swarmtraj;
namespace {
struct Buffer {
    Py_buffer view{};
    explicit Buffer(PyObject* object) {
        if(PyObject_GetBuffer(object,&view,PyBUF_FORMAT|PyBUF_ND|PyBUF_STRIDES)<0)throw std::invalid_argument("expected a float64 buffer");
        if(!view.format||std::strcmp(view.format,"d")||view.itemsize!=sizeof(double)||!PyBuffer_IsContiguous(&view,'C')) {
            PyBuffer_Release(&view);view.obj=nullptr;throw std::invalid_argument("expected a C-contiguous native float64 buffer (no implicit conversion)");
        }
    }
    ~Buffer(){if(view.obj)PyBuffer_Release(&view);}
    Buffer(const Buffer&)=delete;
};
double number(PyObject* object) {
    const double v=PyFloat_AsDouble(object);
    if(PyErr_Occurred()||!std::isfinite(v))throw std::invalid_argument("expected finite numeric inputs");
    return v;
}
double option(PyObject* d,const char* key,double fallback) {PyObject* v=PyDict_GetItemString(d,key);return v?number(v):fallback;}
std::size_t count(PyObject* d,const char* key,std::size_t fallback) {
    auto* v=PyDict_GetItemString(d,key);if(!v)return fallback;
    if(!PyLong_Check(v)||PyBool_Check(v))throw std::invalid_argument("expected integer size");
    const auto value=PyLong_AsLongLong(v);
    if(PyErr_Occurred()||value<0||value>1000000)throw std::invalid_argument("size outside 0..1000000");
    return static_cast<std::size_t>(value);
}
template<std::size_t N> std::array<double,N> vector(PyObject* object) {
    Buffer b(object);
    if(b.view.ndim!=1||b.view.shape[0]!=N)throw std::invalid_argument("invalid vector shape");
    std::array<double,N> out;std::memcpy(out.data(),b.view.buf,sizeof(out));
    for(auto v:out)if(!std::isfinite(v))throw std::invalid_argument("nonfinite buffer");
    return out;
}
template<std::size_t N> std::vector<std::array<double,N>> matrix(PyObject* object) {
    Buffer b(object);
    if(b.view.ndim!=2||b.view.shape[1]!=N||b.view.shape[0]>1000000)throw std::invalid_argument("invalid matrix shape");
    std::vector<std::array<double,N>> out(b.view.shape[0]);
    if(!out.empty())std::memcpy(out.data(),b.view.buf,out.size()*sizeof(out[0]));
    for(const auto& row:out)for(double v:row)if(!std::isfinite(v))throw std::invalid_argument("nonfinite buffer");
    return out;
}
std::vector<double> values(PyObject* d,const char* key) {
    auto* object=PyDict_GetItemString(d,key);if(!object)return {};
    Buffer b(object);if(b.view.ndim!=1||b.view.shape[0]>1000001)throw std::invalid_argument("invalid series shape");
    if(b.view.shape[0]==0)return {};
    const auto* data=static_cast<const double*>(b.view.buf);std::vector<double> out(data,data+b.view.shape[0]);
    for(double v:out)if(!std::isfinite(v))throw std::invalid_argument("nonfinite series");return out;
}
Parameters parameters(PyObject* obj) {
    if(!PyTuple_Check(obj)||PyTuple_Size(obj)!=20)throw std::invalid_argument("expected 20 physical parameters");
    double x[20];for(int i=0;i<20;++i)x[i]=number(PyTuple_GetItem(obj,i));
    Parameters p{x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],x[10],x[11],x[12],x[13],x[14],x[15],x[16],x[17],x[18],x[19]};validate(p);return p;
}
Recipe recipe(PyObject* d) {
    Recipe r{};r.steps=count(d,"steps",0);
    r.speed=option(d,"speed",12);r.yaw_rate=option(d,"yaw_rate",0);r.cruise=option(d,"cruise",80);r.release=option(d,"release",0);r.minimum_hold=option(d,"minimum_hold",3);r.speed_end=option(d,"speed_end",0);r.heading=option(d,"heading",0);r.angle=option(d,"angle",0);r.target_z=option(d,"target_z",80);r.target_radius=option(d,"target_radius",6);r.position_error=option(d,"position_error",2);r.latest_arrival=option(d,"latest_arrival",0);r.rich=option(d,"rich",0)!=0;
    r.levels=values(d,"levels");r.speed_reference=values(d,"speed_reference");r.speed_derivative=values(d,"speed_derivative");
    for(auto item:{std::pair{"gate",&r.gate},std::pair{"direction",&r.direction},std::pair{"normal",&r.normal},std::pair{"target",&r.target}}) {
        auto* v=PyDict_GetItemString(d,item.first);if(v)*item.second=vector<2>(v);
    }
    auto* points=PyDict_GetItemString(d,"waypoints");if(points)r.waypoints=matrix<2>(points);
    return r;
}
PyObject* path_result(const Path& path) {
    PyObject* states=PyBytes_FromStringAndSize(reinterpret_cast<const char*>(path.states.data()),path.states.size()*sizeof(State));
    PyObject* controls=PyBytes_FromStringAndSize(reinterpret_cast<const char*>(path.controls.data()),path.controls.size()*sizeof(Control));
    PyObject* phases=PyList_New(path.phases.size());
    if(!states||!controls||!phases){Py_XDECREF(states);Py_XDECREF(controls);Py_XDECREF(phases);return nullptr;}
    for(std::size_t i=0;i<path.phases.size();++i){const auto& p=path.phases[i];auto* v=Py_BuildValue("(snn)",p.name.c_str(),static_cast<Py_ssize_t>(p.start),static_cast<Py_ssize_t>(p.end));if(!v){Py_DECREF(states);Py_DECREF(controls);Py_DECREF(phases);return nullptr;}PyList_SET_ITEM(phases,i,v);}
    return Py_BuildValue("(NNNsd)",states,controls,phases,path.rejection.c_str(),path.arrival);
}
PyObject* evaluate(PyObject*,PyObject* args) {
    const char* op;PyObject *sobj,*uobj,*pobj,*options;
    if(!PyArg_ParseTuple(args,"sOOOO",&op,&sobj,&uobj,&pobj,&options))return nullptr;
    try {
        if(!PyDict_Check(options))throw std::invalid_argument("expected options dictionary");
        const auto p=parameters(pobj);const auto s=vector<7>(sobj);
        if(s[3]<=0)throw std::invalid_argument("initial airspeed must be positive");
        const std::string operation(op);auto r=recipe(options);
        std::array<double,3> wind{};auto* w=PyDict_GetItemString(options,"wind");if(w)wind=vector<3>(w);
        const double duration=option(options,"duration",p.dt),eps=option(options,"tolerance",1e-7);
        const auto subdivisions=count(options,"substeps",2);
        if(!subdivisions||subdivisions>1024||eps<0)throw std::invalid_argument("invalid subdivisions/tolerance");
        Control u{};std::vector<Control> commands;std::vector<Point> points;
        State ref{};
        if(operation=="rollout")commands=matrix<4>(uobj);
        else if(operation=="onward")points=matrix<2>(uobj);
        else if(operation=="guidance")ref=vector<7>(uobj);
        else if(operation=="rhs"||operation=="propagate"||operation=="checked"||operation=="envelope")u=vector<4>(uobj);
        else if(operation!="prefix"&&operation!="terminal"&&operation!="headroom"&&operation!="pullout")throw std::invalid_argument("unknown native operation");
        const bool check=option(options,"check",0)!=0;
        if(check&&(subdivisions!=2||wind!=std::array<double,3>{}))throw std::invalid_argument("checked nominal rollout requires two substeps and zero wind");
        const auto first=count(options,"first_step",0);
        const double yaw=option(options,"yaw",0), speed_ff=option(options,"speed_ff",0);
        if(operation=="prefix"||operation=="onward") {
            if(!r.steps||r.speed<=0||r.minimum_hold<0||r.release<0||r.target_radius<=0||r.position_error<0||r.levels.empty()||r.levels.size()>3||r.speed_reference.size()!=r.steps+1||r.speed_derivative.size()!=r.steps+1||first>r.steps|| (operation=="prefix"&&r.waypoints.empty()))throw std::invalid_argument("invalid native recipe");
        }
        if((operation=="terminal"||operation=="headroom")&&(!r.steps||r.speed<=0))throw std::invalid_argument("invalid proposal fixture");
        Path result;State out{};Control command{};bool ok=true;double scalar=0.;
        std::exception_ptr error;
        Py_BEGIN_ALLOW_THREADS
        try {
            if(operation=="rhs")out=derivative(s,u,p,wind);
            else if(operation=="propagate")out=propagate(s,u,duration,p,subdivisions,wind);
            else if(operation=="checked")ok=checked_step(s,u,p,out);
            else if(operation=="envelope")ok=envelope(s,u,p,eps);
            else if(operation=="rollout")result=rollout(s,commands,p,check,subdivisions,wind);
            else if(operation=="prefix")result=prefix(s,p,r);
            else if(operation=="onward")result=onward(s,p,r,first,points);
            else if(operation=="pullout")result=pullout(s,p,r.steps);
            else if(operation=="terminal")ok=terminal_distance(r.speed,r.cruise,p,r,scalar);
            else if(operation=="headroom")ok=headroom(r.speed,p,r,scalar);
            else if(operation=="guidance")command=guidance(s,ref,yaw,speed_ff,p);
        } catch(...) {error=std::current_exception();}
        Py_END_ALLOW_THREADS
        if(error)std::rethrow_exception(error);
        if(operation=="envelope")return PyBool_FromLong(ok);
        if((operation=="checked"||operation=="terminal"||operation=="headroom")&&!ok)Py_RETURN_NONE;
        if(operation=="terminal"||operation=="headroom")return PyFloat_FromDouble(scalar);
        if(operation=="rhs"||operation=="propagate"||operation=="checked")return PyBytes_FromStringAndSize(reinterpret_cast<const char*>(out.data()),sizeof(out));
        if(operation=="guidance")return PyBytes_FromStringAndSize(reinterpret_cast<const char*>(command.data()),sizeof(command));
        return path_result(result);
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
    catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
#ifdef _MSC_VER
#define SWARM_STRINGIFY_IMPL(x) #x
#define SWARM_STRINGIFY(x) SWARM_STRINGIFY_IMPL(x)
#define SWARM_COMPILER "MSVC " SWARM_STRINGIFY(_MSC_VER)
#else
#define SWARM_COMPILER __VERSION__
#endif
PyObject* info(PyObject*,PyObject*) {return Py_BuildValue("{s:i,s:s,s:s,s:s}","abi",1,"compiler",SWARM_COMPILER,"source_sha256",SWARM_SOURCE_SHA,"math","float64; strict FP; no fast-math or contraction");}
PyMethodDef methods[]={{"evaluate",evaluate,METH_VARARGS,"Checked native numerical dispatch; float64 contiguous buffers."},{"build_info",info,METH_NOARGS,"Native build identity."},{nullptr,nullptr,0,nullptr}};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_trajectory_native","Optional coordinated point-mass numerical core.",-1,methods};
}
PyMODINIT_FUNC PyInit__trajectory_native(){return PyModule_Create(&module);}
