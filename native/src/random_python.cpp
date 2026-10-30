#include "python_bridge.hpp"
#include "swarmtraj/random.hpp"
#include <limits>
using namespace swarmtraj;

PyObject* swarm_rng_probe(PyObject*,PyObject* args) {
    PyObject *entropy,*operations,*resume=Py_None;
    if(!PyArg_ParseTuple(args,"OO|O",&entropy,&operations,&resume))return nullptr;
    try {
        if(!PyList_Check(entropy)||!PyList_Check(operations)||PyList_Size(operations)>100000)throw std::invalid_argument("expected bounded seed/operation lists");
        std::vector<std::uint32_t> words;
        for(Py_ssize_t k=0;k<PyList_Size(entropy);++k) {
            auto v=bridge::integer(PyList_GetItem(entropy,k));
            if(v<0||v>0xffffffffLL)throw std::invalid_argument("invalid entropy word");words.push_back(static_cast<std::uint32_t>(v));
        }
        Pcg64 rng(words);
        if(resume!=Py_None) {
            if(!PyTuple_Check(resume)||PyTuple_Size(resume)!=6)throw std::invalid_argument("invalid RNG state");
            std::array<std::uint64_t,6> state{};
            for(std::size_t k=0;k<6;++k){state[k]=PyLong_AsUnsignedLongLong(PyTuple_GetItem(resume,k));if(PyErr_Occurred())throw std::invalid_argument("invalid RNG state word");}
            if(state[4]>1||state[5]>0xffffffffULL)throw std::invalid_argument("invalid uint32 cache");
            rng=Pcg64(RandomState{{state[0],state[1]},{state[2],state[3]},state[4]!=0,static_cast<std::uint32_t>(state[5])});
        }
        bridge::Owned results(PyList_New(PyList_Size(operations)));if(!results.object)return nullptr;
        for(Py_ssize_t k=0;k<PyList_Size(operations);++k) {
            auto* operation=PyList_GetItem(operations,k);
            if(!PyTuple_Check(operation)||!PyTuple_Size(operation))throw std::invalid_argument("invalid RNG operation");
            auto name=bridge::string(PyTuple_GetItem(operation,0));PyObject* value=nullptr;
            if(name=="raw"&&PyTuple_Size(operation)==1)value=PyLong_FromUnsignedLongLong(rng.raw());
            else if(name=="uniform"&&PyTuple_Size(operation)==3)value=PyFloat_FromDouble(rng.uniform(bridge::number(PyTuple_GetItem(operation,1)),bridge::number(PyTuple_GetItem(operation,2))));
            else if(name=="integer"&&PyTuple_Size(operation)==2) {
                auto bound=bridge::integer(PyTuple_GetItem(operation,1));
                if(bound<1)throw std::invalid_argument("empty integer range");
                value=PyLong_FromUnsignedLongLong(rng.integer(bound));
            } else throw std::invalid_argument("invalid RNG operation");
            if(!value)return nullptr;PyList_SET_ITEM(results.object,k,value);
        }
        const auto& s=rng.state();bridge::Owned state(Py_BuildValue("(KKKKiI)",
            static_cast<unsigned long long>(s.state.high),static_cast<unsigned long long>(s.state.low),
            static_cast<unsigned long long>(s.increment.high),static_cast<unsigned long long>(s.increment.low),static_cast<int>(s.has_uint32),s.cached_uint32));
        const auto seed=seed_sequence(words);bridge::Owned generated(Py_BuildValue("(KKKK)",
            static_cast<unsigned long long>(seed[0]),static_cast<unsigned long long>(seed[1]),static_cast<unsigned long long>(seed[2]),static_cast<unsigned long long>(seed[3])));
        if(!state.object||!generated.object)return nullptr;
        return Py_BuildValue("(NNN)",results.release(),state.release(),generated.release());
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
