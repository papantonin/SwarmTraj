#pragma once
#include "python_bridge.hpp"
#include "swarmtraj/planning.hpp"

namespace swarmtraj::bridge {
inline Value value(PyObject* object) {
    if(object==Py_None)return {};
    if(PyBool_Check(object))return object==Py_True;
    if(PyLong_Check(object))return integer(object);
    if(PyFloat_Check(object))return number(object,false);
    if(PyUnicode_Check(object))return string(object);
    if(PyList_Check(object)||PyTuple_Check(object)) {
        Value::Array items;const auto n=PySequence_Size(object);
        for(Py_ssize_t k=0;k<n;++k){auto* item=PySequence_GetItem(object,k);if(!item)throw std::invalid_argument("invalid sequence");items.push_back(value(item));Py_DECREF(item);}return items;
    }
    if(PyDict_Check(object)) {
        Value::Object items;Py_ssize_t pos=0;PyObject *key,*item;
        while(PyDict_Next(object,&pos,&key,&item))items[string(key)]=value(item);return items;
    }
    throw std::invalid_argument("unsupported diagnostic value");
}
inline PyObject* python(const Value& value) {
    if(std::holds_alternative<std::monostate>(value.data))return Py_NewRef(Py_None);
    if(auto p=std::get_if<bool>(&value.data))return PyBool_FromLong(*p);
    if(auto p=std::get_if<long long>(&value.data))return PyLong_FromLongLong(*p);
    if(auto p=std::get_if<double>(&value.data))return PyFloat_FromDouble(*p);
    if(auto p=std::get_if<std::string>(&value.data))return PyUnicode_FromStringAndSize(p->data(),p->size());
    if(auto p=std::get_if<Value::Array>(&value.data)) {
        auto* out=PyList_New(p->size());if(!out)return nullptr;
        for(std::size_t k=0;k<p->size();++k){auto* item=python(p->at(k));if(!item){Py_DECREF(out);return nullptr;}PyList_SET_ITEM(out,k,item);}return out;
    }
    auto* out=PyDict_New();if(!out)return nullptr;
    for(const auto& [key,item]:std::get<Value::Object>(value.data)) {
        auto* obj=python(item);if(!obj||PyDict_SetItemString(out,key.c_str(),obj)<0){Py_XDECREF(obj);Py_DECREF(out);return nullptr;}Py_DECREF(obj);
    }
    return out;
}
inline PlanningSettings planning_settings(PyObject* d,const Parameters& p) {
    PlanningSettings c;static_cast<ValidationSettings&>(c)=settings(d,p);
    for(const auto& item:{std::pair{"candidates",&c.candidates}, {"attempts",&c.attempts},{"initial_candidates",&c.initial_candidates},
        {"max_nodes",&c.max_nodes},{"margin_nodes",&c.margin_nodes},{"margin_rounds",&c.margin_rounds},{"max_entries",&c.max_entries},{"max_constraints",&c.max_constraints}})*item.second=size(get(d,item.first));
    c.separation=real(d,"separation");c.detour=real(d,"detour");c.level_hold=real(d,"level_hold");c.cell_size=real(d,"cell_size");c.post_arrival=real(d,"post_arrival");c.deadline=optional_number(get(d,"deadline"));
    for(const auto& item:{std::pair{"spatial",&c.spatial},{"adaptive",&c.adaptive},{"mrv",&c.mrv},{"max_margin",&c.max_margin},{"witnesses",&c.witnesses},
        {"reservations",&c.reservations},{"windows",&c.windows},{"mixed",&c.mixed},{"rejection_feedback",&c.rejection_feedback},{"id_approach",&c.id_approach}})*item.second=boolean(get(d,item.first));
    auto* seed=get(d,"seed_words");if(!PyList_Check(seed)||PyList_Size(seed)>100000)throw std::invalid_argument("invalid seed words");c.seed.clear();
    for(Py_ssize_t k=0;k<PyList_Size(seed);++k){const auto word=integer(PyList_GetItem(seed,k));if(word<0||word>0xffffffffLL)throw std::invalid_argument("invalid seed word");c.seed.push_back(word);}
    if(c.seed.empty()||!c.candidates||!c.attempts||!c.initial_candidates||!c.max_nodes||!c.max_entries||!c.max_constraints||c.cell_size<=0||c.separation<0||c.detour<0||c.level_hold<0||c.post_arrival<0
        ||(c.cruise_levels>1&&!c.cruise_range)||(c.smooth_speed&&c.speed_range[0]>c.speed_range[1]))throw std::invalid_argument("invalid planner settings");
    return c;
}
inline Designation designation(PyObject* d) {
    Designation s;s.drone_id=integer(get(d,"drone_id"));s.initial=vector<7>(get(d,"initial"));s.yaw_rate=real(d,"yaw_rate");s.cruise=optional_number(get(d,"cruise"));
    if(s.drone_id<0)throw std::invalid_argument("negative drone ID");return s;
}
inline RandomState random_state(PyObject* obj) {
    if(!PyTuple_Check(obj)||PyTuple_Size(obj)!=6)throw std::invalid_argument("invalid PCG64 state");
    std::array<std::uint64_t,6> words{};
    for(std::size_t k=0;k<6;++k){words[k]=PyLong_AsUnsignedLongLong(PyTuple_GetItem(obj,k));if(PyErr_Occurred())throw std::invalid_argument("invalid PCG64 state word");}
    if(!(words[3]&1)||words[4]>1||words[5]>0xffffffffULL)throw std::invalid_argument("invalid PCG64 increment/cache");
    return {{words[0],words[1]},{words[2],words[3]},words[4]!=0,static_cast<std::uint32_t>(words[5])};
}
inline PyObject* python(const RandomState& r) {
    return Py_BuildValue("(KKKKiI)",static_cast<unsigned long long>(r.state.high),static_cast<unsigned long long>(r.state.low),
        static_cast<unsigned long long>(r.increment.high),static_cast<unsigned long long>(r.increment.low),static_cast<int>(r.has_uint32),r.cached_uint32);
}
inline std::map<std::string,std::size_t> counts(PyObject* d) {
    if(!PyDict_Check(d))throw std::invalid_argument("invalid counts dictionary");
    std::map<std::string,std::size_t> result;Py_ssize_t pos=0;PyObject *key,*v;
    while(PyDict_Next(d,&pos,&key,&v))result[string(key)]=size(v);return result;
}
inline Catalog catalog(PyObject* d) {
    Catalog c;c.attempted=size(get(d,"attempted"));c.profile_stage=size(get(d,"profile_stage"));c.profile_cursor=size(get(d,"profile_cursor"));c.profile_failures=size(get(d,"profile_failures"));
    c.generation_s=real(d,"generation_s");c.validation_s=real(d,"validation_s");c.minimum_horizon=optional_number(get(d,"minimum_horizon"));
    c.rejections=counts(get(d,"rejections"));c.details=counts(get(d,"details"));
    auto* rng=get(d,"rng");if(rng!=Py_None)c.rng=random_state(rng);
    auto* plans=get(d,"plans");if(!PyList_Check(plans))throw std::invalid_argument("invalid catalog plans");
    for(Py_ssize_t k=0;k<PyList_Size(plans);++k)c.plans.push_back(candidate(PyList_GetItem(plans,k)));
    for(const auto& item:{std::pair{"guided",&c.guided},{"interval_contexts",&c.interval_contexts},{"interval_proposals",&c.interval_proposals},
        {"profile_events",&c.profile_events},{"profile_proposals",&c.profile_proposals},{"adjustments",&c.adjustments}}) {
        auto v=value(get(d,item.first));if(!std::holds_alternative<Value::Array>(v.data))throw std::invalid_argument("invalid catalog diagnostics array");*item.second=std::get<Value::Array>(std::move(v.data));
    }
    return c;
}
inline PyObject* python(const Candidate& p) {
    auto metadata=object({{"drone_id",p.drone_id},{"candidate_id",p.candidate_id},{"speed",p.speed},{"dt",p.dt},{"release",p.release},{"arrival",p.arrival},
        {"cruise",p.cruise?Value(*p.cruise):Value()},{"approach",p.approach?Value(*p.approach):Value()},
        {"family",p.family.empty()?Value():Value(p.family)},{"routing_mode",p.routing_mode},{"levels",values(p.levels)}});
    auto* out=python(metadata);if(!out)return nullptr;
    auto put=[&](const char* name,const void* data,std::size_t bytes) {
        auto* value=PyBytes_FromStringAndSize(static_cast<const char*>(data),bytes);
        if(!value)return false;const bool ok=PyDict_SetItemString(out,name,value)==0;Py_DECREF(value);return ok;
    };
    if(!put("states",p.states.data(),p.states.size()*sizeof(State))||!put("controls",p.controls.data(),p.controls.size()*sizeof(Control))
        ||!put("projected",p.projected.data(),p.projected.size()*sizeof(Projection))||!put("acceleration",p.acceleration.data(),p.acceleration.size()*sizeof(double))
        ||!put("vertical_acceleration",p.vertical_acceleration.data(),p.vertical_acceleration.size()*sizeof(double))){Py_DECREF(out);return nullptr;}
    auto* speed=p.has_speed_reference?PyBytes_FromStringAndSize(reinterpret_cast<const char*>(p.speed_reference.data()),p.speed_reference.size()*sizeof(double)):Py_NewRef(Py_None);
    if(!speed||PyDict_SetItemString(out,"speed_reference",speed)<0){Py_XDECREF(speed);Py_DECREF(out);return nullptr;}Py_DECREF(speed);
    auto* phases=PyList_New(p.phases.size());if(!phases){Py_DECREF(out);return nullptr;}
    for(std::size_t k=0;k<p.phases.size();++k) {
        const auto& phase=p.phases[k];auto* item=Py_BuildValue("(snn)",phase.name.c_str(),static_cast<Py_ssize_t>(phase.start),static_cast<Py_ssize_t>(phase.end));
        if(!item){Py_DECREF(phases);Py_DECREF(out);return nullptr;}PyList_SET_ITEM(phases,k,item);
    }
    if(PyDict_SetItemString(out,"phases",phases)<0){Py_DECREF(phases);Py_DECREF(out);return nullptr;}Py_DECREF(phases);return out;
}
inline PyObject* python(const Catalog& c) {
    Value::Object rejections,details;for(const auto& [k,v]:c.rejections)rejections[k]=v;for(const auto& [k,v]:c.details)details[k]=v;
    auto* out=python(object({{"attempted",c.attempted},{"profile_stage",c.profile_stage},{"profile_cursor",c.profile_cursor},{"profile_failures",c.profile_failures},
        {"generation_s",c.generation_s},{"validation_s",c.validation_s},{"minimum_horizon",c.minimum_horizon?Value(*c.minimum_horizon):Value()},
        {"rejections",rejections},{"details",details},{"guided",c.guided},{"interval_contexts",c.interval_contexts},{"interval_proposals",c.interval_proposals},
        {"profile_events",c.profile_events},{"profile_proposals",c.profile_proposals},{"adjustments",c.adjustments}}));if(!out)return nullptr;
    auto* rng=c.rng?python(*c.rng):Py_NewRef(Py_None);
    if(!rng||PyDict_SetItemString(out,"rng",rng)<0){Py_XDECREF(rng);Py_DECREF(out);return nullptr;}Py_DECREF(rng);
    auto* plans=PyList_New(c.plans.size());if(!plans){Py_DECREF(out);return nullptr;}
    for(std::size_t k=0;k<c.plans.size();++k){auto* item=python(c.plans[k]);if(!item){Py_DECREF(plans);Py_DECREF(out);return nullptr;}PyList_SET_ITEM(plans,k,item);}
    if(PyDict_SetItemString(out,"plans",plans)<0){Py_DECREF(plans);Py_DECREF(out);return nullptr;}Py_DECREF(plans);return out;
}
inline Repair repair(PyObject* d) {
    Repair r;r.drone_id=integer(get(d,"drone_id"));auto* w=get(d,"conflict");
    auto ids=[&](const char* key,std::array<long long,2>& dest) {
        auto* items=get(w,key);if(PySequence_Size(items)!=2)throw std::invalid_argument("invalid witness IDs");
        for(int j=0;j<2;++j){auto* item=PySequence_GetItem(items,j);if(!item)throw std::invalid_argument("invalid witness ID");dest[j]=integer(item);Py_DECREF(item);}
    };
    ids("drone_ids",r.conflict.drones);ids("candidate_ids",r.conflict.candidates);
    const auto positions=matrix<3>(get(w,"positions"));if(positions.size()!=2)throw std::invalid_argument("invalid witness positions");
    r.conflict.positions={positions[0],positions[1]};
    r.conflict.time=real(w,"sample_time_s");r.conflict.lower=real(w,"lower_bound_m");r.conflict.required=real(w,"required_m");
    if(r.drone_id!=r.conflict.drones[0]&&r.drone_id!=r.conflict.drones[1])throw std::invalid_argument("witness does not contain drone");return r;
}
inline DepartureWindows windows(PyObject* d) {
    DepartureWindows w;w.parent=integer(get(d,"parent_candidate_id"));w.constraints=size(get(d,"constraints"));w.exhausted=boolean(get(d,"budget_exhausted"));
    w.intervals=matrix<2>(get(d,"intervals_s"));
    auto* refs=get(d,"reference_drone_ids");const auto n=PySequence_Size(refs);if(n<0)throw std::invalid_argument("invalid references");
    for(Py_ssize_t j=0;j<n;++j){auto* obj=PySequence_GetItem(refs,j);if(!obj)throw std::invalid_argument("invalid reference ID");w.references.push_back(integer(obj));Py_DECREF(obj);}
    return w;
}
}
