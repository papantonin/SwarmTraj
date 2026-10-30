#include "planning_bridge.hpp"
#include "swarmtraj/search.hpp"
#include <atomic>
#include <exception>
using namespace swarmtraj;
extern PyObject* swarm_reservation_error;
namespace {
struct SearchBox {
    JointSearch search;
    std::map<std::array<long long,2>,Candidate> plans;
    std::atomic_flag busy=ATOMIC_FLAG_INIT;
    SearchBox(PlanningSettings c,std::map<long long,PlanningSettings> own):search(std::move(c),std::move(own)){}
    const Candidate* store(PyObject* record) {
        auto candidate=bridge::candidate(record);const std::array<long long,2> key{candidate.drone_id,candidate.candidate_id};
        auto found=plans.find(key);
        if(found==plans.end())found=plans.emplace(key,std::move(candidate)).first;
        else if(candidate.states!=found->second.states||candidate.controls!=found->second.controls||candidate.projected!=found->second.projected
            ||candidate.dt!=found->second.dt||candidate.speed!=found->second.speed||candidate.release!=found->second.release||candidate.arrival!=found->second.arrival)
            throw std::invalid_argument("candidate changed after joint-search caching");
        return &found->second;
    }
    Domain domain(PyObject* records) {
        if(!PyList_Check(records))throw std::invalid_argument("expected candidate list");Domain result;
        for(Py_ssize_t k=0;k<PyList_Size(records);++k)result.push_back(store(PyList_GetItem(records,k)));return result;
    }
    Domains domains(PyObject* records) {
        if(!PyList_Check(records))throw std::invalid_argument("expected domain list");Domains result;
        for(Py_ssize_t k=0;k<PyList_Size(records);++k)result.push_back(domain(PyList_GetItem(records,k)));return result;
    }
};
SearchBox* box(PyObject* obj) {
    if(!PyCapsule_IsValid(obj,"swarmtraj.JointSearch"))throw std::invalid_argument("invalid native search handle");
    return static_cast<SearchBox*>(PyCapsule_GetPointer(obj,"swarmtraj.JointSearch"));
}
void destroy(PyObject* capsule){delete static_cast<SearchBox*>(PyCapsule_GetPointer(capsule,"swarmtraj.JointSearch"));}
struct Guard {
    SearchBox& box;
    explicit Guard(SearchBox& b):box(b){if(box.busy.test_and_set())throw std::invalid_argument("native search handle is already in use");}
    ~Guard(){box.busy.clear();}
};
Value keys(const Domain& plans){Value::Array result;for(auto* p:plans)result.push_back(array({p->drone_id,p->candidate_id}));return result;}
std::vector<std::size_t> indices(PyObject* obj) {
    if(!PyList_Check(obj))throw std::invalid_argument("expected index list");std::vector<std::size_t> result;
    for(Py_ssize_t k=0;k<PyList_Size(obj);++k)result.push_back(bridge::size(PyList_GetItem(obj,k)));return result;
}
std::vector<long long> ids(PyObject* obj) {
    if(!PyList_Check(obj))throw std::invalid_argument("expected ID list");std::vector<long long> result;
    for(Py_ssize_t k=0;k<PyList_Size(obj);++k)result.push_back(bridge::integer(PyList_GetItem(obj,k)));return result;
}
}

PyObject* swarm_search_new(PyObject*,PyObject* args) {
    PyObject *options,*parameters,*own;
    if(!PyArg_ParseTuple(args,"OOO",&options,&parameters,&own))return nullptr;
    try {
        const auto c=bridge::planning_settings(options,bridge::physics(parameters));std::map<long long,PlanningSettings> profiles;
        if(!PyList_Check(own))throw std::invalid_argument("expected own parameter list");
        for(Py_ssize_t k=0;k<PyList_Size(own);++k) {
            auto* item=PyList_GetItem(own,k);if(!PyTuple_Check(item)||PyTuple_Size(item)!=3)throw std::invalid_argument("invalid own parameters");
            const auto id=bridge::integer(PyTuple_GetItem(item,0));
            if(!profiles.emplace(id,bridge::planning_settings(PyTuple_GetItem(item,1),bridge::physics(PyTuple_GetItem(item,2)))).second)throw std::invalid_argument("duplicate own aircraft ID");
        }
        auto* result=new SearchBox(c,std::move(profiles));auto* capsule=PyCapsule_New(result,"swarmtraj.JointSearch",destroy);
        if(!capsule)delete result;return capsule;
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
PyObject* swarm_search_call(PyObject*,PyObject* args) {
    PyObject *handle,*data;const char* operation;
    if(!PyArg_ParseTuple(args,"OsO",&handle,&operation,&data))return nullptr;
    try {
        if(!PyDict_Check(data))throw std::invalid_argument("expected search input dictionary");
        auto& state=*box(handle);Guard guard(state);const std::string op(operation);
        Domain plans,incumbent;Domains domains;std::vector<Catalog> catalogs;std::vector<Designation> snapshots;
        std::vector<std::size_t> selected_indices;std::vector<long long> selected_ids;bool has_incumbent=false,mark=false;
        if(auto* records=PyDict_GetItemString(data,"plans"))plans=state.domain(records);
        if(auto* records=PyDict_GetItemString(data,"domains"))domains=state.domains(records);
        if(auto* records=PyDict_GetItemString(data,"incumbent");records&&records!=Py_None){incumbent=state.domain(records);has_incumbent=true;}
        if(auto* records=PyDict_GetItemString(data,"catalogs")) {
            if(!PyList_Check(records))throw std::invalid_argument("invalid catalog list");
            for(Py_ssize_t k=0;k<PyList_Size(records);++k) {
                auto* item=PyList_GetItem(records,k);Catalog c;c.attempted=bridge::size(bridge::get(item,"attempted"));
                const auto domain=state.domain(bridge::get(item,"plans"));for(auto* p:domain)c.plans.push_back(*p);catalogs.push_back(std::move(c));
            }
        }
        if(auto* records=PyDict_GetItemString(data,"snapshot_ids"))for(auto id:ids(records)){Designation s;s.drone_id=id;snapshots.push_back(s);}
        if(auto* records=PyDict_GetItemString(data,"indices"))selected_indices=indices(records);
        if(auto* records=PyDict_GetItemString(data,"drone_ids"))selected_ids=ids(records);
        if(op=="mark_exhausted")mark=bridge::boolean(bridge::get(data,"value"));
        if((op=="compatible"||op=="margin")&&plans.size()!=2)throw std::invalid_argument("pair operation requires two candidates");
        if((op=="expansion_indices"||op=="margin_expansion_indices")&&catalogs.size()!=snapshots.size())throw std::invalid_argument("catalog/ID mismatch");
        for(auto i:selected_indices)if(i>=catalogs.size())throw std::invalid_argument("catalog index out of bounds");
        Value result;std::exception_ptr error;
        Py_BEGIN_ALLOW_THREADS
        try {
            if(op=="solve"){auto answer=state.search.solve(domains);result=answer?keys(*answer):Value();}
            else if(op=="refine"){auto [answer,quality]=state.search.refine(domains,incumbent);result=object({{"plans",keys(answer)},{"quality",quality}});}
            else if(op=="compatible")result=state.search.compatible(*plans[0],*plans[1]);
            else if(op=="margin")result=state.search.margin(*plans[0],*plans[1]);
            else if(op=="score")result=values(state.search.score(plans));
            else if(op=="pair_scores")result=state.search.pair_scores(plans);
            else if(op=="optimization_available")result=state.search.optimization_available();
            else if(op=="expansion_indices")result=values(state.search.expansion_indices(catalogs,snapshots));
            else if(op=="margin_expansion_indices")result=values(state.search.margin_expansion_indices(catalogs,snapshots,plans));
            else if(op=="repair_hints") {
                Value::Object hints;
                for(const auto& [id,r]:state.search.repair_hints(selected_ids,has_incumbent?&incumbent:nullptr))hints[std::to_string(id)]=object({{"drone_id",id},{"conflict",witness_record(r.conflict)}});
                result=std::move(hints);
            } else if(op=="interval_hints") {
                Value::Object hints;
                for(const auto& [id,w]:state.search.interval_hints(catalogs,selected_indices,has_incumbent?&incumbent:nullptr))hints[std::to_string(id)]=window_record(w);
                result=std::move(hints);
            } else if(op=="diagnostics")result=state.search.diagnostics();
            else if(op=="verify_joint")result=state.search.verify(plans);
            else if(op=="mark_exhausted"){state.search.exhausted=mark;}
            else throw std::invalid_argument("unknown native search operation");
        }catch(...){error=std::current_exception();}
        Py_END_ALLOW_THREADS
        if(error)std::rethrow_exception(error);
        return bridge::python(object({{"value",result},{"metrics",object({{"nodes",state.search.nodes},{"exhausted",state.search.exhausted},
            {"optimization_nodes",state.search.optimization_nodes},{"optimization_budget_exhausted",state.search.optimization_budget_exhausted}})}}));
    }catch(const std::bad_alloc&){PyErr_NoMemory();return nullptr;}
     catch(const ReservationBudgetError& e){PyErr_Clear();PyErr_SetString(swarm_reservation_error,e.what());return nullptr;}
     catch(const std::exception& e){PyErr_Clear();PyErr_SetString(PyExc_ValueError,e.what());return nullptr;}
}
