"""Native serial/spawn planning on immutable physical C++ campaign jobs.

Trials preserve seeds, limits and budgets. One worker uses the complete native
planner; multiple workers schedule native catalogs. No PX4 qualification.
"""
import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import signal
import subprocess
import sys
from time import perf_counter, process_time
import numpy as np
from benchmark_scaling_campaign import decode
from benchmark_planning import signature
from swarm_sim.trajectory2d import plan_mission
from swarm_sim.trajectory2d.native_backend import identity

ROOT=Path(__file__).resolve().parents[1]


def sources():
    paths=[*sorted((ROOT/'src/swarm_sim/trajectory2d').glob('*.py')),
           *sorted((ROOT/'native').rglob('*.cpp')), *sorted((ROOT/'native').rglob('*.hpp')),
           Path(__file__).resolve(),ROOT/'scripts/benchmark_scaling_campaign.py']
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def worker(output,index):
    manifest=json.loads((output/'manifest.json').read_text())
    if sources()!=manifest['sources']:raise RuntimeError('benchmark sources changed')
    trial=manifest['trials'][index]
    snapshots,c=decode(trial['job'])
    from swarm_sim.trajectory2d.production import require_production
    require_production(c)
    if trial['backend'] != 'cpp':
        raise ValueError('Python production benchmark workers are retired')
    c=replace(c,planning_workers=trial['workers'])
    if c.aircraft_profiles:raise ValueError('saved benchmark scope is the common-airframe campaign')
    cpu_start=process_time();start=perf_counter()
    result=plan_mission(snapshots,c);seconds=perf_counter()-start
    planning_process_cpu_s=process_time()-cpu_start
    plan_bytes=sum(a.nbytes for p in result.plans for a in vars(p).values() if isinstance(a,np.ndarray))
    metadata={k:trial[k] for k in ('id','backend','workers','repeat')}
    metadata.update(seconds=seconds,planning_process_cpu_s=planning_process_cpu_s,
        status=result.status,counts=result.counts,search_nodes=result.search_nodes,
        signature=signature(result),timings=result.timings,
        self_peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        largest_child_peak_rss_bytes=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss*1024,
        plan_array_bytes=plan_bytes)
    arrays={}
    for k,p in enumerate(result.plans):
        for name in ('flight_states','flight_controls','states','vertical_states'):
            arrays[f'{k}_{name}']=getattr(p,name)
    np.savez_compressed(output/f'{index}.npz',**arrays)
    (output/f'{index}.json').write_text(json.dumps(metadata,indent=2,allow_nan=False))
    if sources()!=manifest['sources']:raise RuntimeError('benchmark sources changed')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=ROOT/'results/scaling-campaign-20261028/manifest.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--jobs',nargs='+',default=['core-baseline-n3-s7','core-rich_strict-n3-s7','core-grid-n3-s7'])
    parser.add_argument('--workers',type=int,nargs='+',default=[1,2])
    parser.add_argument('--repeats',type=int,default=1)
    parser.add_argument('--worker',type=int,help=argparse.SUPPRESS)
    args=parser.parse_args();output=args.output.resolve()
    if args.worker is not None:return worker(output,args.worker)
    if args.repeats<1 or any(w<1 for w in args.workers):parser.error('repeats/workers must be positive')
    if output.exists():parser.error('output must be new')
    raw=json.loads(args.reference.read_text());lookup={j['id']:j for j in raw['jobs']}
    if len(set(args.jobs))!=len(args.jobs) or set(args.jobs)-lookup.keys():parser.error('unknown/duplicate job IDs')
    from swarm_sim.trajectory2d.production import require_production
    for identifier in args.jobs:
        require_production(decode(lookup[identifier])[1])
    output.mkdir(parents=True)
    trials=[dict(id=j,job=lookup[j],backend=b,workers=w,repeat=r) for j in args.jobs
            for r in range(args.repeats) for b in ('cpp',) for w in args.workers]
    manifest={'schema':'swarm-native-benchmark-v2','sources':sources(),'native':identity(),
        'reference_sha256':hashlib.sha256(args.reference.read_bytes()).hexdigest(),
        'python':platform.python_version(),'numpy':np.__version__,'cpu_info':subprocess.check_output(['lscpu'],text=True),
        'blas_threads':1,'timeout_s':raw['timeout_s'],'trials':trials,
        'scope':'complete physical planning; serial C++ reference; serialized trials; uncontrolled desktop; process CPU excludes spawn children'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    rows=[]
    for index,trial in enumerate(trials):
        print('START '+str(index)+' '+trial['id']+' '+trial['backend']+' '+str(trial['workers']),flush=True)
        with (output/f'{index}.log').open('x') as log:
            try:
                proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--output',str(output),'--worker',str(index)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                proc.wait(timeout=raw['timeout_s'])
                if proc.returncode:
                    row={k:trial[k] for k in ('id','backend','workers','repeat')};row.update(completion='worker_error',status=None,returncode=proc.returncode)
                else:
                    row=json.loads((output/f'{index}.json').read_text());row['completion']='complete'
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                row={k:trial[k] for k in ('id','backend','workers','repeat')};row.update(completion='timeout',status=None)
        rows.append(row)
        with (output/'journal.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
    comparisons=[];errors=[]
    for index,row in enumerate(rows):
        if row['completion']!='complete':errors.append(f'{index}: '+row['completion']);continue
        reference=next(i for i,r in enumerate(rows) if r['id']==row['id'] and r['backend']=='cpp' and r['workers']==args.workers[0] and r['repeat']==row['repeat'])
        old=rows[reference];error=0.
        if old['completion']!='complete':errors.append(f'{index}: serial C++ reference incomplete');continue
        if any(row[k]!=old.get(k) for k in ('status','counts','search_nodes')):errors.append(f'{index}: acceptance/search changed')
        if (output/f'{reference}.npz').exists():
            with np.load(output/f'{reference}.npz') as a,np.load(output/f'{index}.npz') as b:
                if set(a.files)!=set(b.files):errors.append(f'{index}: plan count changed')
                else:
                    for key in a.files:
                        if a[key].shape!=b[key].shape:errors.append(f'{index}: shape changed');continue
                        delta=float(np.max(np.abs(a[key]-b[key])))
                        error=max(error,delta)
                        if delta>1e-9:errors.append(f'{index}: {key} exceeds 1e-9')
        sibling=next(i for i,r in enumerate(rows) if r['id']==row['id'] and r['backend']==row['backend'] and r['workers']==args.workers[0] and r['repeat']==row['repeat'])
        if row['signature']!=rows[sibling].get('signature'):errors.append(f'{index}: serial/spawn mismatch')
        comparisons.append({'id':row['id'],'backend':row['backend'],'workers':row['workers'],
            'seconds':row['seconds'],'speed_ratio_to_cpp_serial':old.get('seconds',0)/row['seconds'],
            'max_array_difference':error,'exact_serial_signature':row['signature']==old.get('signature')})
    report={'comparisons':comparisons,'errors':errors,'all_trials_complete':all(r['completion']=='complete' for r in rows),
        'limits':['No guaranteed speedups','cProfile/RSS are not certified integration error bounds','RSS peaks include imports; child peak is largest child, not process-tree sum','No PX4 certificate transfer','Failed catalogs remain failures']}
    (output/'comparison.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return int(bool(errors))

if __name__=='__main__':raise SystemExit(main())
