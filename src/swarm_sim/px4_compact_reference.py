"""Bounded compact-aircraft spatial reference; no native safety transfer."""
import hashlib
import json
import numpy as np
from .trajectory2d import Config,Snapshot,plan_mission
from .trajectory2d.planning import mission_document


def native_altitude_fields(has_position,absolute_altitude,home_altitude,world_reference):
    if not has_position:return 2,0.
    # A world-ground landing may be below the measured home antenna height.
    # Preserve its absolute altitude, rather than uploading a negative relative
    # altitude or clamping/moving the original geometry.
    return (5,float(absolute_altitude)) if world_reference else (6,float(absolute_altitude-home_altitude))


def generate_compact_reference(profile):
    config=profile.apply(Config(target=(1800.,0.),target_altitude=80.,horizon=100.,
        activation_time=0.,min_extra_loiter=0.,max_extra_loiter=0.,candidates=2,attempts=12,seed=7))
    snapshots=(Snapshot(0,(0.,0.,0.,0.),profile.airspeed_for('cruise'),80.,80.),)
    result=plan_mission(snapshots,config)
    if not result.validated:raise ValueError('compact native reference planning failed: '+result.status)
    return mission_document(result,config,snapshots,aircraft_profile=profile)


def checked_reference(document,profile):
    """Strictly bounded first adapter; preserve original plan and full horizon."""
    record=dict(document);expected=record.pop('sha256')
    if hashlib.sha256(json.dumps(record,sort_keys=True,allow_nan=False).encode()).hexdigest()!=expected:
        raise ValueError('compact reference fingerprint mismatch')
    if (record['status']!='validated' or record['config']['numerical_backend']!='cpp'
            or record['aircraft_profile']!=profile.record() or len(record['plans'])!=1):
        raise ValueError('validated own-profile native single-aircraft reference required')
    points=np.asarray(record['plans'][0]['flight_states'],dtype=float)
    if (points.ndim!=2 or points.shape[1]!=7 or len(points)<2 or not np.all(np.isfinite(points))
            or not np.allclose(points[0],[0,0,80,profile.airspeed_for('cruise'),0,0,0],atol=1e-9,rtol=0)
            or np.max(np.abs(points[:,2]-80))>.1 or np.max(np.abs(points[:,1]))>.1
            or np.max(np.abs(points[:,3]-profile.airspeed_for('cruise')))>.01
            or np.any(np.diff(points[:,0])<=0) or np.max(np.linalg.norm(points[:,:3],axis=1))>5000
            or tuple(record['config']['target'])!=(1800.,0.) or record['config']['target_altitude']!=80.
            or record['config']['obstacles']):
        raise ValueError('compact reference requires the declared straight level cruise fixture')
    return points[:,:3].copy()
