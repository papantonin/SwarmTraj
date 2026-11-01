"""Container-only own-model servo step experiment; no PX4 or flight commands."""
import os
from pathlib import Path
import subprocess
import time
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double


def main():
    if not Path('/.dockerenv').exists() or os.environ.get('SWARM_COMPACT_ACTUATOR_BENCH')!='1':
        raise RuntimeError('isolated actuator bench only')
    with Path('/bench/gazebo.log').open('w') as log:
        simulator=subprocess.Popen(['gz','sim','-r','-s','--iterations','6000','/bench/world.sdf'],stdout=log,stderr=log)
        try:
            node=Node()
            publishers=[node.advertise(f'/model/compact_5kg_prototype/servo_{k}',Double) for k in range(5)]
            deadline=time.monotonic()+20
            while not all(p.has_connections() for p in publishers):
                if simulator.poll() is not None or time.monotonic()>deadline:raise RuntimeError('own servo subscriptions unavailable')
                time.sleep(.05)
            start=time.monotonic()
            while simulator.poll() is None:
                values=(0,0,0,0,0) if time.monotonic()-start<.5 else (-.2,.2,.15,-.15,.35)
                for publisher,value in zip(publishers,values):
                    msg=Double();msg.data=value
                    if not publisher.publish(msg):raise RuntimeError('servo publication failed')
                if time.monotonic()-start>20:raise TimeoutError('actuator bench exceeded wall budget')
                time.sleep(.05)
            if simulator.returncode!=0:raise RuntimeError('Gazebo actuator step experiment failed')
        finally:
            if simulator.poll() is None:
                simulator.terminate()
                try:simulator.wait(timeout=5)
                except subprocess.TimeoutExpired:simulator.kill();simulator.wait(timeout=5)


if __name__=='__main__':main()
