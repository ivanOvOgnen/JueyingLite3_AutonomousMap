#!/usr/bin/env python3
"""Robot-free UI demo: synthetic room, route planning, virtual arrival and STOP.

Only binds loopback. No ROS, SSH, robot Wi-Fi, SDK, UDP, or motor interface is
imported or opened. Synthetic bundles cannot pass the real backend map review.
"""
import argparse
import base64
import heapq
from http.cookies import SimpleCookie
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/lite3_odom_tf_adapter'))
from lite3_odom_tf_adapter.map_persistence import save_grid
from lite3_odom_tf_adapter.map_bundle import load_manifest, write_manifest
from lite3_odom_tf_adapter.map_identity import read_bundle
from lite3_odom_tf_adapter.runtime_state import RuntimeState


class Demo:
    def __init__(self, workspace):
        self.lock = threading.RLock()
        self.workspace = Path(workspace)
        self.maps = self.workspace/'maps'
        self.maps.mkdir(parents=True, exist_ok=True)
        # The virtual A* planner runs under the demo server's lock. Give its
        # synthetic session enough time on a loaded laptop; this does not
        # change the robot API's short, heartbeat-backed operator lease.
        self.runtime = RuntimeState(lease_seconds=5.0)
        self.width, self.height, self.resolution = 100, 80, .05
        self.cells = [100 if x in (0,99) or y in (0,79) or (48<=x<=51 and 25<=y<=55) else 0
                      for y in range(self.height) for x in range(self.width)]
        self.pose = [1.025, 1.025, 0.0]
        self.nav = self.mapping = self.opt_in = False
        self.stopped = True
        self.map_name = self.active_id = None
        self.preview = None
        self.goal_id, self.goal_state, self.goal_detail = None, 'idle', 'SIMULATION: no goal submitted'
        self.route, self.route_index = [], 0
        self.goal = None
        self.faults = set()
        self.saved = {}
        self.events = []
        self.save('demo_room')

    def save(self, name):
        if not isinstance(name,str) or not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
            raise ValueError('Use a new alphanumeric map name')
        q=NS(x=0.,y=0.,z=0.,w=1.)
        grid=NS(info=NS(width=self.width,height=self.height,resolution=self.resolution,
                        origin=NS(position=NS(x=0.,y=0.,z=0.),orientation=q)),data=self.cells,
                header=NS(frame_id='map',stamp=NS(sec=0,nanosec=0)))
        save_grid(grid,self.maps/name,provenance={'synthetic':True,'generator':'offline_navigation_demo'})
        manifest=load_manifest(self.maps/name)
        manifest['synthetic']=True
        manifest['quality']['notes']=['SYNTHETIC SIMULATION ONLY. Never authorize real robot navigation.']
        write_manifest(self.maps/name,manifest)
        self.saved[name]=read_bundle(self.maps/name)
        return {'ok':True,'name':name,'message':'Synthetic map saved; no robot was scanned.'}

    def clear(self,x,y):
        cx,cy=math.floor(x/self.resolution),math.floor(y/self.resolution)
        radius=math.ceil(.30/self.resolution)
        for dy in range(-radius,radius+1):
            for dx in range(-radius,radius+1):
                if dx*dx+dy*dy>radius*radius:continue
                ix,iy=cx+dx,cy+dy
                if not 0<=ix<self.width or not 0<=iy<self.height or self.cells[iy*self.width+ix]!=0:return False
        return True

    def path(self,x,y):
        if not self.clear(x,y):raise ValueError('Target is occupied, unknown or lacks clearance')
        start=(int(self.pose[0]/self.resolution),int(self.pose[1]/self.resolution))
        goal=(int(x/self.resolution),int(y/self.resolution))
        frontier=[(0,start)];came={start:None};cost={start:0}
        while frontier:
            _,node=heapq.heappop(frontier)
            if node==goal:break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt=(node[0]+dx,node[1]+dy)
                wx,wy=(nxt[0]+.5)*self.resolution,(nxt[1]+.5)*self.resolution
                if not self.clear(wx,wy):continue
                value=cost[node]+1
                if value<cost.get(nxt,float('inf')):
                    cost[nxt]=value;came[nxt]=node
                    heapq.heappush(frontier,(value+abs(nxt[0]-goal[0])+abs(nxt[1]-goal[1]),nxt))
        if goal not in came:raise ValueError('Target is in a disconnected free region')
        nodes=[];node=goal
        while node is not None:nodes.append(node);node=came[node]
        points=[[(ix+.5)*self.resolution,(iy+.5)*self.resolution] for ix,iy in reversed(nodes)]
        points[0]=self.pose[:2];points[-1]=[x,y]
        return points

    def stop(self,reason='Software STOP'):
        self.stopped,self.opt_in,self.nav=True,False,False
        self.runtime.reset(reason)
        self.preview=None
        self.goal_state='stopped';self.goal_detail='SIMULATION: '+reason
        self.route=[]
        self.events.append({'event':'stop','reason':reason,'time':time.monotonic()})

    def tick(self,dt=.05):
        with self.lock:
            if self.runtime.lease_owner is not None and not self.runtime.valid_lease():
                self.stop('Operator lease expired; explicit restart required');return
            if self.goal_state not in ('accepted','executing'):return
            if not self.runtime.valid_lease() or self.faults:
                self.stop('Lease/dependency lost; explicit restart required');return
            self.goal_state='executing'
            budget=.5*dt  # Virtual demo speed, unrelated to physical gait limits.
            while budget>0 and self.route_index<len(self.route):
                target=self.route[self.route_index]
                dx,dy=target[0]-self.pose[0],target[1]-self.pose[1]
                distance=math.hypot(dx,dy)
                if distance<=budget:
                    self.pose[:2]=target;budget-=distance;self.route_index+=1
                else:
                    self.pose[0]+=dx/distance*budget;self.pose[1]+=dy/distance*budget;budget=0
                if distance>0:self.pose[2]=math.atan2(dy,dx)
            if self.route_index==len(self.route):
                self.pose[2]=self.goal[2];self.goal_state='succeeded'
                self.goal_detail='SIMULATION: virtual target reached; no robot moved'
                self.events.append({'event':'virtual_arrival','goal_id':self.goal_id,'pose':list(self.pose)})
            else:self.goal_detail='SIMULATION: virtual robot following previewed route'

    def status(self):
        with self.lock:
            self.tick(0)
            ready=self.nav and not self.stopped and not self.faults
            components={name:{'state':'ready' if ready else 'not_ready','source':'simulation'}
                        for name in ('camera','scan','odometry','pointcloud','stitched_pointcloud','map','transforms','costmaps')}
            components['camera']['state']='ready' if 'camera' not in self.faults else 'not_ready'
            components['mapping']={'state':'ready' if self.mapping else 'not_ready','process_running':self.mapping,'paused':False}
            components['navigation_stack']={'state':'ready' if self.nav else 'not_ready','process_running':self.nav}
            components['localization']={'state':'ready' if ready else 'not_ready','converged':ready,'stable_samples':3 if ready else 0,
                                         'pose':dict(zip(('x','y','yaw'),self.pose)),'reason':'SIMULATED pose, not AMCL'}
            components['global_localization']={'state':'ready' if self.nav else 'not_ready','search_active':False,'refresh_ready':self.nav}
            lease=self.runtime.valid_lease()
            components['navigation']={'state':'ready' if ready and self.opt_in and lease else 'not_ready',
                'planner_ready':ready,'goal_requests_enabled':self.opt_in,'goal_requests_opt_in_available':ready,
                'active_map_name':self.map_name,'active_map_id':self.active_id,'session_epoch':self.runtime.epoch,
                'operator_lease_valid':lease,'active_goal_id':self.goal_id,'active_goal_state':self.goal_state,
                'active_goal_detail':self.goal_detail,'unresolved_goal_count':int(self.goal_state in ('accepted','executing')),
                'stop_latched':self.stopped,'reason':'SIMULATION ONLY: no ROS/Nav2 or robot connection',
                'path_preview':self.preview}
            components['motion_output']={'state':'not_ready','bridge_profile':'simulation','reason':'Virtual robot only; no motor interface exists'}
            components['safety_stop']={'state':'latched' if self.stopped else 'not_latched','cleanup_pending':False,'cleanup_error':None,
                                       'disable_results':{},'physical_estop_verified':False}
            return {'ok':True,'simulation':True,'api_contract_version':2,'api_paired':True,'control_mode':'simulation',
                    'capabilities':['priority_software_stop','tracked_goal_lifecycle','map_identity','path_preview','operator_lease'],
                    'components':components,'map':{'frame':'map','width':self.width,'height':self.height,'resolution':self.resolution}}

    def action(self,path,body,owner):
        with self.lock:
            if path=='/api/v1/safety/stop':self.stop();return {'ok':True,'message':'Virtual STOP latched','cleanup_pending':False}
            if path=='/api/v1/control/acquire':
                if not self.nav or self.stopped:raise ValueError('Start simulated localization first')
                return {'ok':True,'lease_token':self.runtime.acquire(owner),'session_epoch':self.runtime.epoch}
            if path=='/api/v1/control/heartbeat':
                self.runtime.renew(owner,body.get('lease_token'),body.get('session_epoch'));return {'ok':True}
            if path=='/api/v1/navigation/start':
                name=body.get('name')
                if name not in self.saved:raise ValueError('Select a synthetic demo map')
                self.runtime.reset('Explicit simulated startup');self.stopped=False;self.nav=True;self.mapping=False
                self.map_name,self.active_id=name,self.saved[name]['map_id'];self.preview=None;self.opt_in=False
                return {'ok':True,'message':'Simulated localization started; no robot action'}
            if path=='/api/v1/navigation/stop':self.stop('Navigation stopped');return {'ok':True}
            if path=='/api/v1/mapping/start':self.mapping=True;self.nav=False;return {'ok':True,'message':'Synthetic scan opened; no sensor data captured'}
            if path in ('/api/v1/maps/save','/api/v1/maps/finish'):
                result=self.save(body.get('name'));self.mapping=False;return result
            if path=='/api/v1/navigation/cancel':
                self.route=[];self.preview=None;self.goal_state='canceled'
                self.goal_detail='SIMULATION: virtual goal canceled; no robot moved'
                return {'ok':True,'message':'Virtual goal canceled'}
            if path.startswith('/api/v1/localization/'):
                if 'x' in body:
                    if not self.clear(body['x'],body['y']):raise ValueError('Start pose is not free')
                    self.pose=[body['x'],body['y'],body.get('yaw',0)]
                self.preview=None;return {'ok':True,'message':'Virtual pose updated'}
            if path=='/api/v1/diagnostics/preflight':return {'ok':True,'output':'SIMULATION ONLY. No ROS graph, sensor or motor acceptance.'}
            if path=='/api/v1/demo/fault':
                if body.get('active',True):self.faults.add(body['name'])
                else:self.faults.discard(body['name'])
                self.tick(0);return {'ok':True}
            if path=='/api/v1/navigation/preview':
                if body.get('map_id')!=self.active_id or body.get('session_epoch')!=self.runtime.epoch:raise ValueError('Map/session changed')
                x,y,yaw=body['x'],body['y'],body.get('yaw',0)
                if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (x,y,yaw)):raise ValueError('Invalid target')
                self.preview={'preview_id':secrets.token_hex(16),'map_id':self.active_id,'session_epoch':self.runtime.epoch,
                              'goal':[x,y,yaw],'start':self.pose[:2],'points':self.path(x,y),'created_monotonic':time.monotonic()}
                return {'ok':True,'path_preview':self.preview,'motion_command_sent':False}
            if path in ('/api/v1/navigation/goal-requests','/api/v1/navigation/goal'):
                if not self.runtime.valid_lease(owner,body.get('lease_token')):raise ValueError('Current operator lease required')
                if not self.nav or self.stopped or self.faults:raise ValueError('Simulated dependencies not ready')
                if path.endswith('goal-requests'):self.opt_in=body.get('enabled') is True;return {'ok':True,'message':'Simulation planner opt-in updated'}
                if not self.opt_in:raise ValueError('Enable planner requests first')
                if self.goal_state in ('accepted','executing'):raise ValueError('A goal is active')
                goal=[body['x'],body['y'],body.get('yaw',0)]
                if (not self.preview or self.preview['preview_id']!=body.get('preview_id') or
                        body.get('map_id')!=self.active_id or body.get('session_epoch')!=self.runtime.epoch or
                        goal!=self.preview['goal'] or time.monotonic()-self.preview['created_monotonic']>10):raise ValueError('Fresh matching route preview required')
                self.goal_id=secrets.token_hex(16);self.goal_state='accepted';self.goal=goal
                self.route=self.preview['points'];self.route_index=0
                self.preview=None
                return {'ok':True,'goal_id':self.goal_id,'message':'Virtual goal accepted (SIMULATION)','planner_only':True}
            raise ValueError('Unsupported demo operation')


def make_handler(demo):
    class Handler(SimpleHTTPRequestHandler):
        sessions=set()
        def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(ROOT/'webapp'),**kwargs)
        def log_message(self,*args):pass
        def reply(self,status,value,headers=None):
            payload=json.dumps(value,allow_nan=False).encode();self.send_response(status)
            self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(payload)))
            for key,value in (headers or {}).items():self.send_header(key,value)
            self.end_headers();self.wfile.write(payload)
        def do_GET(self):
            parsed=urlparse(self.path)
            if parsed.path=='/api/v1/session':
                token=secrets.token_urlsafe(24);self.sessions.add(token)
                return self.reply(200,{'ok':True,'authorized':True},{'Set-Cookie':f'demo_session={token}; HttpOnly; SameSite=Strict; Path=/'})
            if parsed.path=='/api/v1/status':return self.reply(200,demo.status())
            if parsed.path=='/api/v1/maps':
                return self.reply(200,{'ok':True,'maps':[{'name':name,'map_id':bundle['map_id'],'approval_state':'simulation_only',
                    'image_url':f'/maps/{name}.pgm','yaml_url':f'/maps/{name}.yaml'} for name,bundle in demo.saved.items()]})
            if parsed.path=='/api/v1/maps/bundle':
                query=parse_qs(parsed.query);bundle=demo.saved.get(query.get('name',[''])[0])
                if not bundle or bundle['map_id']!=query.get('map_id',[''])[0]:return self.reply(409,{'ok':False,'error':'Map changed'})
                return self.reply(200,{'ok':True,'name':bundle['name'],'map_id':bundle['map_id'],
                    'image_base64':base64.b64encode(bundle['_image']).decode(),'yaml_text':bundle['_yaml'].decode()})
            if parsed.path=='/api/v1/map/grid':return self.reply(200,{'ok':True,'width':demo.width,'height':demo.height,
                                                                   'resolution':demo.resolution,'data':demo.cells,'frame':'map'})
            if parsed.path.startswith('/api/'):return self.reply(404,{'ok':False,'error':'Unavailable in robot-free demo'})
            return super().do_GET()
        def do_POST(self):
            try:
                cookie=SimpleCookie(self.headers.get('Cookie',''));token=cookie.get('demo_session')
                if token is None or token.value not in self.sessions:return self.reply(401,{'ok':False,'error':'Demo session required'})
                origin=self.headers.get('Origin')
                if origin and urlparse(origin).netloc!=self.headers.get('Host'):return self.reply(403,{'ok':False,'error':'Same-origin request required'})
                length=int(self.headers.get('Content-Length','0'))
                if not 0<=length<=16384:raise ValueError('Request too large')
                body=json.loads(self.rfile.read(length)) if length else {}
                if not isinstance(body,dict):raise ValueError('Expected JSON object')
                return self.reply(200,demo.action(urlparse(self.path).path,body,token.value))
            except (ValueError,KeyError,TypeError,OSError) as error:return self.reply(409,{'ok':False,'error':str(error)})
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=18765)
    parser.add_argument('--workspace',type=Path)
    options=parser.parse_args()
    workspace=options.workspace or Path(tempfile.mkdtemp(prefix='lite3_simulation_'))
    demo=Demo(workspace)
    server=ThreadingHTTPServer(('127.0.0.1',options.port),make_handler(demo))
    running=threading.Event();running.set()
    def simulate():
        while running.wait(.05):
            demo.tick()
            time.sleep(.05)
    worker=threading.Thread(target=simulate,daemon=True);worker.start()
    print(f'SIMULATION ONLY: http://127.0.0.1:{options.port}/\nSynthetic evidence: {workspace}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        running.clear();demo.stop('Demo server exiting');server.server_close()
        (workspace/'simulation-events.json').write_text(json.dumps({'simulation':True,'events':demo.events},indent=2))


if __name__=='__main__':main()
