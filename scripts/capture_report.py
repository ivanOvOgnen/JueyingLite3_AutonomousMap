#!/usr/bin/env python3
"""Standalone offline HTML timing/TF report with input hashes and diagnostics.

Numerically evaluates interpolation/composition; never claims robot geometry,
calibration authority, stop behavior or physical acceptance from a recording.
"""
import argparse
import base64
import html
import io
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/lite3_odom_tf_adapter'))
from lite3_odom_tf_adapter.capture_tf import CaptureTf
from validate_mapping_capture import validate_capture


def build_report(capture,max_tf_gap=.1):
    root=Path(capture)
    report=validate_capture(root)
    transforms=CaptureTf(max_tf_gap)
    observations=[];odom=[];errors=[]
    try:
        with (root/'events.jsonl').open() as stream:
            for number,line in enumerate(stream,1):
                try:
                    row=json.loads(line);kind=row['kind']
                    if kind in ('tf','tf_static'):
                        for item in row['transforms']:transforms.add(item,static=kind=='tf_static')
                    elif kind in ('depth','scan'):
                        observations.append((kind,row['header']['stamp_ns'],row['header']['frame_id'],row['received_unix_ns']))
                    elif kind=='normalized_odom':
                        p=row['pose']['position'];odom.append([row['header']['stamp_ns'],p['x'],p['y']])
                except (ValueError,KeyError,TypeError) as error:errors.append(f'line {number}: {error}')
    except OSError as error:errors.append(str(error))
    coverage={kind:{'samples':0,'resolved':0,'missing_stamp_ns':[]} for kind in ('depth','scan')}
    for kind,stamp,frame,received in observations:
        coverage[kind]['samples']+=1
        try:transforms.lookup('map',frame,stamp);coverage[kind]['resolved']+=1
        except (ValueError,TypeError):coverage[kind]['missing_stamp_ns'].append(stamp)
    report['numerical_tf']={'max_interpolation_gap_seconds':max_tf_gap,'coverage':coverage,'conflicts':errors}
    report['unverified']=[item for item in report['unverified'] if not item.startswith('Actual acquisition-time TF')]
    report['unverified']+=['Physical extrinsics and source/TF publisher authority identity',
                           'Independent room geometry and moving-session scan projection accuracy']
    if errors or any(value['samples']==0 or value['samples']!=value['resolved'] for value in coverage.values()):
        report['result']='fail'
        report['failures'].append({'code':'numerical_tf','detail':'Missing acquisition-time transforms or conflicting frame data'})
    fig,axes=plt.subplots(2,1,figsize=(10,6),constrained_layout=True)
    for kind in ('depth','scan'):
        rows=[row for row in observations if row[0]==kind]
        if rows:
            start=rows[0][1]
            axes[0].plot([(row[1]-start)/1e9 for row in rows],[(row[3]-row[1])/1e9 for row in rows],'.',markersize=2,label=kind)
    axes[0].set(xlabel='Acquisition time from stream start (s)',ylabel='Receipt minus acquisition (s)',title='Recorded source age; intentional delays remain visible')
    if observations:axes[0].legend()
    if odom:
        array=np.array(odom);axes[1].plot(array[:,1]-array[0,1],array[:,2]-array[0,2],'.-')
    axes[1].set(xlabel='Normalized odometry x from first sample (m)',ylabel='y from first sample (m)',title='Recorded trajectory; physical stationary state is unverified')
    axes[1].axis('equal')
    output=io.BytesIO();fig.savefig(output,format='png',dpi=130);plt.close(fig)
    rendered=f'''<!doctype html><meta charset="utf-8"><title>Lite3 capture diagnostics</title>
<style>body{{max-width:1100px;margin:2em auto;font:16px system-ui}}pre{{white-space:pre-wrap;background:#f3f5f7;padding:1em}}img{{width:100%}}</style>
<h1>Capture diagnostics: {html.escape(root.name)}</h1>
<p><strong>Offline result: {report['result'].upper()}. Robot acceptance: NOT ESTABLISHED.</strong></p>
<p>This report evaluates recorded timing and numerical TF interpolation/composition. Validate calibration, geometry, publisher authority and robot safety separately.</p>
<img alt="Source ages and recorded odometry" src="data:image/png;base64,{base64.b64encode(output.getvalue()).decode()}">
<h2>Machine-readable evidence and remaining checks</h2><pre>{html.escape(json.dumps(report,indent=2,sort_keys=True))}</pre>'''
    return report,rendered


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-tf-gap',type=float,default=.1)
    options=parser.parse_args()
    if not 0<options.max_tf_gap<=1:parser.error('TF gap must be in (0, 1] seconds')
    report,rendered=build_report(options.capture,options.max_tf_gap)
    with options.output.open('x') as stream:stream.write(rendered)
    with options.output.with_suffix('.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(f"{report['result']}: {options.output} (physical acceptance remains unverified)")
    return 0 if report['result']=='pass' else 1


if __name__=='__main__':raise SystemExit(main())
