"""Convert the XGO-Duck MJCF (robot_walk.xml) + STL meshes into one compact JSON for three.js.

Each body keeps its pos/quat (MuJoCo wxyz) and hinge joint; each mesh is welded,
quantised to int16 inside its bounding box and stored as base64.
    ../../.venv/bin/python build/mjcf_to_json.py
"""
import base64
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
ROBOT = HERE.parents[1] / 'cad/ref/xgoduck_rl/src/mjlab_microduck/robot/xgoduck'
OUT = HERE.parent / 'assets' / 'duck_model.json'


def floats(s, n):
    return [float(v) for v in s.split()] if s else None


def pack_mesh(path):
    m = trimesh.load_mesh(path, process=True)
    m.merge_vertices()
    v = np.asarray(m.vertices, dtype=np.float64)
    f = np.asarray(m.faces, dtype=np.uint32)
    lo, hi = v.min(0), v.max(0)
    span = np.maximum(hi - lo, 1e-9)
    q = np.round((v - lo) / span * 65535 - 32768).astype(np.int16)
    idx = f.astype(np.uint16 if len(v) < 65536 else np.uint32)
    return {
        'min': lo.tolist(), 'span': span.tolist(), 'n': int(len(v)),
        'pos': base64.b64encode(q.tobytes()).decode(),
        'idx': base64.b64encode(idx.tobytes()).decode(), 'i32': bool(len(v) >= 65536),
    }, len(f)


def walk(el, meshes):
    body = {'name': el.get('name'), 'pos': floats(el.get('pos'), 3) or [0, 0, 0],
            'quat': floats(el.get('quat'), 4) or [1, 0, 0, 0], 'children': [], 'meshes': []}
    j = el.find('joint')
    if j is not None:
        body['joint'] = {'name': j.get('name'), 'axis': floats(j.get('axis'), 3),
                         'range': floats(j.get('range'), 2)}
    for g in el.findall('geom'):
        if g.get('class') == 'visual':
            body['meshes'].append(g.get('mesh'))
            meshes.add(g.get('mesh'))
    for c in el.findall('body'):
        body['children'].append(walk(c, meshes))
    return body


def main():
    root = ET.parse(ROBOT / 'robot_walk.xml').getroot()
    meshes = set()
    tree = walk(root.find('worldbody/body'), meshes)
    packed, tris = {}, 0
    for name in sorted(meshes):
        packed[name], n = pack_mesh(ROBOT / 'assets' / f'{name}.stl')
        tris += n
    OUT.write_text(json.dumps({'source': 'LuwuDynamics xgoduck_rl robot_walk.xml', 'root': tree, 'meshes': packed},
                              separators=(',', ':')))
    print(f'{len(packed)} meshes, {tris} triangles, {OUT.stat().st_size / 1e6:.2f} MB -> {OUT}')


if __name__ == '__main__':
    sys.exit(main())
