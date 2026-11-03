"""Pure Python 2.7/3.x geometry, constraint and loading algorithms.

Node keys are (instance_name, node_label). Coordinates are assembly coordinates.
H and P are components of F-I and first Piola stress, respectively.
"""
from __future__ import division
import copy
import itertools
import math
import re

COMPONENTS = ('11', '22', '33', '12', '13', '23')


def default_config():
    return dict(model='Model-1', instances=[], tag='RVE', mode='3D',
                thickness='Z', tolerance=0.0, test='TENSION', source='UMAT',
                organization='MULTI', step_source='CREATE', nlgeom=True,
                cetol=1.e-5, previous='Initial',
                controls=dict((c, 'P') for c in COMPONENTS),
                stages=[stage('LoadingStep', 1.0)], report_directory='')


def stage(name, duration, kind='LOAD', targets=None):
    hold = kind == 'HOLD'
    return dict(name=name, duration=duration, kind=kind,
                targets=dict((c, float((targets or {}).get(c, 0.0)))
                             for c in COMPONENTS),
                initial=min(0.01, duration), minimum=min(1.e-5, duration),
                maximum=min(100.0 if hold else 0.02, duration),
                increments=100000 if hold else 10000, amplitude='')


def active_components(config):
    if config['mode'] == '3D':
        return COMPONENTS
    t = 'XYZ'.index(config['thickness']) + 1
    axes = [i for i in (1, 2, 3) if i != t]
    return ('11', '22', '33', '%d%d' % tuple(axes))


def preset(config, kind, axes='X', value1=0.0, value2=0.0, control='H',
           loading_time=1.0, hold_time=10000.0):
    cfg = copy.deepcopy(config)
    cfg['test'] = kind
    selected = ['XYZ'.index(a) + 1 for a in axes]
    cfg['controls'] = dict((c, 'H' if kind == 'SHEAR' else 'P')
                           for c in COMPONENTS)
    values = dict((c, 0.0) for c in COMPONENTS)
    if kind == 'SHEAR':
        if len(selected) != 2 or selected[0] >= selected[1]:
            raise ValueError('Simple shear requires XY, XZ or YZ.')
        component = '%d%d' % tuple(selected)
        if component not in active_components(cfg):
            raise ValueError('Thickness shear is not allowed in quasi-2D mode.')
        values[component] = value1
    else:
        if len(selected) not in (1, 2) or len(set(selected)) != len(selected):
            raise ValueError('Select one axis or two different axes.')
        for index, axis in enumerate(selected):
            component = '%d%d' % (axis, axis)
            mode = 'P' if kind == 'CREEP' else control
            if control == 'MIXED' and kind != 'CREEP':
                mode = 'P' if index == 0 else 'H'
            cfg['controls'][component] = mode
            values[component] = value1 if index == 0 else value2
    cfg['stages'] = [stage('LoadingStep', loading_time, targets=values)]
    if kind == 'CREEP':
        cfg['stages'].append(stage('HoldStep', hold_time, 'HOLD', values))
    return cfg


def finite(value, description):
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        raise ValueError('%s must be finite.' % description)
    return value


def validate_config(config):
    cfg = copy.deepcopy(config)
    if not re.match(r'^[A-Za-z][A-Za-z0-9_]{0,23}$', cfg['tag']):
        raise ValueError('Configuration tag: use 1-24 ASCII letters/digits/underscores, starting with a letter.')
    if cfg['mode'] not in ('3D', 'QUASI2D') or cfg['thickness'] not in ('X', 'Y', 'Z'):
        raise ValueError('Invalid mode or thickness axis.')
    if cfg['source'] not in ('UMAT', 'CREEP', 'ELASTIC'):
        raise ValueError('Source must be UMAT, CREEP or ELASTIC.')
    if cfg['test'] == 'CREEP' and cfg['source'] == 'ELASTIC':
        raise ValueError('Creep requires a time-dependent UMAT or built-in Creep material.')
    if cfg['organization'] not in ('SINGLE', 'MULTI') or cfg['step_source'] not in ('CREATE', 'EXISTING'):
        raise ValueError('Invalid step organization/source.')
    if cfg['organization'] == 'SINGLE' and any(s.get('amplitude') for s in cfg['stages']):
        raise ValueError('Single-step paths use generated tabular amplitudes; clear existing amplitude names.')
    cfg['tolerance'] = finite(cfg.get('tolerance', 0), 'Tolerance')
    cfg['cetol'] = finite(cfg['cetol'], 'CETOL')
    if cfg['tolerance'] < 0 or cfg['cetol'] <= 0:
        raise ValueError('Tolerance must be nonnegative; CETOL must be positive.')
    if not cfg['stages']:
        raise ValueError('At least one path stage is required.')
    names = set()
    previous = dict((c, 0.0) for c in COMPONENTS)
    for c in active_components(cfg):
        if cfg['controls'].get(c) not in ('H', 'P'):
            raise ValueError('Each active component must use H or P control.')
    for row in cfg['stages']:
        if not re.match(r'^[A-Za-z][A-Za-z0-9_-]{0,39}$', row['name']) or row['name'] == 'Initial':
            raise ValueError('Invalid stage name: %s' % row['name'])
        if row['name'] in names:
            raise ValueError('Duplicate stage name: %s' % row['name'])
        names.add(row['name'])
        if row['kind'] not in ('LOAD', 'HOLD', 'UNLOAD'):
            raise ValueError('Stage kind must be LOAD, HOLD or UNLOAD.')
        for key in ('duration', 'initial', 'minimum', 'maximum'):
            row[key] = finite(row[key], key)
        if not (0 < row['minimum'] <= row['initial'] <= row['maximum'] <= row['duration']):
            raise ValueError('%s: require 0 < min <= initial <= max <= duration.' % row['name'])
        row['increments'] = int(row['increments'])
        if row['increments'] < int(math.ceil(row['duration'] / row['maximum'])):
            raise ValueError('%s: maximum increment count is too small for duration/maxInc.' % row['name'])
        if row['kind'] == 'HOLD':
            row['targets'] = copy.deepcopy(previous)
        for c in COMPONENTS:
            value = finite(row['targets'].get(c, 0), 'Target %s' % c)
            if c not in active_components(cfg) and abs(value)>1.e-15:
                raise ValueError('Thickness shear %s is not allowed in QUASI2D; clear its targets.' % c)
            row['targets'][c] = value
            if c in ('11', '22', '33') and cfg['controls'].get(c) == 'H' and value <= -1:
                raise ValueError('Stretch F%s must remain positive.' % c)
        previous = row['targets']
    return cfg


class SpatialIndex(object):
    def __init__(self, tolerance):
        self.tolerance = tolerance
        self.buckets = {}

    def bucket(self, point):
        return tuple(int(math.floor(x / self.tolerance)) for x in point)

    def find(self, point):
        center = self.bucket(point)
        found = []
        for offset in itertools.product((-1, 0, 1), repeat=3):
            key = tuple(center[i] + offset[i] for i in range(3))
            for entry in self.buckets.get(key, ()):
                error = max(abs(entry[0][i] - point[i]) for i in range(3))
                if error <= self.tolerance:
                    found.append((entry[1], error))
        return found

    def add(self, point, value):
        self.buckets.setdefault(self.bucket(point), []).append((point, value))


def build_constraints(nodes, config, progress=None):
    """Return equations (slave,dof,master, [(component,coefficient)]).

    An equation is u_slave-u_master-sum(coefficient*d_component)=0.
    master=None is an affine/zero thickness equation. Boundary slave DOFs
    never occur as masters. Node coordinates within tolerance are normalized.
    """
    cfg = validate_config(config)
    if not nodes:
        raise ValueError('Selected instances contain no nodes.')
    node_map = dict(nodes)
    if len(node_map) != len(nodes):
        raise ValueError('Duplicate instance/node identity.')
    low = tuple(min(p[j] for p in node_map.values()) for j in range(3))
    high = tuple(max(p[j] for p in node_map.values()) for j in range(3))
    lengths = tuple(high[j] - low[j] for j in range(3))
    if min(lengths) <= 0:
        raise ValueError('RVE must have three positive dimensions.')
    tol = cfg['tolerance'] or 1.e-6 * min(lengths)
    if tol >= min(lengths) / 100:
        raise ValueError('Pairing tolerance is too large relative to cell dimensions.')
    raw = SpatialIndex(tol)
    for key, xyz in nodes:
        if any(math.isnan(v) or math.isinf(v) for v in xyz):
            raise ValueError('Nonfinite node coordinates: %s' % (key,))
        if raw.find(xyz):
            raise ValueError('Coincident/too-close nodes (including across instances): %s at %s. Merge/connect interfaces first, or reduce tolerance.' % (key, xyz))
        raw.add(xyz, key)
    t = 'XYZ'.index(cfg['thickness'])
    periodic = [0, 1, 2] if cfg['mode'] == '3D' else [j for j in range(3) if j != t]
    normalized = {}
    for key, xyz in nodes:
        normalized[key] = tuple(low[j] if abs(xyz[j]-low[j]) <= tol else high[j] if abs(xyz[j]-high[j]) <= tol else xyz[j] for j in range(3))
    index = SpatialIndex(tol)
    for key, xyz in normalized.items():
        index.add(xyz, key)
    equations, fixed = [], []
    masters, errors = set(), []
    for count, (key, xyz) in enumerate(sorted(normalized.items())):
        if progress and count % 200 == 0:
            progress(count, len(nodes), 'Pairing nodes / composing equations')
        # Validate every boundary image, including face/edge/corner counterparts.
        switches = [j for j in periodic if xyz[j] in (low[j], high[j])]
        for flags in itertools.product((0, 1), repeat=len(switches)):
            target = list(xyz)
            for j, flag in zip(switches, flags):
                if flag:
                    target[j] = high[j] if xyz[j] == low[j] else low[j]
            matches = index.find(target)
            if len(matches) != 1:
                raise ValueError('Missing/ambiguous periodic image for %s at %s; expected %s, candidates=%d.' % (key, xyz, target, len(matches)))
            errors.append(matches[0][1])
        target = list(xyz)
        offsets = [0, 0, 0]
        for j in periodic:
            if xyz[j] == high[j]:
                target[j] = low[j]
                offsets[j] = 1
        if cfg['mode'] == 'QUASI2D':
            # Every layer must have the same column footprint.
            target[t] = low[t]
        matches = index.find(target)
        if len(matches) != 1:
            raise ValueError('Incomplete thickness column/canonical image for %s; expected %s.' % (key, target))
        master = matches[0][0]
        masters.add(master)
        for i in range(3):
            if cfg['mode'] == 'QUASI2D' and i == t:
                coefficient = (xyz[t] - low[t]) / lengths[t]
                if abs(coefficient) < 1.e-15:
                    fixed.append((key, i+1))
                else:
                    equations.append((key, i+1, None, [('%d%d' % (t+1,t+1), coefficient)]))
            elif key != master:
                terms = [('%d%d' % (i+1,j+1), float(offsets[j])) for j in periodic if offsets[j] and i <= j]
                equations.append((key, i+1, master, terms))
    # Match complete layer populations; reject partial/truncated columns.
    if cfg['mode'] == 'QUASI2D':
        levels = []
        for z in sorted(set(p[t] for p in normalized.values())):
            if not levels or z-levels[-1] > tol:
                levels.append(z)
        column_counts = {}
        for key, xyz in normalized.items():
            base = list(xyz); base[t] = low[t]
            found = index.find(base)
            if len(found) != 1:
                raise ValueError('Incomplete thickness column: %s' % (key,))
            column_counts[found[0][0]] = column_counts.get(found[0][0], 0) + 1
        if any(n != len(levels) for n in column_counts.values()):
            raise ValueError('Thickness columns do not have identical layer populations.')
    anchor = min(masters, key=lambda k: (normalized[k], k))
    for i in range(3):
        if cfg['mode'] == '3D' or i != t:
            fixed.append((anchor, i+1))
    slave_dofs = set((e[0], e[1]) for e in equations)
    if len(slave_dofs) != len(equations):
        raise AssertionError('Duplicate dependent DOF.')
    if any(e[2] is not None and (e[2], e[1]) in slave_dofs for e in equations):
        raise AssertionError('Constraint chain/cycle detected.')
    if slave_dofs.intersection(fixed):
        raise AssertionError('Dependent DOF also has a direct boundary condition.')
    return dict(equations=equations, fixed=fixed, anchor=anchor,
                low=low, high=high, lengths=lengths, tolerance=tol,
                volume=lengths[0]*lengths[1]*lengths[2],
                node_count=len(nodes), maximum_pairing_error=max(errors or [0]),
                independent_nodes=sorted(masters), config=cfg)


def compile_path(config, lengths):
    cfg = validate_config(config)
    rows = cfg['stages']
    active = active_components(cfg)
    volume = lengths[0]*lengths[1]*lengths[2]
    elapsed, previous = 0.0, dict((c, 0.0) for c in active)
    stages, hold_starts = [], []
    for row in rows:
        data = copy.deepcopy(row)
        data['start'] = elapsed
        data['paths'] = {}
        if row['kind'] == 'HOLD':
            hold_starts.append(elapsed)
        for c in active:
            j = int(c[1])-1
            factor = lengths[j] if cfg['controls'][c] == 'H' else volume / lengths[j]
            data['paths'][c] = [(0.0, previous[c]*factor),
                                (row['duration'], row['targets'][c]*factor)]
        stages.append(data)
        previous = row['targets']; elapsed += row['duration']
    if cfg['organization'] == 'SINGLE':
        data = copy.deepcopy(stages[0])
        data['duration'] = elapsed
        data['maximum'] = min(r['maximum'] for r in stages)
        data['minimum'] = min(r['minimum'] for r in stages)
        data['increments'] = max(max(r['increments'] for r in stages),
                                 2*int(math.ceil(elapsed/data['maximum']))+100)
        data['paths'] = {}
        for c in active:
            points = [(0.0, 0.0)]
            for row in stages:
                points.append((row['start']+row['duration'], row['paths'][c][-1][1]))
            data['paths'][c] = points
        stages = [data]
    return dict(stages=stages, duration=elapsed, hold_starts=hold_starts,
                active=active, config=cfg)
