"""Abaqus/Standard adapter. No material edits or job submissions."""
from __future__ import division, print_function
import copy
import json
import os
import sys
import tempfile
import time
import traceback

import pbc_core as core

# Abaqus lazily installs Model repositories when these modules are imported.
# This is needed in noGUI sessions and when the user opens the plugin before
# visiting the Interaction/Load modules.
try:
    import interaction
    import load
    import step
    import assembly
    import material
    import section
except ImportError:
    pass

VERSION = '1.0.0'


def native(path):
    if sys.version_info[0] < 3 and isinstance(path, unicode):
        return path.encode('mbcs')
    return path


def _write(path, data):
    kwargs = {'encoding': 'mbcs'} if sys.version_info[0] < 3 else {}
    text = json.dumps(data, indent=2, ensure_ascii=True, **kwargs)
    with open(native(path), 'wb') as stream:
        stream.write(text.encode('ascii'))


def _native_values(value):
    if isinstance(value, dict):
        return dict((_native_values(k), _native_values(v)) for k,v in value.items())
    if isinstance(value, list):
        return [_native_values(v) for v in value]
    if sys.version_info[0] < 3 and isinstance(value, unicode):
        return value.encode('mbcs')
    return value


def _records():
    from abaqus import mdb
    import customKernel
    return copy.deepcopy(getattr(mdb.customData, 'PeriodicBC', {}))


def _save_records(records):
    from abaqus import mdb
    mdb.customData.PeriodicBC = records


def _key(config):
    return config['model'] + '/' + config['tag']


def _prefix(config):
    return 'PBC_' + config['tag'] + '_'


def _collect(model, config):
    assembly = model.rootAssembly
    selected = config['instances']
    if not selected:
        raise ValueError('Select at least one meshed instance (comma-separated names).')
    if len(set(selected)) != len(selected):
        raise ValueError('Duplicate instance names.')
    nodes, elements, materials = [], [], set()
    for name in selected:
        if name not in list(assembly.instances.keys()):
            raise ValueError('Instance not found: %s' % name)
        instance = assembly.instances[name]
        if not len(instance.nodes) or not len(instance.elements):
            raise ValueError('Instance %s has no active mesh.' % name)
        nodes.extend([((name, int(n.label)), tuple(n.coordinates)) for n in instance.nodes])
        for element in instance.elements:
            if str(element.type) not in ('C3D8', 'C3D8R'):
                raise ValueError('Unsupported element %s:%s (%s).' % (name, element.label, element.type))
            elements.append(tuple((name, int(n.label)) for n in element.getNodes()))
        part = model.parts[instance.partName]
        if not len(part.sectionAssignments):
            raise ValueError('No section assignments in %s.' % name)
        for assignment in part.sectionAssignments:
            if getattr(assignment, 'suppressed', False):
                continue
            section = model.sections[assignment.sectionName]
            material = getattr(section, 'material', None)
            if not material:
                raise ValueError('Only homogeneous solid material sections are supported.')
            materials.add(material)
    umat_materials, creep_materials = [], []
    for name in materials:
        mat = model.materials[name]
        try:
            if len(mat.userMaterial.mechanicalConstants)>0: umat_materials.append(name)
        except Exception:
            pass
        try:
            if len(mat.creep.table)>0: creep_materials.append(name)
        except Exception:
            pass
    source=config['source']
    # Ordinary elastic phases can coexist with time-dependent phases.
    if source=='UMAT' and (not umat_materials or creep_materials):
        raise ValueError('UMAT mode requires a user material and cannot include built-in Creep phases in the same configuration.')
    if source=='CREEP' and (not creep_materials or umat_materials):
        raise ValueError('CREEP mode requires built-in Creep data; a simultaneous UMAT/Creep mixture is outside this release.')
    if source=='ELASTIC' and (umat_materials or creep_materials):
        raise ValueError('Select UMAT or CREEP for the user/time-dependent materials already present; ELASTIC mode does not override materials.')
    return nodes, elements, sorted(materials)


def _connection_check(nodes, elements, plan):
    parent = dict((key, key) for key, xyz in nodes)

    def root(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k

    def join(a, b):
        parent[root(a)] = root(b)
    for element in elements:
        for node in element[1:]:
            join(element[0], node)
    for slave, dof, master, terms in plan['equations']:
        if master is not None:
            join(slave, master)
    components = set(root(k) for k in parent)
    if len(components) != 1:
        raise ValueError('%d disconnected mesh components remain after periodic pairing. Connect/merge physical interfaces first; the plugin does not add interface ties.' % len(components))


def _conflicts(model, config, owned):
    """Conservative fail-closed handling for uninspectable CAE region objects."""
    issues = []
    for repo_name in ('boundaryConditions', 'constraints', 'loads', 'interactions'):
        repo = getattr(model, repo_name)
        ours = set(owned.get(repo_name, ()))
        for name in repo.keys():
            obj = repo[name]
            if name in ours or getattr(obj, 'suppressed', False):
                continue
            # Existing BC/constraint regions may be geometry, internal sets or
            # transformed nodal regions. Do not silently assume independence.
            issues.append('%s/%s (%s)' % (repo_name, name, type(obj).__name__))
    if issues:
        raise ValueError('Existing active BCs/constraints/loads/interactions require review in a separate model copy before PBC generation. This release conservatively rejects: ' + ', '.join(issues[:20]))
    if getattr(model, 'keywordBlock', None) is not None:
        for block in model.keywordBlock.sieBlocks:
            if str(block).lstrip().upper().startswith(('*TRANSFORM', '*EQUATION', '*MPC', '*BOUNDARY', '*COUPLING', '*TIE')):
                if getattr(model.keywordBlock, 'edited', False):
                    raise ValueError('Manually edited constraint/transform keywords are unsupported.')
    for bc in model.boundaryConditions.values():
        if getattr(bc, 'name', '') not in owned.get('boundaryConditions', ()) and getattr(bc, 'localCsys', None):
            raise ValueError('Local nodal transforms must be removed first.')


def _step_names(cfg, path):
    if cfg['step_source'] == 'EXISTING':
        return [r['name'] for r in path['stages']]
    return [_prefix(cfg)+r['name'] for r in path['stages']]


def inspect(config, progress=None):
    from abaqus import mdb
    cfg = _native_values(core.validate_config(config))
    if cfg['model'] not in list(mdb.models.keys()):
        raise ValueError('Model not found: %s' % cfg['model'])
    model = mdb.models[cfg['model']]
    owned = _records().get(_key(cfg), {}).get('objects', {})
    _conflicts(model, cfg, owned)
    nodes, elements, materials = _collect(model, cfg)
    plan = core.build_constraints(nodes, cfg, progress)
    if cfg['mode'] == 'QUASI2D':
        xyz = dict(nodes); t = 'XYZ'.index(cfg['thickness']); tol = plan['tolerance']
        for element in elements:
            bottom = min(xyz[k][t] for k in element)
            top = max(xyz[k][t] for k in element)
            lower = [k for k in element if abs(xyz[k][t]-bottom) <= tol]
            upper = [k for k in element if abs(xyz[k][t]-top) <= tol]
            if len(lower) != 4 or len(upper) != 4 or top-bottom <= tol:
                raise ValueError('Quasi-2D requires straight extruded hex layers; invalid element near %s.' % (element[0],))
            for k in lower:
                if sum(all(abs(xyz[k][j]-xyz[q][j]) <= tol for j in range(3) if j != t) for q in upper) != 1:
                    raise ValueError('Quasi-2D element does not connect matching thickness columns: %s.' % (element[0],))
    _connection_check(nodes, elements, plan)
    path = core.compile_path(cfg, plan['lengths'])
    for row in path['stages']:
        amplitude = row.get('amplitude','')
        if amplitude:
            if amplitude not in list(model.amplitudes.keys()):
                raise ValueError('Amplitude not found: %s' % amplitude)
            obj=model.amplitudes[amplitude]
            if type(obj).__name__!='TabularAmplitude' or str(obj.timeSpan)!='STEP':
                raise ValueError('Existing amplitudes must be STEP-time TabularAmplitude objects.')
            if any(abs(points[0][1])>1.e-15 for points in row['paths'].values()):
                raise ValueError('Existing amplitude requires zero stage-start targets; use generated paths for holds/unloads.')
            if not obj.data or abs(obj.data[0][0])>1.e-12 or abs(obj.data[0][1])>1.e-12 or abs(obj.data[-1][0]-row['duration'])>1.e-8*max(1.,row['duration']) or abs(obj.data[-1][1]-1.)>1.e-12:
                raise ValueError('Existing amplitude must span (0,0) to (stage duration,1).')
    names = _step_names(cfg, path)
    if cfg['step_source'] == 'EXISTING':
        actual = list(model.steps.keys())
        if any(name not in actual or name == 'Initial' for name in names):
            raise ValueError('Existing step names must match path stage names (single-step uses first row name).')
        indices = [actual.index(n) for n in names]
        if indices != list(range(1, 1+len(names))):
            raise ValueError('Existing selected steps must be consecutive and start directly after Initial; preloaded models are unsupported.')
        for row, name in zip(path['stages'], names):
            step = model.steps[name]
            expected = 'ViscoStep' if cfg['source'] == 'CREEP' else 'StaticStep'
            if type(step).__name__ != expected:
                raise ValueError('%s must be %s for the selected constitutive source.' % (name, expected))
            if abs(step.timePeriod-row['duration']) > 1.e-8*max(1.,row['duration']):
                raise ValueError('%s duration differs from the path; existing steps are not changed.' % name)
            if bool(step.nlgeom) != bool(cfg['nlgeom']):
                raise ValueError('%s NLGEOM differs from the requested setting.' % name)
    else:
        foreign = [n for n in model.steps.keys() if n != 'Initial' and n not in owned.get('steps', ())]
        if foreign or cfg['previous'] != 'Initial':
            raise ValueError('CREATE requires no user analysis steps. Use EXISTING to bind the initial loading history, or work in a clean model copy.')
    report = dict(version=VERSION, config=cfg, node_count=len(nodes),
                  element_count=len(elements), materials=materials,
                  lengths=plan['lengths'], initial_cell_volume=plan['volume'],
                  tolerance=plan['tolerance'], maximum_pairing_error=plan['maximum_pairing_error'],
                  equation_count=len(plan['equations']), fixed_dof_count=len(plan['fixed']),
                  macro_components=list(path['active']), step_names=names,
                  duration=path['duration'], hold_start_times=path['hold_starts'],
                  path=path['stages'],
                  warnings=['Stress is first Piola nominal stress on initial gross cell area.',
                            'Upper-triangular macro F fixes macro orientation; lower components are zero.',
                            'Model units must match material/UMAT time units.'])
    if cfg['mode'] == 'QUASI2D':
        report['warnings'].append('Uniform thickness stretch and zero thickness shear/curvature are imposed. Confirm through-thickness microstructure applicability.')
    if cfg['organization'] == 'SINGLE' and path['duration'] > 10:
        report['warnings'].append('SINGLE uses a conservative global maxInc and may need many increments; MULTI is recommended for long creep holds.')
    return model, plan, path, report


def _remove_objects(model, record):
    objects = record.get('objects', {})
    for repo_name in ('boundaryConditions','constraints','loads','interactions'):
        for name in getattr(model,repo_name).keys():
            if name not in objects.get(repo_name, ()):
                raise ValueError('Cannot safely remove PBC while user object %s/%s exists; remove it in a model copy first.' % (repo_name,name))
    owned_steps = set(objects.get('steps', ()))
    for name, step in model.steps.items():
        if name not in owned_steps and getattr(step, 'previous', '') in owned_steps:
            raise ValueError('User step %s depends on a plugin step; remove/relink it first.' % name)
    # Do not delete user objects that have been attached to plugin steps.
    for repo_name in ('boundaryConditions', 'loads', 'historyOutputRequests', 'fieldOutputRequests'):
        for name, obj in getattr(model, repo_name).items():
            if name not in objects.get(repo_name, ()) and getattr(obj, 'createStepName', '') in owned_steps:
                raise ValueError('User object %s/%s depends on a plugin step.' % (repo_name, name))
    for repo_name in ('historyOutputRequests', 'fieldOutputRequests', 'loads', 'boundaryConditions', 'constraints', 'amplitudes'):
        repo = getattr(model, repo_name)
        for name in objects.get(repo_name, ()):
            if name in list(repo.keys()):
                del repo[name]
    for name in reversed(list(model.steps.keys())):
        if name in owned_steps:
            del model.steps[name]
    for name in objects.get('sets', ()):
        if name in list(model.rootAssembly.sets.keys()):
            del model.rootAssembly.sets[name]
    for name in objects.get('features', ()):
        if name in list(model.rootAssembly.features.keys()):
            del model.rootAssembly.features[name]


def _transaction(model_name, operation):
    from abaqus import mdb
    records = _records()
    backup = 'PBC_Backup_%d' % int(time.time()*1000)
    mdb.Model(name=backup, objectToCopy=mdb.models[model_name])
    try:
        result = operation(mdb.models[model_name], records)
    except Exception:
        del mdb.models[model_name]
        mdb.models.changeKey(fromName=backup, toName=model_name)
        _save_records(records)
        raise
    else:
        del mdb.models[backup]
        return result


def remove(config):
    from abaqus import mdb
    config = _native_values(config)
    model_name = config['model']
    key = _key(config)
    if key not in _records():
        raise ValueError('No saved PeriodicBC configuration for this model/tag.')

    def operation(model, original_records):
        records = copy.deepcopy(original_records)
        _remove_objects(model, records[key])
        del records[key]
        _save_records(records)
        return dict(removed=key, note='Only recorded plugin objects were removed.')
    return _transaction(model_name, operation)


def generate(config, progress=None):
    from abaqus import mdb
    from abaqusConstants import ON, OFF, STEP, UNSET
    model, plan, path, report = inspect(config, progress)
    cfg = plan['config']; prefix = _prefix(cfg); key = _key(cfg)

    def operation(model, original_records):
        records = copy.deepcopy(original_records)
        if key in records:
            _remove_objects(model, records[key])
        objects = dict((r, []) for r in ('sets','features','constraints','boundaryConditions','loads','amplitudes','steps','historyOutputRequests','fieldOutputRequests'))
        # Reject namespace collisions before creating any PBC object.
        for repo_name in objects:
            repo = getattr(model.rootAssembly if repo_name in ('sets','features') else model, repo_name)
            if any(n.startswith(prefix) for n in repo.keys()):
                raise ValueError('Unowned object uses reserved prefix %s in %s.' % (prefix,repo_name))
        assembly = model.rootAssembly
        node_sets = {}

        def node_set(node):
            if node not in node_sets:
                name = prefix+'N%d' % (len(node_sets)+1)
                assembly.SetFromNodeLabels(name=name, nodeLabels=((node[0], (node[1],)),))
                objects['sets'].append(name); node_sets[node] = name
            return node_sets[node]

        def bc(name, set_name, dof, value, step='Initial', amplitude=None):
            kwargs = {'u%d' % dof: value} if dof <= 3 else {'ur%d' % (dof-3): value}
            if amplitude:
                kwargs['amplitude'] = amplitude
            model.DisplacementBC(name=name, createStepName=step,
                                 region=assembly.sets[set_name], **kwargs)
            objects['boundaryConditions'].append(name)

        rps = {}
        for j in range(3):
            point = list(plan['high']); point[j] += plan['lengths'][j]*.25
            feature = assembly.ReferencePoint(point=tuple(point))
            name = prefix+'RP_'+ 'XYZ'[j]
            assembly.features.changeKey(fromName=feature.name, toName=name)
            objects['features'].append(name)
            assembly.Set(name=name, referencePoints=(assembly.referencePoints[feature.id],))
            objects['sets'].append(name); rps[j+1] = name
            # Unused reference-point DOFs never enter any equation/load, so
            # Abaqus does not activate them. A BC on an inactive DOF produces
            # spurious warnings; lower-triangular zero jumps are omitted above.

        total = len(plan['equations'])
        for index, (slave,dof,master,terms) in enumerate(plan['equations']):
            expression = [(1.0,node_set(slave),dof)]
            if master is not None:
                expression.append((-1.0,node_set(master),dof))
            for component, coefficient in terms:
                expression.append((-coefficient,rps[int(component[1])],int(component[0])))
            name = prefix+'EQ_%d' % (index+1)
            model.Equation(name=name, terms=tuple(expression))
            objects['constraints'].append(name)
            if progress and index % 100 == 0:
                progress(index,total,'Creating periodic equations')
        for index,(node,dof) in enumerate(plan['fixed']):
            bc(prefix+'FIX_%d' % index,node_set(node),dof,0.)

        names = _step_names(cfg,path)
        previous = 'Initial'
        for row,name in zip(path['stages'],names):
            if cfg['step_source'] == 'CREATE':
                kwargs = dict(name=name,previous=previous,timePeriod=row['duration'],
                              initialInc=row['initial'],minInc=row['minimum'],
                              maxInc=row['maximum'],maxNumInc=row['increments'],
                              nlgeom=ON if cfg['nlgeom'] else OFF)
                if cfg['source'] == 'CREEP':
                    model.ViscoStep(cetol=cfg['cetol'], **kwargs)
                else:
                    model.StaticStep(**kwargs)
                objects['steps'].append(name)
            previous = name

        for component in path['active']:
            i,j = int(component[0]),int(component[1])
            mode = cfg['controls'][component]
            object_name = prefix+mode+component
            for step_index,(row,name) in enumerate(zip(path['stages'],names)):
                points = row['paths'][component]
                amplitude = row.get('amplitude','')
                magnitude = 1.0
                if amplitude:
                    # User amplitude controls only transition from zero; reject
                    # nonzero-start paths rather than silently resetting a hold.
                    if abs(points[0][1]) > 1.e-15:
                        raise ValueError('Existing amplitudes require a zero stage-start value for %s; use generated paths for holds/unloads.' % component)
                    if amplitude not in list(model.amplitudes.keys()):
                        raise ValueError('Amplitude not found: %s' % amplitude)
                    magnitude = points[-1][1]
                    if abs(magnitude)<1.e-30:
                        amplitude=prefix+'AMP_%s_%d' % (component,step_index)
                        model.TabularAmplitude(name=amplitude,timeSpan=STEP,data=tuple(points))
                        objects['amplitudes'].append(amplitude)
                        magnitude=1.0
                else:
                    amplitude = prefix+'AMP_%s_%d' % (component,step_index)
                    model.TabularAmplitude(name=amplitude,timeSpan=STEP,data=tuple(points))
                    objects['amplitudes'].append(amplitude)
                if mode == 'H':
                    if step_index == 0:
                        bc(object_name,rps[j],i,magnitude,name,amplitude)
                    else:
                        model.boundaryConditions[object_name].setValuesInStep(
                            stepName=name,amplitude=amplitude,**{'u%d'%i:magnitude})
                else:
                    kwargs = {'cf%d'%i:magnitude}
                    if step_index == 0:
                        model.ConcentratedForce(name=object_name,createStepName=name,
                                                region=assembly.sets[rps[j]],amplitude=amplitude,**kwargs)
                        objects['loads'].append(object_name)
                    else:
                        model.loads[object_name].setValuesInStep(stepName=name,amplitude=amplitude,**kwargs)
        for j,name in rps.items():
            output = prefix+'HISTORY_'+str(j)
            dofs = [int(c[0]) for c in path['active'] if int(c[1]) == j]
            variables = tuple('%s%d' % (v,i) for v in ('U','RF') for i in dofs)
            variables += tuple('CF%d' % i for i in dofs if cfg['controls']['%d%d'%(i,j)] == 'P')
            model.HistoryOutputRequest(name=output,createStepName=names[0],
                                       variables=variables,
                                       region=assembly.sets[name],frequency=1)
            objects['historyOutputRequests'].append(output)
        report['objects'] = objects
        records[key] = report
        _save_records(records)
        if progress:
            progress(total,total,'Completed')
        return report
    return _transaction(cfg['model'], operation)


def dispatch(action, config_json, report_path):
    status_path = report_path+'.status'
    def progress(current,total,stage):
        _write(status_path,dict(state='PROGRESS',current=current,total=total,stage=stage))
    try:
        config = json.loads(config_json)
        progress(0,1,'Checking model')
        if action == 'CHECK':
            result = inspect(config,progress)[3]
        elif action == 'GENERATE':
            result = generate(config,progress)
        elif action == 'REMOVE':
            result = remove(config)
        else:
            raise ValueError('Unknown action: %s' % action)
        _write(report_path,result)
        _write(status_path,dict(state='DONE',stage=action+' completed',current=1,total=1))
        print('PeriodicBC %s complete. Report: %s' % (action,report_path))
        return result
    except Exception as error:
        _write(status_path,dict(state='ERROR',message=str(error),traceback=traceback.format_exc()))
        print(traceback.format_exc())
        return None
