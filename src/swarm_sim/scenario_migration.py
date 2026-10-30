"""Explicit, non-overwriting migration of shared PX4 editor scenario records."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path


def migrate_scenario(source, destination):
    source, destination = Path(source), Path(destination)
    provenance = destination.with_name(destination.name + '.migration.json')
    if destination.exists() or provenance.exists():
        raise ValueError('Migration requires a new scenario and provenance path; no file is overwritten.')
    original = source.read_bytes()
    record = json.loads(original)
    schema = record.get('schema', '') if isinstance(record, dict) else ''
    if schema.startswith('swarm-px4-editor-fleet-'):
        from .px4_fleet import load_fleet as load, fleet_record as export
    elif schema.startswith('swarm-px4-editor-v'):
        from .px4_scenario import load_scenario as load, scenario_record as export
    else:
        raise ValueError('Expected a shared single-aircraft or four-aircraft PX4 editor scenario.')
    snapshots, old = load(source)
    updated = replace(old, generator_mode='physical_mission', numerical_backend='cpp')
    from .trajectory2d.production import require_production
    require_production(updated)
    migrated = export(snapshots, updated)
    if source.read_bytes() != original:
        raise ValueError('Source changed during migration; no output written.')
    changes = {name: {'before': getattr(old, name), 'after': getattr(updated, name)}
               for name in ('generator_mode', 'numerical_backend')
               if getattr(old, name) != getattr(updated, name)}
    evidence = dict(schema='swarm-scenario-migration-v1',
        source_sha256=hashlib.sha256(original).hexdigest(), source_schema=schema,
        destination_schema=migrated['schema'], changes=changes,
        generator_semantics_changed=old.generator_mode != updated.generator_mode,
        planning_performed=False, trajectory_validation_transferred=False)
    # Exclusive creation also protects against another writer after the checks.
    with provenance.open('x', encoding='utf-8') as stream:
        json.dump(evidence, stream, indent=2, allow_nan=False)
        stream.write('\n')
    try:
        with destination.open('x', encoding='utf-8') as stream:
            json.dump(migrated, stream, indent=2, allow_nan=False)
            stream.write('\n')
    except BaseException:
        provenance.unlink()
        raise
    return evidence


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='New scenario JSON; the source is never modified')
    args = parser.parse_args(argv)
    try:
        evidence = migrate_scenario(args.source, args.output)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(f'Migrated scenario: {args.output}')
    if evidence['generator_semantics_changed']:
        print('Generator changed from geometric to physical: this is a new planning request, not a trajectory replay.')
    print('No trajectory or safety validation was transferred; the new mission must be planned and validated.')
    return 0
