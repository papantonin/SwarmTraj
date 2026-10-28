"""Follow-up matrix: computation budgets, horizon sensitivity and mixed profiles.

Uses the isolated, failure-preserving scaling harness. Physical limits and
validation margins are identical within all paired computational-budget tests.
Horizon and profile-policy changes are explicit separate mission experiments.
"""

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from benchmark_scaling_campaign import base_case, family_case, run, sources, write_json


def jobs():
    records = []
    def add(suite, family, n, seed, snapshots, config, expected=None):
        records.append({'id': f'{suite}-{family}-n{n}-s{seed}', 'suite': suite,
            'family': family, 'num_drones': n, 'seed': seed, 'config': asdict(config),
            'snapshots': [asdict(s) for s in snapshots], 'expected_status': expected})
    for n in range(1, 6):
        for seed in (7, 42):
            snapshots, config = base_case(n, seed)
            for candidates, attempts in ((2, 10), (8, 30), (28, 90), (56, 180)):
                add('budgets', f'c{candidates}_a{attempts}', n, seed, snapshots,
                    replace(config, candidates=candidates, attempts=attempts))
        snapshots, config = family_case(n, 7, 'spread180')
        for nodes in (1, 5, 10, 30, 30000):
            add('search_caps', f'nodes{nodes}', n, 7, snapshots,
                replace(config, max_search_nodes=nodes, selection_policy='first'),
                'search_budget_exhausted' if nodes == 1 and n > 1 else None)
        for seed in (7, 19):
            snapshots, config = family_case(n, seed, 'rich_strict')
            add('profiles', 'mixed', n, seed, snapshots, replace(config, profile_policy='adaptive_mixed'))
    for n in (1, 3, 5):
        for seed in (7, 42):
            snapshots, config = base_case(n, seed)
            for horizon in (60, 90, 120, 180):
                add('horizon', f'h{horizon}', n, seed, snapshots, replace(config, horizon=horizon))
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True, type=Path, help='Completed or running main campaign directory')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args(argv)
    directory = args.output.resolve()
    if args.resume:
        manifest = json.loads((directory / 'manifest.json').read_text())
        if manifest['source_sha256'] != sources():
            parser.error('source differs from saved campaign')
    else:
        if directory.exists():
            parser.error('output must be new (or use --resume)')
        reference = json.loads((args.reference / 'manifest.json').read_text())
        if reference['source_sha256'] != sources():
            parser.error('reference campaign uses different sources')
        if args.timeout <= 0:
            parser.error('timeout must be positive')
        directory.mkdir(parents=True)
        for name in ('logs', 'missions', 'checkpoints', 'completed'):
            (directory / name).mkdir()
        manifest = {k: v for k, v in reference.items() if k not in ('jobs', 'created_utc', 'execute')}
        manifest.update(created_utc=datetime.now(timezone.utc).isoformat(), jobs=jobs(), execute=False,
                        timeout_s=args.timeout, reference_campaign=str(args.reference.resolve()),
                        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        write_json(directory / 'manifest.json', manifest)
        (directory / 'matrix-generator.py').write_bytes(Path(__file__).read_bytes())
    return run(directory, manifest)


if __name__ == '__main__':
    raise SystemExit(main())
