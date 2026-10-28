"""Render saved exploratory mission scenes without replanning or changing inputs.

The 3D panel focuses on arrival; the XY and altitude panels retain the entire
validated horizon. Failed jobs show inputs only, never invented trajectories.
"""

import argparse
from collections import Counter
import gzip
import hashlib
import html
import json
from pathlib import Path
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Circle, Rectangle
import numpy as np

from benchmark_scaling_campaign import decode
from swarm_sim.trajectory2d.dynamics import holding_circle
from swarm_sim.trajectory2d.obstacle_view import draw_obstacles_3d


COLORS = tuple(plt.get_cmap('tab10').colors[:5])


def load_cases(directory, count):
    manifest = json.loads((directory / 'manifest.json').read_text())
    jobs = {j['id']: j for j in manifest['jobs']}
    rows = [json.loads(line) for line in (directory / 'journal.jsonl').read_text().splitlines()]
    cases = []
    for row in rows:
        if row['suite'] not in ('core', 'random') or row['num_drones'] != count:
            continue
        with gzip.open(directory / 'missions' / (row['id']+'.json.gz'), 'rt') as stream:
            doc = json.load(stream)
        canonical = {k: v for k, v in doc.items() if k != 'sha256'}
        digest = hashlib.sha256(json.dumps(canonical, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if digest != doc['sha256'] or digest != row['mission_sha256'] or doc['status'] != row['status']:
            raise RuntimeError(f'original mission mismatch: {row["id"]}')
        snapshots, config = decode(jobs[row['id']])
        if doc['status'] == 'validated' and len(doc['plans']) != count:
            raise RuntimeError('incomplete selected fleet')
        cases.append((row, doc, snapshots, config))
    return cases


def paths(doc):
    return [(p, np.asarray(p['flight_states'])[:, :3]) for p in doc['plans']]


def scene_points(snapshots, config, arrays):
    data = [np.array([s.position3d for s in snapshots]), np.array([[*config.target, config.target_altitude]])]
    for snapshot in snapshots:
        circle = holding_circle(snapshot.state, snapshot.speed)
        if circle is not None:
            center, radius = circle
            data.extend([np.array([[center[0]-radius, center[1]-radius, snapshot.altitude]]),
                         np.array([[center[0]+radius, center[1]+radius, snapshot.altitude]])])
    data.extend(arrays)
    for obstacle in config.obstacles:
        data.extend([np.array([np.subtract(obstacle.center, obstacle.half_extents)]),
                     np.array([np.add(obstacle.center, obstacle.half_extents)])])
    return np.concatenate(data)


def limits(axis, points, three=False):
    low, high = points.min(axis=0), points.max(axis=0)
    pad = np.maximum((high-low)*.08, (20, 20, 8))
    axis.set_xlim(low[0]-pad[0], high[0]+pad[0])
    axis.set_ylim(low[1]-pad[1], high[1]+pad[1])
    if three:
        axis.set_zlim(min(0, low[2]-pad[2]), high[2]+pad[2])
        aspect = np.maximum(high-low, 20)
        aspect[2] = max(aspect[2], .35*max(aspect[:2]))
        axis.set_box_aspect(aspect)
    else:
        axis.set_aspect('equal', adjustable='box')
    axis.set_autoscale_on(False)


def draw_xy(axis, snapshots, config, arrays, compact=False):
    for obstacle in config.obstacles:
        center, half = np.array(obstacle.center), np.array(obstacle.half_extents)
        shape = (Circle(center[:2], half[0]) if obstacle.kind == 'cylinder' else
                 Rectangle(center[:2]-half[:2], *(2*half[:2])))
        shape.set(facecolor='#777777', edgecolor='#333333', alpha=.45)
        axis.add_patch(shape)
    for s in snapshots:
        circle = holding_circle(s.state, s.speed)
        if circle is not None:
            axis.add_patch(Circle(circle[0], circle[1], fill=False, ls=':', lw=.8,
                                  color=COLORS[s.drone_id], alpha=.5))
        axis.scatter(*s.state[:2], marker='o', color=COLORS[s.drone_id], s=20, zorder=5)
    for p, xyz in arrays:
        axis.plot(*xyz[:, :2].T, color=COLORS[p['drone_id']], lw=1 if compact else 1.3)
    axis.scatter(*config.target, marker='*', s=110 if compact else 150, color='gold', edgecolor='black', zorder=10)
    axis.add_patch(Circle(config.target, config.target_radius, color='gold', fill=False, lw=1))
    if compact:
        axis.tick_params(labelsize=6)
    else:
        axis.set(xlabel='X (m)', ylabel='Y (m)',
                 title='Full-horizon XY projection' if arrays else 'Initial XY scene')
    axis.grid(alpha=.15)


def full_page(case, index):
    row, doc, snapshots, config = case
    arrays = paths(doc)
    end = min(config.horizon, max((p['arrival_time'] for p, _ in arrays), default=config.horizon)+5)
    visible = [(p, xyz[:int(np.floor(end/p['dt']))+1]) for p, xyz in arrays]
    fig = plt.figure(figsize=(15, 9), layout='constrained')
    grid = fig.add_gridspec(2, 2, width_ratios=(1.4, 1))
    ax = fig.add_subplot(grid[:, 0], projection='3d')
    xy = fig.add_subplot(grid[0, 1])
    altitude = fig.add_subplot(grid[1, 1])
    ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Altitude (m)',
           title=f'Nominal 3D paths through t={end:.1f} s' if arrays else 'Initial scene — no validated trajectory')
    ax.view_init(elev=28, azim=-58)
    bounds = scene_points(snapshots, config, [xyz for _, xyz in visible])
    limits(ax, bounds, three=True)
    draw_obstacles_3d(ax, config)
    limits(ax, bounds, three=True)
    for s in snapshots:
        ax.scatter(*s.position3d, color=COLORS[s.drone_id], s=38)
        ax.text(*s.position3d, f'  D{s.drone_id}', fontsize=8)
        circle = holding_circle(s.state, s.speed)
        if circle is not None:
            angles = np.linspace(0, 2*np.pi, 80)
            ax.plot(circle[0][0]+circle[1]*np.cos(angles), circle[0][1]+circle[1]*np.sin(angles),
                    np.full_like(angles, s.altitude), ':', color=COLORS[s.drone_id], lw=.8, alpha=.5)
    for p, xyz in visible:
        ax.plot(*xyz.T, color=COLORS[p['drone_id']], lw=1.7, label=f"Drone {p['drone_id']} · arrival {p['arrival_time']:.1f} s")
    target = np.r_[config.target, config.target_altitude]
    ax.scatter(*target, color='gold', edgecolor='black', marker='*', s=170, label='Target')
    u, v = np.meshgrid(np.linspace(0, 2*np.pi, 20), np.linspace(0, np.pi, 12))
    r = config.target_radius
    ax.plot_wireframe(target[0]+r*np.cos(u)*np.sin(v), target[1]+r*np.sin(u)*np.sin(v),
                      target[2]+r*np.cos(v), color='goldenrod', alpha=.35, linewidth=.5)
    ax.legend(loc='upper left', fontsize=8)
    draw_xy(xy, snapshots, config, arrays)
    limits(xy, scene_points(snapshots, config, [xyz for _, xyz in arrays]))
    for p, xyz in arrays:
        altitude.plot(np.arange(len(xyz))*p['dt'], xyz[:, 2], color=COLORS[p['drone_id']], label=f"D{p['drone_id']}")
        altitude.axvline(p['arrival_time'], color=COLORS[p['drone_id']], lw=.7, ls=':', alpha=.5)
    altitude.axhline(config.target_altitude, color='goldenrod', ls='--', label='Target altitude')
    altitude.set(xlabel='Mission time (s)', ylabel='Altitude (m)', xlim=(0, config.horizon),
                 title='Full-horizon altitude profiles' if arrays else 'No selected plan — input scene only')
    altitude.grid(alpha=.2)
    if arrays:
        altitude.legend(fontsize=8, ncols=3)
    else:
        altitude.text(.04, .76, row['status']+'\n\n'+textwrap.fill(row['reason'], width=65), transform=altitude.transAxes,
                      va='top', wrap=True, fontsize=9, color='#9c2525')
    status = 'VALIDATED' if arrays else row['status']
    fig.suptitle(f"{index:02d} / {row['id']}\n{status} · obstacles: {len(config.obstacles)} · target: "
                 f"({config.target[0]:.1f}, {config.target[1]:.1f}, {config.target_altitude:.1f}) m", fontsize=14)
    fig.supxlabel('Saved nominal point-mass plans. 3D view focuses on arrival; right panels retain the full horizon. '
                  'No replanning, added obstacles or execution replay.', fontsize=9)
    return fig


def overview(cases, output):
    fig, axes = plt.subplots(12, 4, figsize=(22, 49), layout='constrained')
    for index, (axis, case) in enumerate(zip(axes.flat, cases), 1):
        row, doc, snapshots, config = case
        arrays = paths(doc)
        end = min(config.horizon, max((p['arrival_time'] for p, _ in arrays), default=config.horizon)+5)
        visible = [(p, xyz[:int(end/p['dt'])+1]) for p, xyz in arrays]
        draw_xy(axis, snapshots, config, visible, compact=True)
        limits(axis, scene_points(snapshots, config, [xyz for _, xyz in visible]))
        text = f"{index:02d} · {row['family']} · seed {row['seed']}\n"
        text += f"{len(config.obstacles)} obstacles · "+('validated' if arrays else row['status'])
        axis.set_title(text, fontsize=9, color='#176837' if arrays else '#a32525')
        if not arrays:
            axis.text(.5, .05, 'Initial scene only', transform=axis.transAxes, ha='center', fontsize=7, color='#a32525',
                      bbox={'facecolor': 'white', 'alpha': .8, 'edgecolor': 'none'})
    for axis in axes.flat[len(cases):]:
        axis.axis('off')
    fig.suptitle(f"{len(cases)} exploratory cases · {len(cases[0][2])} aircraft · XY approach overview\n"
                 'Gold star = target · gray = original obstacles · colors = drones · dots = initial positions\n'
                 'Only original obstacles are shown. Rejected cases have no trajectory.', fontsize=17)
    fig.savefig(output, dpi=120)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--num-drones', type=int, choices=range(1, 6), default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output directory must be new')
    cases = load_cases(args.campaign, args.num_drones)
    if len(cases) != 46:
        parser.error(f'expected 46 exploratory cases, found {len(cases)}')
    args.output.mkdir(parents=True)
    images = args.output / 'images'
    images.mkdir()
    cards = []
    with PdfPages(args.output / 'plans.pdf') as pdf:
        for index, case in enumerate(cases, 1):
            row = case[0]
            fig = full_page(case, index)
            name = f'{index:02d}-{row["id"]}.png'
            fig.savefig(images/name, dpi=120)
            pdf.savefig(fig)
            plt.close(fig)
            cards.append(f'<article data-status="{row["status"]}" data-family="{row["family"]}">'
                f'<h2>{index:02d} · {html.escape(row["id"])}</h2>'
                f'<p>{html.escape(row["status"])} · {len(case[3].obstacles)} original obstacles</p>'
                f'<a href="images/{name}" target="_blank"><img src="images/{name}" loading="lazy" '
                f'alt="Saved scene {html.escape(row["id"])}"></a></article>')
            print(f'{index}/46 {row["id"]}', flush=True)
    overview(cases, args.output / 'overview.png')
    valid = sum(c[0]['status'] == 'validated' for c in cases)
    document = '<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
    document += '<title>Saved trajectory plans</title><style>body{font:16px system-ui;margin:24px;background:#f3f6fa;color:#182535}'
    document += 'main{max-width:1600px;margin:auto}article{background:white;padding:16px;border-radius:10px;margin:20px 0}'
    document += 'img{width:100%;height:auto}h2{font-size:18px}select{padding:8px;margin:8px}header{position:sticky;top:0;background:#f3f6fa;padding:10px;z-index:2}</style><main>'
    document += f'<header><h1>{len(cases)} saved cases · {args.num_drones} aircraft</h1><p>{valid} validated plans; {len(cases)-valid} rejected cases show their initial scenes only. '
    document += '3D views focus on arrival. XY and altitude panels show the full horizon. No obstacles were added.</p>'
    document += '<a href="plans.pdf">Download all 46 pages (PDF)</a> · <a href="overview.png">Overview</a><br>'
    document += '<label>Status <select id="status"><option value="all">All cases</option><option value="validated">Validated only</option><option value="failed">Rejected only</option></select></label>'
    document += '<label>Family <select id="family"><option value="all">All families</option>'
    document += ''.join(f'<option>{name}</option>' for name in dict.fromkeys(c[0]['family'] for c in cases))+'</select></label><span id="shown"></span></header>'
    document += ''.join(cards)
    document += '<script>function filter(){let n=0;document.querySelectorAll("article").forEach(a=>{let s=document.getElementById("status").value,f=document.getElementById("family").value;let show=(s==="all"||(s==="failed"?a.dataset.status!=="validated":a.dataset.status===s))&&(f==="all"||a.dataset.family===f);a.hidden=!show;n+=show});document.getElementById("shown").textContent=n+" cases shown"}document.querySelectorAll("select").forEach(x=>x.onchange=filter);filter()</script></main></html>'
    (args.output/'index.html').write_text(document)
    (args.output/'index.json').write_text(json.dumps({'num_drones': args.num_drones, 'cases': [c[0]['id'] for c in cases],
        'statuses': dict(Counter(c[0]['status'] for c in cases)), 'scope': 'saved nominal plans only; failed cases show input scene'}, indent=2))
    print(f'SAVED {args.output} · {valid} validated / {len(cases)} cases', flush=True)


if __name__ == '__main__':
    main()
