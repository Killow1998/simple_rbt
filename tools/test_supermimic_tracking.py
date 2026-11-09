#!/usr/bin/env python3
"""Compare native and tracked Go2W policy response on an external flat fixture."""
import argparse
import concurrent.futures
import json
import math
import statistics
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--harness', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--index', type=int, default=1)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    base = json.loads(args.contract.read_text())
    speeds = [-.3, -.1, -.03, -.01, 0., .01, .03, .1, .3]
    cases = []
    for value in speeds:
        label = f'{value:+.3f}'
        config = json.loads(json.dumps(base))
        config['rows'][args.index]['command'] = [0, value, 0]
        contract = output / f'{label}.json'
        contract.write_text(json.dumps(config))
        cases.extend((mode, value, contract, False) for mode in ('native', 'tracked'))
    for value in (-.03, .03):
        config = json.loads(json.dumps(base))
        config['rows'][args.index]['command'] = [.2, value, 0]
        config['rows'][args.index]['release_lateral_only'] = True
        contract = output / f'forward-{value:+.3f}.json'
        contract.write_text(json.dumps(config))
        cases.append(('tracked', value, contract, True))

    def run(case):
        mode, desired, contract, partial_release = case
        directory = output / f'{mode}-{desired:+.3f}{"-forward" if partial_release else ""}'
        command = [str(args.harness.resolve()), str(args.profile.resolve()), str(contract),
                   str(args.index), str(directory), '0', mode]
        subprocess.run(command, check=True, capture_output=True, text=True)
        report = json.loads((directory / 'report.json').read_text())
        rows = json.loads((directory / 'trajectory.json').read_text())
        if not report['success'] or len(rows) < 550:
            return {'mode': mode, 'command_vy': desired, 'completed': False,
                    'reason': report['reason'], 'frames': len(rows),
                    'tracker_updates': report['tracker_updates'], 'tracking_passed': False}
        active = [row['state'][5] for row in rows[200:400]]
        stop = [abs(row['state'][5]) if partial_release else
                math.hypot(row['state'][4], row['state'][5]) for row in rows[500:550]]
        mean = statistics.mean(active)
        error = statistics.mean(abs(value - desired) for value in active)
        stop_speed = statistics.mean(stop)
        forward_speed = (statistics.mean(row['state'][4] for row in rows[500:550])
                         if partial_release else None)
        tilt = max(math.acos(max(-1., min(1., -row['state'][3]))) for row in rows)
        passed = (report['success'] and abs(mean - desired) <= max(.005, abs(desired) * .25)
                  and error <= .05 and stop_speed <= .04 and tilt < .8
                  and (not partial_release or abs(forward_speed - .2) <= .04))
        return {'mode': mode, 'command_vy': desired, 'mean_vy': mean, 'mae_vy': error,
                'stop_speed': stop_speed, 'max_tilt_rad': tilt,
                'release_lateral_only': partial_release, 'forward_speed_after_release': forward_speed,
                'tracker_updates': report['tracker_updates'], 'completed': report['success'],
                'tracking_passed': passed}

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, cases))
    passed = (all(row['completed'] for row in results) and
              all(row['tracking_passed'] for row in results if row['mode'] == 'tracked'))
    (output / 'summary.json').write_text(json.dumps({'passed': passed, 'cases': results}, indent=2)+'\n')
    for row in results:
        if row['completed']:
            print(f"{row['mode']:7} {row['command_vy']:+.3f} -> {row['mean_vy']:+.4f} m/s "
                  f"MAE={row['mae_vy']:.4f} stop={row['stop_speed']:.4f}")
        else:
            print(f"{row['mode']:7} {row['command_vy']:+.3f} "
                  f"terminated={row['reason']} after {row['frames']} frames")
    if not passed:
        raise SystemExit('Tracked lateral acceptance failed; see summary.json')


if __name__ == '__main__':
    main()
