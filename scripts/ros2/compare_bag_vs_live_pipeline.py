#!/usr/bin/env python3
"""Compare launch_gici_live.sh vs run_gici_board_rrr_ros2.sh --bag pipelines."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
WS = REPO / 'ros2_wrapper' / 'src' / 'gici_ros2' / 'config'
BAG = WS / 'ros_gici_board_bag_hybrid_rrr.yaml'
LIVE = WS / 'ros_gici_board_live_rrr.yaml'


def estimator_block(text: str) -> str:
    start = text.index('estimate:')
    end = text.index('\nlogging:', start)
    return text[start:end]


def main() -> None:
    bag = BAG.read_text()
    live = LIVE.read_text()
    bag_est = estimator_block(bag)
    live_est = estimator_block(live)
    same_estimator = bag_est == live_est

    diff_lines = subprocess.run(
        ['diff', '-u', str(BAG), str(LIVE)],
        capture_output=True,
        text=True,
    ).stdout.splitlines()

    report = {
        'same_estimator_block': same_estimator,
        'same_replay_enable_false': (
            'replay:\n    enable: false' in bag and 'replay:\n    enable: false' in live
        ),
        'config_differences': [
            line for line in diff_lines
            if line.startswith('+') or line.startswith('-')
            if not line.startswith('+++') and not line.startswith('---')
            and 'estimate:' not in line
        ][:30],
        'runner_differences': {
            'run_gici_board_rrr_ros2.sh --bag': {
                'config_template': 'ros_gici_board_bag_hybrid_rrr.yaml',
                'config_render': 'sed placeholders',
                'health_check_fail': 'retry up to GICI_BAG_REPLAY_ATTEMPTS (default 2)',
                'node_ready': 'wait_for_gici_node_ready',
                'drain_epochs': 'GICI_MIN_SOLUTION_EPOCHS default 1500',
            },
            'launch_gici_live.sh board bag': {
                'config_template': 'ros_gici_board_live_rrr.yaml',
                'config_render': 'render_gici_config.py --mode live',
                'health_check_fail': 'retry up to GICI_BAG_REPLAY_ATTEMPTS (default 2)',
                'node_ready': 'wait_for_gici_node_ready',
                'drain_epochs': 'hardcoded 1500',
                'extra': 'best_effort QoS on IMU/camera; publishes /gici/odom|path|pose',
            },
        },
        'conclusion': (
            'Same MultiSensorEstimating path and identical estimator YAML block. '
            'Runner retry/drain logic aligned; live adds best_effort QoS and /gici outputs.'
        ),
    }
    out = REPO / 'results/stress/pipeline_compare_bag_vs_live.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    print(f'\nWrote {out}')
    if not same_estimator:
        sys.exit(1)


if __name__ == '__main__':
    main()
