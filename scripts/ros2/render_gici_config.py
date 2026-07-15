#!/usr/bin/env python3
"""Render a GICI ROS2 YAML template with path placeholders."""
from __future__ import annotations

import argparse
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
WS = REPO / 'ros2_wrapper' / 'src' / 'gici_ros2' / 'config'

TEMPLATES = {
    'postfile': WS / 'ros_gici_board_postfile_rrr.yaml',
    'bag': WS / 'ros_gici_board_bag_hybrid_rrr.yaml',
    'live': WS / 'ros_gici_board_live_rrr.yaml',
    'urbannav-live': WS / 'ros_urbannav_live_rrr.yaml',
}


def render(template: Path, out: Path, mapping: dict[str, str]) -> None:
    text = template.read_text()
    for key, value in mapping.items():
        text = text.replace(key, value)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(f'Rendered {out}')


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=sorted(TEMPLATES), required=True)
    p.add_argument('--template', type=Path, help='override template yaml')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--dataset-dir', default='')
    p.add_argument('--gici-root', default=str(REPO))
    p.add_argument('--output-dir', required=True)
    p.add_argument('--log-dir', default='')
    p.add_argument('--rtcm-start', default='')
    args = p.parse_args()
    template = args.template or TEMPLATES[args.mode]
    log_dir = args.log_dir or str(Path(args.output_dir) / 'log')

    if args.mode == 'urbannav-live':
        render(
            template,
            args.out,
            {
                'OUTPUT_DIR': args.output_dir,
                'LOG_DIR': log_dir,
            },
        )
        return

    if not args.dataset_dir:
        raise SystemExit('--dataset-dir required for board modes')
    if not args.rtcm_start:
        raise SystemExit('--rtcm-start required for board modes')
    render(
        template,
        args.out,
        {
            'DATASET_DIR': args.dataset_dir,
            'GICI_ROOT': args.gici_root,
            'OUTPUT_DIR': args.output_dir,
            'LOG_DIR': log_dir,
            'RTCM_START': args.rtcm_start,
        },
    )


if __name__ == '__main__':
    main()
