#!/usr/bin/env python3
"""Refine two MEI camera extrinsics from a fixed opening LiDAR/IMU interval."""
import argparse
import sys

from dual_mei.pipeline import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="version 1 or 2 YAML configuration")
    parser.add_argument("--output", help="legacy override of YAML output.directory")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check-static-only", action="store_true", help="check static interval without writing artifacts")
    group.add_argument("--prepare-only", action="store_true", help="save review inputs and initial overlays without final extrinsics")
    args = parser.parse_args()
    try:
        return run(args.config, args.check_static_only, args.prepare_only, args.output)
    except KeyboardInterrupt:
        print("Interrupted; incomplete calibration was not published", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
