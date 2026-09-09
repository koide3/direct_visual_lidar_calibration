#!/usr/bin/env python3
"""Review a saved dual-MEI calibration without the original bag or optimizer."""
import argparse
import sys
from pathlib import Path

import yaml

from dual_mei.viewer import export_result, show_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", required=True, help="directory containing extrinsics.yaml or prepared.yaml")
    parser.add_argument("--config", help="optional YAML with a visualization section to override saved display settings")
    parser.add_argument("--render-only", action="store_true", help="export overlays without opening a window")
    parser.add_argument("--output", help="overlay export directory; defaults to result directory")
    args = parser.parse_args()
    try:
        overrides = None
        if args.config:
            config = yaml.safe_load(Path(args.config).expanduser().read_text())
            if not isinstance(config, dict) or not isinstance(config.get("visualization"), dict):
                raise ValueError("display configuration must contain a visualization mapping")
            overrides = config["visualization"]
        if args.output and not args.render_only:
            raise ValueError("--output is used with --render-only")
        if args.render_only:
            stats = export_result(args.result, overrides, args.output)
            for name, stages in stats.items():
                for stage, values in stages.items():
                    print(f"{name} {stage}: {values}")
        else:
            show_result(args.result, overrides)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
