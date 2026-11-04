"""Offline M4 file CLI. No ROS, detector, device or task controller is started."""
import argparse
import json
import os
from pathlib import Path
import sys

from perception.offline_memory import OfflineMemory


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("import", "query", "revise"))
    parser.add_argument("--store", type=Path, required=True)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--category")
    parser.add_argument("--out", type=Path, help="optional new result file, never overwritten")
    parser.add_argument("--association-m", type=float, help="new store only")
    args = parser.parse_args(argv)
    try:
        if args.out and args.out.exists():
            raise ValueError("output already exists")
        if args.command != "query" and args.input is None:
            raise ValueError("--input required")
        if args.store.exists():
            memory = OfflineMemory.load(args.store)
            if args.association_m is not None:
                raise ValueError("configuration is frozen in existing store")
        elif args.command == "import":
            memory = OfflineMemory({"association_m": args.association_m} if args.association_m is not None else None)
        else:
            raise FileNotFoundError(args.store)
        if args.command == "query":
            result = memory.query(args.category)
        else:
            document = json.loads(args.input.read_text(encoding="utf8"))
            result = memory.import_document(document) if args.command == "import" else memory.revise(document)
            memory.save(args.store)
        result["process_id"] = os.getpid()
        result["store"] = str(args.store.absolute())
        encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.out:
            with args.out.open("x", encoding="utf8") as stream:
                stream.write(encoded)
        else:
            print(encoded, end="")
        return 0
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps({"ok": False, "error": type(exc).__name__, "reason": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
