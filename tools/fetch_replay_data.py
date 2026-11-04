#!/usr/bin/env python3
"""Fetch the official TUM desk archive, verify gzip/tar, safely extract files.

Uses ordinary urllib proxy settings. Use --direct only when a configured proxy
fails and direct Internet access is authorized. Does not overwrite existing data.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import urllib.request


def path_within(path, root):
    """``PurePath.is_relative_to`` for Python 3.8, the declared reference
    interpreter; the standard-library method only exists from Python 3.9.

    This is the CPython 3.9 implementation, so it keeps the exact semantics: a
    purely lexical parts-prefix test that touches no filesystem, does not resolve
    symlinks and does not collapse ``..``, and that answers ``False`` (rather than
    raising) for a relative/absolute mix. The caller below resolves both sides
    first, exactly as it must with the standard-library method.
    """
    try:
        Path(path).relative_to(root)
    except ValueError:
        return False
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--archive", type=Path, help="existing COMPLETE official archive, skip download")
    p.add_argument("--direct", action="store_true")
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    url = "https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz"
    archive = args.archive or args.out/"desk.tgz"
    if args.archive is None:
        handler = urllib.request.ProxyHandler({}) if args.direct else urllib.request.ProxyHandler()
        with urllib.request.build_opener(handler).open(url, timeout=60) as source, archive.open("xb") as f:
            while block := source.read(1024*1024): f.write(block)
    h = hashlib.sha256()
    with archive.open("rb") as f:
        while block := f.read(1024*1024): h.update(block)
    with tarfile.open(archive) as tar:
        for member in tar:
            target = (args.out/member.name).resolve()
            if not path_within(target, args.out.resolve()) or not (member.isfile() or member.isdir()):
                raise ValueError("archive contains non-regular or escaping member")
        tar.extractall(args.out, filter="data")
    (args.out/"SOURCE.json").write_text(json.dumps({"source_url": url, "archive": str(archive.resolve()),
        "bytes": archive.stat().st_size, "sha256": h.hexdigest(), "checksum_scope": "local instance identifier; not publisher authentication",
        "format_source": "https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats"}, indent=2)+"\n")
    print(args.out/"rgbd_dataset_freiburg1_desk")


if __name__ == "__main__": main()
