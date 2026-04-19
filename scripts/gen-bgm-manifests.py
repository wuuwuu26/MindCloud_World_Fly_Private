#!/usr/bin/env python3
# Copyright 2026 Manifold Tech Ltd.
# Licensed under the Apache License, Version 2.0.
"""
Regenerate manifest.json for each BGM playlist folder under asset/music/.

Browsers cannot enumerate a remote directory, so BGM discovery in main.js
tries two strategies:
  1. Fetch the folder URL expecting an HTTP directory listing (works with
     Python's http.server, node http-server, and most dev servers).
  2. Fall back to fetching manifest.json in the folder (works everywhere,
     including static hosts like GitHub Pages / Netlify).

This script keeps manifest.json in sync with whatever FLAC/MP3/OGG/WAV/M4A
files are actually on disk. Run it whenever you add or remove a track:

    python scripts/gen-bgm-manifests.py

Or wire it as a pre-commit hook.
"""
from __future__ import annotations

import json
import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
MUSIC_DIR = REPO_ROOT / "asset" / "music"
AUDIO_EXTENSIONS = {".flac", ".mp3", ".ogg", ".wav", ".m4a"}


def regen(folder: pathlib.Path) -> int:
    """Write ``folder/manifest.json`` listing every audio file in ``folder``.

    Returns the track count.
    """
    tracks = sorted(
        p.name
        for p in folder.iterdir()
        if p.is_file() and p.suffix.lower() in AUDIO_EXTENSIONS
    )
    manifest_path = folder / "manifest.json"
    payload = {"tracks": tracks}
    manifest_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    rel = manifest_path.relative_to(REPO_ROOT)
    print(f"  {rel}: {len(tracks)} track(s)")
    return len(tracks)


def main() -> int:
    if not MUSIC_DIR.is_dir():
        print(f"error: {MUSIC_DIR} does not exist", file=sys.stderr)
        return 1

    total_folders = 0
    total_tracks = 0
    print(f"Scanning {MUSIC_DIR.relative_to(REPO_ROOT)}/ ...")
    for sub in sorted(MUSIC_DIR.iterdir()):
        if sub.is_dir():
            total_folders += 1
            total_tracks += regen(sub)

    if total_folders == 0:
        print("  (no subdirectories found — nothing to generate)")
    else:
        print(f"Done: {total_tracks} track(s) across {total_folders} folder(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
