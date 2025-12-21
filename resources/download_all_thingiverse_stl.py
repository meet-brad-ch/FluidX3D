#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Download all STL files required by FluidX3D examples.

This script downloads all STL models from Thingiverse needed for FluidX3D examples.
Run this once to populate the resources/stl/ directory.

Requirements:
    pip install playwright
    playwright install chromium

Usage:
    python download_all_thingiverse_stl.py
    python download_all_thingiverse_stl.py --force  # Re-download existing files
"""

import os
import sys
import time
from pathlib import Path

# Add current directory to path to import download_thingiverse_stl
sys.path.insert(0, str(Path(__file__).parent))

try:
    from download_thingiverse_stl import download_thingiverse_stl
except ImportError:
    print("Error: Could not import download_thingiverse_stl.py", file=sys.stderr)
    print("Make sure download_thingiverse_stl.py is in the same directory.", file=sys.stderr)
    sys.exit(1)


# List of all STL files to download
# Format: (thing_id, stl_filename)
STL_FILES = [
    ("1625155", "BELL222__FIXED.stl"),         # Bell 222 helicopter
    ("2772812", "techtris_airplane.stl"),      # Boeing 747 (techtris model)
    ("814319", "Airplane.stl"),                # Cessna 172
    ("1176931", "concord_cut_large.stl"),      # Concorde
    ("182114", "Cow_t.stl"),                   # Cow
    ("3014759", "edf_v39.stl"),                # EDF fighter jet (part 1)
    ("3014759", "edf_v391.stl"),               # EDF fighter jet (part 2)
    ("6113", "FAN_Solid_Bottom.stl"),          # Radial fan
    ("4975964", "Full_Shuttle.stl"),           # Space Shuttle
    ("4912729", "StarShipV2.stl"),             # SpaceX Starship
    ("2919109", "DWG_Tie_Fighter_Assembled_02.stl"),  # TIE Fighter
    ("353276", "X-Wing.stl"),                  # X-Wing
]


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='Download all STL files required by FluidX3D examples',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('-f', '--force', action='store_true',
                        help='Force re-download even if files exist')

    args = parser.parse_args()

    # Get script directory (resources/) as output directory
    script_dir = Path(__file__).parent
    output_dir = script_dir  # Output directly to resources/

    print("=" * 60)
    print("FluidX3D STL Downloader")
    print("=" * 60)
    print(f"Output directory: {output_dir}")
    print()

    total = len(STL_FILES)
    downloaded = 0
    skipped = 0
    failed = 0

    print(f"Found {total} STL files to download")
    print()

    for idx, (thing_id, stl_name) in enumerate(STL_FILES, 1):
        print(f"[{idx}/{total}] {stl_name}")
        print(f"          Thing ID: {thing_id}")

        # Check if already exists
        stl_path = output_dir / stl_name
        if stl_path.exists() and not args.force:
            size_mb = stl_path.stat().st_size / 1024 / 1024
            print(f"          Status: Already exists ({size_mb:.1f} MB)")
            skipped += 1
        else:
            print(f"          Status: Downloading...")

            # Remove if forcing re-download
            if args.force and stl_path.exists():
                stl_path.unlink()

            # Download
            success = download_thingiverse_stl(thing_id, stl_name, str(output_dir))

            if success:
                size_mb = stl_path.stat().st_size / 1024 / 1024
                print(f"          Status: Downloaded ({size_mb:.1f} MB)")
                downloaded += 1
            else:
                print(f"          Status: FAILED")
                print(f"          Manual download: https://www.thingiverse.com/thing:{thing_id}/files")
                failed += 1

        print()

    # Print summary
    print("=" * 60)
    print("Download Summary")
    print("=" * 60)
    print(f"Total:      {total}")
    print(f"Downloaded: {downloaded}")
    print(f"Skipped:    {skipped}")
    print(f"Failed:     {failed}")
    print("=" * 60)

    if failed > 0:
        print()
        print("Some downloads failed. Please download them manually from:")
        print("  https://www.thingiverse.com/")
        print(f"and place them in: {output_dir}/")
        return 1

    print()
    print("All STL files ready!")
    print(f"Location: {output_dir}/")
    return 0


if __name__ == '__main__':
    sys.exit(main())
