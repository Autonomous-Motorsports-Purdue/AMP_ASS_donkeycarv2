#!/usr/bin/env python3
"""Convert KML-style coordinates (lon,lat,alt space-separated) to latitude,longitude CSV."""
import argparse
import csv
import sys


def convert(text: str):
    rows = []
    for token in text.replace("\n", " ").split():
        parts = token.split(",")
        if len(parts) < 2:
            continue
        lon, lat = float(parts[0]), float(parts[1])
        rows.append((lat, lon))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", help="Input file (KML coords). Omit to read stdin.")
    ap.add_argument("-o", "--output", required=True, help="Output CSV path")
    args = ap.parse_args()

    text = open(args.input).read() if args.input else sys.stdin.read()
    rows = convert(text)

    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["latitude", "longitude"])
        w.writerows(rows)

    print(f"Wrote {len(rows)} points to {args.output}")


if __name__ == "__main__":
    main()
