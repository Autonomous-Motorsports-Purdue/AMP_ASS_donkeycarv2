#!/usr/bin/env python3
"""Convert a KML file to one latitude,longitude CSV per Placemark.

Each <Placemark>'s <coordinates> block (lon,lat,alt space-separated) becomes a
CSV with columns latitude,longitude — matching the format of 3rd_line.csv.
"""
import argparse
import csv
import os
import re
import xml.etree.ElementTree as ET


NS = {"kml": "http://www.opengis.net/kml/2.2"}


def slugify(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_-]+", "_", name.strip()).strip("_")
    return s or "placemark"


def parse_coords(text: str):
    rows = []
    for token in text.split():
        parts = token.split(",")
        if len(parts) < 2:
            continue
        lon, lat = float(parts[0]), float(parts[1])
        rows.append((lat, lon))
    return rows


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["latitude", "longitude"])
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="Input KML file")
    ap.add_argument("-o", "--outdir", default=".", help="Output directory")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    tree = ET.parse(args.input)
    root = tree.getroot()

    used = {}
    count = 0
    for i, pm in enumerate(root.iter("{%s}Placemark" % NS["kml"])):
        name_el = pm.find("kml:name", NS)
        name = slugify(name_el.text if name_el is not None and name_el.text else f"placemark_{i}")
        n = used.get(name, 0)
        used[name] = n + 1
        fname = f"{name}.csv" if n == 0 else f"{name}_{n}.csv"

        for coords_el in pm.iter("{%s}coordinates" % NS["kml"]):
            rows = parse_coords(coords_el.text or "")
            if not rows:
                continue
            out = os.path.join(args.outdir, fname)
            write_csv(out, rows)
            print(f"Wrote {len(rows):4d} points -> {out}")
            count += 1
            break

    print(f"Done: {count} CSV(s) written to {args.outdir}")


if __name__ == "__main__":
    main()
