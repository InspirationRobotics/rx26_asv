# Offline street basemap

The ground station's two map canvases (Map tab, Mission tab) draw a street/water
basemap of the competition area (Marina Bay, Singapore) underneath the boat, buoys
and waypoints. It needs no internet at run time.

## Why vectors, not tiles

The OpenStreetMap tile usage policy forbids what an offline tile cache would need:

- "You must not: Bulk download ('scrape') tiles or offer prefetch features"
- "Offline use is not permitted on tile.openstreetmap.org"

OSM **data**, on the other hand, is ODbL and may be used offline with attribution.
So one Overpass query is compacted into `basemap.json`, committed in
`crusader_groundstation/crusader_groundstation/laptop/`, and drawn by the GUI's own
canvas renderer (`map.js`). The ground station serves it as `GET /basemap.json`; if
the file is missing the server answers 404 and the map simply has no basemap.

## Refreshing or adding an area

Windows laptop, needs internet, one request (be polite to Overpass, do not loop):

    python tools/scripts/fetch_basemap.py [--bbox S,W,N,E] [--out PATH]

Default bbox is `1.2720,103.8460,1.2920,103.8660` (about 2.2 km) and the default
output is the file above. The script prints the size and feature counts; it
tightens itself (drops small buildings, coarser simplification) to stay under
700 KB. `--from-file` compacts an already saved Overpass response instead.
The JSON schema is documented at the top of the script. Only one area is drawn at a
time; to cover another site, fetch it with its own bbox into `basemap.json`.
Tests: `python -m pytest crusader_groundstation/test/test_basemap.py -q`.

## Attribution (required)

"© OpenStreetMap contributors" (ODbL) is drawn at the bottom-right of each
canvas whenever any basemap feature is on screen. Do not remove or hide it, and keep
the `attribution` field in the JSON.
