#pragma once

#include <filesystem>

// Import road networks from OpenStreetMap XML (.osm) exports.
//
// FAR-FUTURE / LOW-PRIORITY MODULE — parked here to keep the idea visible
// in the codebase. Not required for the core engine, editor, or renderer
// to work. Come back to this once the hand-rolled JSON map format,
// simulation engine, and editor are stable.
//
// Scope for a first pass (deliberately simplified):
//   - Parse .osm XML: <node> (points, incl. traffic_signals) and
//     <way> (polylines with tags: highway, lanes, maxspeed, oneway).
//   - Map OSM tags -> our types:
//       way.highway            -> Lane (one or more, depending on oneway)
//       way.lanes               -> Lane::num_sublanes
//       way.maxspeed             -> Lane::speed_limit
//       node with traffic_signals -> TrafficSignal
//   - Project lat/lon (WGS84) to local planar metres (simple equirectangular
//     projection around a reference point is enough at city-district scale;
//     no need for full UTM initially).
//
// Deliberately OUT of scope for v1 (revisit later if it stops being boring):
//   - True multi-level interchanges: OSM only gives `layer`/`bridge`/`tunnel`
//     tags, not real 3D geometry — reconstructing which road physically
//     passes over another requires heuristics, not just tag reading.
//   - Precise per-lane geometry from `turn:lanes` — often missing/inconsistent
//     in real data; will need sane defaults.
//   - Anything beyond a bounding-box sized city district (PBF binary format,
//     streaming parse, Overpass API querying) — XML + full-file parse is
//     enough for our use case.
//
// Source data: exported via https://www.openstreetmap.org/export for small
// areas, or the Overpass API for targeted bounding-box queries.

namespace ts {

class World;  // forward declaration, defined in core/world.hpp

namespace osm {

// TODO: World import_osm_xml(const std::filesystem::path& osm_file);
// TODO: struct ImportOptions — reference lat/lon for projection origin,
//       default lane count when `lanes` tag is missing, etc.

}  // namespace osm
}  // namespace ts
