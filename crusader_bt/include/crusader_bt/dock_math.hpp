// dock_math.hpp — every calculation Task 3 needs, and nothing else.
//
// Header-only, stdlib only, no ROS, no BehaviorTree.CPP (same contract as
// nav_math.hpp), so it builds and runs on a laptop in a second:
//
//     g++ -std=c++17 -O2 -I include -o t test/test_dock_math.cpp && ./t
//
// The Task 3 leaves (src/task3_leaves.cpp) read the Context, call in here, and publish.
//
// Colours: three numbering schemes meet here and DISAGREE (the CV's DockBay and
// DockWindow put OFF at 1 and RED at 2; RoboCommand's Color puts RED at 1), so every crossing is a named function with a test. Bay identity comes
// from WHERE a bay is, never from the CV's per-frame bay_index.
//
// CONVENTIONS (the same as nav_math, and the same order they bite):
//
//   World is nav::Vec2: metres, x EAST, y NORTH, about the mission origin.
//   Heading is compass degrees, clockwise from north.
//   Camera and body frames are REP-103: x FORWARD, y LEFT. A bearing in the
//   camera frame is therefore POSITIVE TO PORT - the opposite sense to compass.
//   cam_yaw_deg is + to PORT, as in target_tracker, so the camera's boresight
//   points at compass (heading - cam_yaw_deg).
#ifndef CRUSADER_BT__DOCK_MATH_HPP_
#define CRUSADER_BT__DOCK_MATH_HPP_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{
namespace dock
{

using nav::Vec2;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ------------------------------------------------------------------ colours

/// A light's colour, numbered EXACTLY as the CV's draft DockBay.COLOUR_* and
/// DockWindow.STATE_* constants, so a message field converts with
/// colourFromCv() and nothing else.
enum class Colour
{
  Unknown = 0,   // the colour rule abstained - a real answer, not an error
  Off = 1,
  Red = 2,
  Green = 3,
  Blue = 4
};

/// DockBay.indicator_colour / DockWindow.state -> Colour. Out of range is
/// Unknown rather than a cast: a new constant on the CV side must not become a
/// colour here by accident.
inline Colour colourFromCv(int v)
{
  return (v >= 0 && v <= 4) ? static_cast<Colour>(v) : Colour::Unknown;
}

/// DockObservation.target_colours are STRINGS ("red", "blue"), not the enum.
inline Colour colourFromName(const std::string & s)
{
  std::string l;
  for (char c : s) {l += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}
  if (l == "red") {return Colour::Red;}
  if (l == "green") {return Colour::Green;}
  if (l == "blue") {return Colour::Blue;}
  if (l == "off" || l == "black") {return Colour::Off;}
  return Colour::Unknown;
}

inline bool isLit(Colour c)
{
  return c == Colour::Red || c == Colour::Green || c == Colour::Blue;
}

inline const char * colourName(Colour c)
{
  switch (c) {
    case Colour::Off: return "off";
    case Colour::Red: return "red";
    case Colour::Green: return "green";
    case Colour::Blue: return "blue";
    default: return "unknown";
  }
}

// RoboCommand's Color (rx_common.proto) is NOT the CV's numbering: there is no OFF, so
// RED is 1, not 2.
constexpr int kWireUnknown = 0;   // never send - "not yet identified"
constexpr int kWireRed = 1;
constexpr int kWireGreen = 2;
constexpr int kWireBlue = 3;
constexpr int kWireAny = 4;       // Advanced tier: any tin will do

/// Colour -> RoboCommand Color. OFF and UNKNOWN map to 0, which the
/// OCS validator refuses; callers must check wireColour(c) != 0 before sending.
inline int wireColour(Colour c)
{
  switch (c) {
    case Colour::Red: return kWireRed;
    case Colour::Green: return kWireGreen;
    case Colour::Blue: return kWireBlue;
    default: return kWireUnknown;
  }
}

/// The protobuf enum NAME, which is what the JSON reports carry (the Task 1
/// report sends "BEACON_STATE_FLASHING_RED", not 2).
inline const char * wireColourName(int w)
{
  switch (w) {
    case kWireRed: return "COLOR_RED";
    case kWireGreen: return "COLOR_GREEN";
    case kWireBlue: return "COLOR_BLUE";
    case kWireAny: return "COLOR_ANY";
    default: return "COLOR_UNKNOWN";
  }
}

// --------------------------------------------------------------- placing

/// Where the camera sits on the hull: camera_link in base_link, REP-103.
/// Defaults are target_tracker's cam_x/cam_y/cam_yaw_deg.
struct Mount
{
  double x = 0.37;         ///< m forward of the body origin
  double y = 0.0;          ///< m to PORT
  double yaw_deg = 0.0;    ///< + = aimed to PORT
  /// + = aimed DOWN, as target_tracker's cam_pitch_deg. From inside a 2 m slip a
  /// level camera 0.41 m up cannot see the upper window
  /// (docs/T3_coordinated_logistics.md), so the mount may end up pitched UP
  /// (negative); CV ranges and normals are then in a tilted frame.
  double pitch_deg = 0.0;
};

/// A vector in the CAMERA frame, levelled: its (forward, left) components in the
/// body's horizontal plane (undoes the mount pitch). Camera forward (1, 0, 0)
/// pitched down by p is (cos p, 0, -sin p) in the body, as in target_tracker.
inline Vec2 levelled(double cx, double cy, double cz, double pitch_deg)
{
  const double p = pitch_deg * nav::kDeg;
  return {cx * std::cos(p) + cz * std::sin(p), cy};
}

/// A point in the body frame (x forward, y left) in the world.
inline Vec2 bodyToWorld(Vec2 boat, double heading_deg, double bx, double by)
{
  const Vec2 f = nav::headingVec(heading_deg);
  return boat + f * bx + nav::portOf(f) * by;
}

/// A DIRECTION in the camera frame (x forward, y left) as a world unit vector.
/// The mount yaw is + to port (counter-clockwise) and compass is clockwise,
/// hence the subtraction.
inline Vec2 camDirToWorld(double heading_deg, double mount_yaw_deg, double cx, double cy)
{
  const Vec2 f = nav::headingVec(heading_deg - mount_yaw_deg);
  return nav::unit(f * cx + nav::portOf(f) * cy);
}

/// Range along the HORIZONTAL ray at `bearing_deg` (camera frame, + left) to
/// the plane n.p + d = 0 (camera frame; DockBay.plane_normal / plane_offset).
///
/// NaN when the ray grazes the plane, points away from it, or the plane is not
/// finite. The ray is horizontal because DockBay carries only a bearing.
inline double rangeToPlane(double nx, double ny, double d, double bearing_deg)
{
  if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(d) ||
    !std::isfinite(bearing_deg))
  {
    return kNaN;
  }
  const double b = bearing_deg * nav::kDeg;
  const double den = nx * std::cos(b) + ny * std::sin(b);
  if (std::abs(den) < 0.05) {return kNaN;}   // within ~3 deg of edge-on
  const double t = -d / den;
  return t > 0.0 ? t : kNaN;
}

/// One window of one bay in one frame (DockWindow, reduced to what we use).
struct WindowSighting
{
  int index = -1;                ///< slot index in the bay design, 0 = leftmost
  Colour state = Colour::Unknown;
  double conf = 0.0;
  bool has_position = false;     ///< aim point below is valid
  double x = 0.0, y = 0.0, z = 0.0;   ///< camera frame, REP-103
  double u_px = kNaN;            ///< its box's centre column, RGB px (DockWindow.bbox)
};

/// One bay face in one frame (DockBay, reduced to what we use).
struct BaySighting
{
  double bearing_deg = kNaN;     ///< face centre, camera frame, + LEFT
  double range_m = kNaN;         ///< plane range, else size range, else NaN
  bool has_normal = false;       ///< plane normal below is valid
  double nx = 0.0, ny = 0.0;     ///< plane normal, camera frame, toward the camera
  double nz = 0.0;               ///< ... and its up component (matters when pitched)
  double d = kNaN;               ///< plane offset: n.p + d = 0, camera frame
  bool truncated = false;        ///< cut by the image edge: centre is biased
  bool indicator_present = false;
  Colour indicator = Colour::Unknown;
  double indicator_conf = 0.0;
  /// The indicator box's centre column, RGB px (DockBay.indicator_bbox); NaN = no box.
  /// DockBay gives the indicator no bearing: Pinhole turns this into one.
  double ind_u_px = kNaN;
  std::vector<WindowSighting> windows;
  int lit_window_index = -1;
};

/// The camera's column -> bearing map, learned from the windows' own aim points.
///
/// The CV puts each window's aim point on the ray through its box's centre pixel
/// (dock_obs_core.pixel_ray: y/x = -(u - cx) / fx), so every positioned window is an
/// EXACT sample of tan(bearing) = a + k u, whatever its depth. A least-squares line
/// through them gives the bearing of any other column of the same image - the
/// indicator's - with no intrinsics passed to the tree and no change to DockBay.
struct Pinhole
{
  double min_spread_px = 40.0;   ///< columns this spread (std dev) before it is trusted
  double n = 0.0, su = 0.0, st = 0.0, suu = 0.0, sut = 0.0;

  void add(double u, double x, double y)
  {
    if (!std::isfinite(u) || !std::isfinite(x) || !std::isfinite(y) || x < 0.1) {return;}
    const double t = y / x;
    n += 1.0; su += u; st += t; suu += u * u; sut += u * t;
  }

  /// tan(bearing) = a + k u; false until the columns are spread and the slope is sane
  /// (+ left: a column further right is a smaller bearing, so k < 0).
  bool line(double & a, double & k) const
  {
    if (n < 2.0) {return false;}
    const double var = suu / n - (su / n) * (su / n);
    if (!(var >= min_spread_px * min_spread_px)) {return false;}
    k = (n * sut - su * st) / (n * suu - su * su);
    a = (st - k * su) / n;
    return std::isfinite(k) && std::isfinite(a) && k < 0.0;
  }
  bool ok() const {double a, k; return line(a, k);}

  /// The bearing of column u, deg, camera frame, + LEFT; NaN until ok().
  double bearingDeg(double u) const
  {
    double a, k;
    if (!std::isfinite(u) || !line(a, k)) {return kNaN;}
    return std::atan(a + k * u) / nav::kDeg;
  }
};

/// A point found in the camera frame (forward, left, levelled), in the BODY frame (ahead,
/// left of the body origin): the mount's offset and yaw (+ = aimed to port).
inline Vec2 camToBody(const Mount & m, Vec2 cam)
{
  const double y = m.yaw_deg * nav::kDeg;
  return {m.x + cam.x * std::cos(y) - cam.y * std::sin(y),
    m.y + cam.x * std::sin(y) + cam.y * std::cos(y)};
}

/// Where a face is, HORIZONTALLY, in the body frame (forward, left), from its
/// plane - exactly, even with the camera pitched.
///
/// The CV's bearing is the face centre's azimuth in the CAMERA frame; with a
/// pitched camera the horizontal ray at that azimuth meets the plane BESIDE the
/// centreline (0.2 m off for a face 2 m to the side at 5 m under 20 deg of pitch;
/// test_dock_math). So take the point on the plane at that azimuth AND at the
/// face centre's height above the camera, `face_dz` (the build guide puts the
/// face centre ~0.8 m above the water); at zero pitch it does not matter. False
/// when the plane is edge-on or behind.
inline bool faceInBody(
  double bearing_deg, double nx, double ny, double nz, double d, double pitch_deg,
  double face_dz, Vec2 & out)
{
  if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz) ||
    !std::isfinite(d) || !std::isfinite(bearing_deg))
  {
    return false;
  }
  const double b = bearing_deg * nav::kDeg, p = pitch_deg * nav::kDeg;
  const double cb = std::cos(b), sb = std::sin(b), cp = std::cos(p), sp = std::sin(p);
  const double a = nx * cb + ny * sb;
  const double den = -(d * cp + face_dz * nz);
  if (std::abs(den) < 1e-9) {return false;}
  const double w = (face_dz * a - d * cb * sp) / den;
  const double nr = a + nz * w;
  if (std::abs(nr) < 0.05) {return false;}          // within ~3 deg of edge-on
  const double t = -d / nr;
  if (t <= 0.0) {return false;}
  out = Vec2{t * (cb * cp + w * sp), t * sb};
  return true;
}

/// DockBay's two ranges -> the one to use. The plane is preferred: it is
/// stereo, and it is what the CV fits RANSAC to. The window-size range is the
/// only one closer than the stereo minimum (0.31-0.62 m) and the fallback when
/// depth is missing.
inline double sightingRange(
  bool has_plane, double nx, double ny, double d, double bearing_deg,
  double range_from_size_m)
{
  if (has_plane) {
    const double r = rangeToPlane(nx, ny, d, bearing_deg);
    if (std::isfinite(r)) {return r;}
  }
  return (std::isfinite(range_from_size_m) && range_from_size_m > 0.0) ?
         range_from_size_m : kNaN;
}

/// One DockObservation: every bay seen in one camera frame, plus the timing
/// layer's verdict for the bay it tracks.
struct Frame
{
  double t = 0.0;                        ///< seconds, the caller's clock
  std::vector<BaySighting> bays;
  std::string target_pattern;            ///< "", steady, flash, code, unresolved
  std::vector<Colour> target_colours;
  int target_window_index = -1;
  std::string last_event;                ///< "", hit, hit_done, lost - THIS frame only
  double observed_fps = 0.0;
};

// --------------------------------------------------------------- the book

struct BookParams
{
  /// Association radius, m. MUST be under half the bay spacing, or two bays
  /// merge into one track and the dock looks two bays wide.
  /// The real dock's bays are 2.0 m apart (1.5 m slips, 0.5 m fingers).
  double gate_m = 0.7;
  /// Two TRACKS closer than this are one bay seen twice, and are merged. The
  /// same number as layout()'s min_spacing_m, so nothing layout() would refuse
  /// as "too close to be two bays" stalls the survey. Under the 2 m pitch.
  double merge_m = 1.2;
  /// With the dock heading known (ingest's square_deg), the gate (gate_m) and the merge
  /// (merge_across_m) are ACROSS the dock, and along it only loosely (along_gate_m): the
  /// camera's range errors run along the ray, about along the heading, and a 0.7 m gate
  /// both ways split one bay into six tracks on the water (2026-11-12).
  /// Across the dock two tracks closer than merge_across_m are one bay seen twice: the
  /// same 1.2 m as merge_m, under the 2 m pitch (with the window-size range calibrated:
  /// at 0.25 m it read 63 %, the bays looked 1.1-1.3 m apart and 0.8 m was needed).
  double merge_across_m = 1.2;
  double along_gate_m = 2.5;
  /// The dock has three bays, so the book holds at most three (Chase, 2026-11-12): a fourth
  /// face is taken only in place of a weak stale track (prune_max_n, stale_s), and
  /// otherwise rejected. Faces outside the course (DockBook::keep_in) never get this far.
  int max_tracks = 3;
  /// A full book makes room for a new bay by dropping its weakest track seen fewer than
  /// prune_max_n times and not for stale_s (a sighting from mid-turn, a reflection).
  int prune_max_n = 3;
  double stale_s = 5.0;
  /// A face is dropped only this far OUTSIDE the course (DockBook::keep_in), m. The dock
  /// may sit at the course's edge - its bays 0.9-1.8 m in on ALPHA, 11-12 - and a size range
  /// that reads 15 % long put them all outside (the replay at window_size_m 0.55).
  double fence_margin_m = 3.0;
  /// Face centre above the CAMERA, m, for placing sightings from a pitched
  /// camera (faceInBody). Build guide: face centre ~0.8 m above the water;
  /// the camera is 0.41 m up. Irrelevant while the camera is level.
  double face_dz = 0.39;
  /// The same for the GREEN/RED indicator, at the base of the structure: about the
  /// camera's own height. Irrelevant while the camera is level.
  double ind_dz = 0.0;
  double max_range_m = 14.0;    ///< beyond this a sighting places nothing
  double min_ind_conf = 0.5;    ///< indicator readings below are not votes
  double ema_alpha = 0.15;      ///< weight of one reading in green_ema
};

/// One physical bay, anchored in the world.
struct BayTrack
{
  int id = -1;                  ///< stable for the mission; NOT the bay number
  Vec2 p;                       ///< weighted mean face centre
  double w = 0.0;               ///< total weight behind p
  int n = 0;                    ///< sightings folded in
  Vec2 normal_sum;              ///< sum of world outward normals from the plane
  int n_normal = 0;
  Vec2 view_sum;                ///< sum of face->camera unit vectors (fallback)
  int red = 0;                  ///< indicator frames read RED
  int green = 0;                ///< indicator frames read GREEN
  int other = 0;                ///< present but unknown/off/blue
  int off = 0;                  ///< of `other`, the frames read OFF (unlit)
  /// Recent-weighted share of GREEN among red/green readings. Close-range
  /// readings arrive last, so an early far misread is overturned here first.
  double green_ema = 0.5;
  /// The same for not-RED among red/green/off readings: verdictOffIsGreen's.
  double nonred_ema = 0.5;
  double first_seen = 0.0;
  double last_seen = 0.0;
  std::vector<WindowSighting> windows;   ///< from the latest sighting
  /// The latest sighting as the camera saw it, in the body frame (ahead, left of the
  /// body origin, m): no heading in it, so no heading error either.
  double live_ahead_m = std::numeric_limits<double>::quiet_NaN();
  double live_left_m = std::numeric_limits<double>::quiet_NaN();
  bool live_truncated = false;           ///< that sighting's face was cut by the image edge
  /// The same sighting's indicator, body frame, where the frame placed it (its box, the
  /// learned Pinhole and the face plane); NaN when it did not.
  double live_ind_ahead_m = std::numeric_limits<double>::quiet_NaN();
  double live_ind_left_m = std::numeric_limits<double>::quiet_NaN();
  int lit_window_index = -1;
  double windows_t = -1.0;

  /// Unit vector from the face OUT to the water the boat approaches from.
  Vec2 outward() const
  {
    if (n_normal > 0 && nav::norm(normal_sum) > 1e-6) {return nav::unit(normal_sum);}
    return nav::unit(view_sum);
  }
  int votes() const {return red + green;}
  double greenShare() const {return votes() > 0 ? static_cast<double>(green) / votes() : 0.0;}
};

/// One bay of the newest frame as that frame placed it, before any track: Task 3 aligns and
/// aims on the GREEN one of these (Context::green), whatever track it lands on.
struct FrameBay
{
  Vec2 p;                                ///< face centre, local frame
  double ahead_m = kNaN, left_m = kNaN;  ///< face centre, body frame (+ left)
  bool truncated = false;                ///< cut by the image edge: centre biased
  bool plane = false;                    ///< placed by the face plane (stereo), not the size range
  int track = -1;                        ///< the track the book put it on (-1: none)
  /// The indicator, body frame, where the frame placed it (its box, the learned Pinhole and
  /// the face plane); NaN when it did not.
  double ind_ahead_m = kNaN, ind_left_m = kNaN;
  Colour indicator = Colour::Unknown;    ///< its reading; Unknown below min_ind_conf
  std::vector<WindowSighting> windows;
};

/// `p` inside the polygon `poly` (either winding; a repeated closing vertex is harmless).
inline bool inPolygon(Vec2 p, const std::vector<Vec2> & poly)
{
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const Vec2 a = poly[i], b = poly[j];
    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {
      in = !in;
    }
  }
  return in;
}

/// `p` more than `margin_m` outside the polygon `poly`; never, for an empty one.
inline bool beyondFence(Vec2 p, const std::vector<Vec2> & poly, double margin_m)
{
  if (poly.empty() || inPolygon(p, poly)) {return false;}
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const Vec2 a = poly[j], ab = poly[i] - a;
    const double len2 = nav::dot(ab, ab);
    const double u = len2 > 0.0 ? std::clamp(nav::dot(p - a, ab) / len2, 0.0, 1.0) : 0.0;
    if (nav::norm(p - (a + ab * u)) <= margin_m) {return false;}
  }
  return true;
}

/// The bays seen so far. Written by the runner on every DockObservation;
/// read by the leaves.
struct DockBook
{
  BookParams prm;
  std::vector<BayTrack> tracks;
  int next_id = 0;
  int frames = 0;
  /// (merged-away id, the id it went into). A merge must not make an id held
  /// elsewhere - the committed bay's, above all - stop resolving: find()
  /// follows these.
  std::vector<std::pair<int, int>> aliases;
  /// Column -> bearing, learned from every positioned window of every frame (the search
  /// along the dock spreads them across the picture long before the approach).
  Pinhole pinhole;
  /// The newest frame's bays, placed (FrameBay); empty when it could place none.
  std::vector<FrameBay> frame_bays;
  /// RoboCommand's course boundary in the local frame (the runner's placeCourse), either
  /// winding; empty = no course, and nothing is checked. A face placed more than
  /// prm.fence_margin_m outside it is not this course's dock (a building across the
  /// water, the shore) and places nothing.
  std::vector<Vec2> keep_in;
  int keep_in_seq = -1;                  ///< the Context::course_seq keep_in was made from
  int fenced = 0;                        ///< faces dropped beyond keep_in, this mission

  /// The id `id` lives under now, after any merges.
  int resolve(int id) const
  {
    for (bool moved = true; moved; ) {
      moved = false;
      for (const auto & a : aliases) {
        if (a.first == id) {id = a.second; moved = true; break;}
      }
    }
    return id;
  }

  /// This frame's face on track `id` (after merges), nullptr when the frame put none there.
  /// A committed bay is followed this way: the track holds its place against range noise
  /// and against the cut faces that drag a single sighting sideways.
  const FrameBay * onTrack(int id) const
  {
    if (id < 0) {return nullptr;}
    id = resolve(id);
    for (const auto & b : frame_bays) {
      if (b.track >= 0 && resolve(b.track) == id) {return &b;}
    }
    return nullptr;
  }

  const BayTrack * find(int id) const
  {
    id = resolve(id);
    for (const auto & t : tracks) {if (t.id == id) {return &t;}}
    return nullptr;
  }
  BayTrack * find(int id)
  {
    id = resolve(id);
    for (auto & t : tracks) {if (t.id == id) {return &t;}}
    return nullptr;
  }

  /// Fold one frame in. Returns, per sighting, the track id it went to (-1 if it
  /// placed nothing: no range, too far, or a truncated face with no track to
  /// join). Association is greedy by distance, one sighting per track per frame:
  /// two bays in one frame are two bays, even if the gate would let both join one
  /// track. `square_deg`: the dock heading (the bow's, facing the bays), NaN when
  /// unset; with it the distance is measured across the dock (gap()).
  std::vector<int> ingest(
    const Frame & f, Vec2 boat, double heading_deg, const Mount & m, double square_deg = kNaN)
  {
    ++frames;
    frame_bays.clear();
    for (const auto & s : f.bays) {
      for (const auto & w : s.windows) {
        if (w.has_position) {pinhole.add(w.u_px, w.x, w.y);}
      }
    }
    std::vector<int> out(f.bays.size(), -1);
    if (!std::isfinite(heading_deg)) {return out;}   // cannot place anything

    const Vec2 cam = bodyToWorld(boat, heading_deg, m.x, m.y);
    struct Placed {std::size_t i; Vec2 q; double r; Vec2 fb;};
    std::vector<Placed> placed;
    for (std::size_t i = 0; i < f.bays.size(); ++i) {
      const auto & s = f.bays[i];
      if (!std::isfinite(s.range_m) || !std::isfinite(s.bearing_deg)) {continue;}
      if (s.range_m <= 0.0 || s.range_m > prm.max_range_m) {continue;}
      // Without the face plane (window-size range, below the stereo minimum) the
      // range is along the camera-frame ray, which a pitched camera tilts out of
      // the horizontal: level it and scale.
      Vec2 fb;
      const bool by_plane = s.has_normal &&
        faceInBody(s.bearing_deg, s.nx, s.ny, s.nz, s.d, m.pitch_deg, prm.face_dz, fb);
      if (!by_plane) {
        const double b = s.bearing_deg * nav::kDeg;
        fb = levelled(std::cos(b), std::sin(b), 0.0, m.pitch_deg) * s.range_m;
      }
      const double hr = nav::norm(fb);
      const Vec2 dir = camDirToWorld(heading_deg, m.yaw_deg, fb.x, fb.y);
      const Vec2 q = cam + dir * hr;
      if (beyondFence(q, keep_in, prm.fence_margin_m)) {++fenced; continue;}
      placed.push_back({i, q, hr, fb});
      frame_bays.push_back(frameBay(s, placed.back().q, fb, m));
      frame_bays.back().plane = by_plane;
    }

    struct Pair {double d; std::size_t pi; std::size_t ti;};
    std::vector<Pair> pairs;
    for (std::size_t pi = 0; pi < placed.size(); ++pi) {
      for (std::size_t ti = 0; ti < tracks.size(); ++ti) {
        const double d = gap(placed[pi].q, tracks[ti].p, square_deg, prm.gate_m);
        if (std::isfinite(d)) {pairs.push_back({d, pi, ti});}
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {return a.d < b.d;});
    std::vector<bool> pused(placed.size(), false), tused(tracks.size(), false);
    for (const auto & pr : pairs) {
      if (pused[pr.pi] || tused[pr.ti]) {continue;}
      pused[pr.pi] = tused[pr.ti] = true;
      const Placed & pl = placed[pr.pi];
      fold(tracks[pr.ti], f.bays[pl.i], pl.q, pl.r, cam, heading_deg, m, f.t);
      live(tracks[pr.ti], frame_bays[pr.pi]);
      out[pl.i] = frame_bays[pr.pi].track = tracks[pr.ti].id;
    }
    // Unmatched: a new bay, unless the face is cut off (its centre is biased
    // toward the image edge, and a track born from it starts in the wrong place).
    for (std::size_t pi = 0; pi < placed.size(); ++pi) {
      if (pused[pi]) {continue;}
      const Placed & pl = placed[pi];
      if (f.bays[pl.i].truncated) {continue;}
      if (static_cast<int>(tracks.size()) >= prm.max_tracks && !pruneOne(f.t)) {continue;}
      BayTrack t;
      t.id = next_id++;
      t.first_seen = f.t;
      tracks.push_back(t);
      fold(tracks.back(), f.bays[pl.i], pl.q, pl.r, cam, heading_deg, m, f.t);
      live(tracks.back(), frame_bays[pi]);
      out[pl.i] = frame_bays[pi].track = tracks.back().id;
    }
    mergeClose(square_deg);
    return out;
  }

  /// How far apart two placed faces are, for association: +inf beyond `limit`. With the
  /// dock heading `square_deg`, the distance ACROSS it (within `limit`) plus a fifth of the
  /// distance along it (within along_gate_m), the fifth only to rank the pairs; without,
  /// the plain distance.
  double gap(Vec2 a, Vec2 b, double square_deg, double limit) const
  {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const Vec2 d = a - b;
    if (!std::isfinite(square_deg)) {
      const double r = nav::norm(d);
      return r < limit ? r : kInf;
    }
    const Vec2 fwd = nav::headingVec(square_deg);
    const double across = std::fabs(nav::dot(d, nav::portOf(fwd)));
    const double along = std::fabs(nav::dot(d, fwd));
    return across < limit && along < prm.along_gate_m ? across + 0.2 * along : kInf;
  }

  /// Room for a new bay in a full book: drop the weakest track seen fewer than
  /// prune_max_n times and not for stale_s. False when there is none to drop.
  bool pruneOne(double now)
  {
    std::size_t k = tracks.size();
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      const BayTrack & t = tracks[i];
      if (t.n >= prm.prune_max_n || now - t.last_seen < prm.stale_s) {continue;}
      if (k == tracks.size() || t.n < tracks[k].n) {k = i;}
    }
    if (k == tracks.size()) {return false;}
    tracks.erase(tracks.begin() + static_cast<std::ptrdiff_t>(k));
    return true;
  }

  /// Where this sighting put the face (`q` local, `fb` camera), and its indicator if the
  /// frame can place it: the indicator's column through the learned Pinhole is its bearing,
  /// and that bearing meets the face plane (faceInBody, at the indicator's height). No
  /// plane, no indicator.
  FrameBay frameBay(const BaySighting & s, Vec2 q, Vec2 fb, const Mount & m) const
  {
    FrameBay b;
    b.p = q;
    const Vec2 face = camToBody(m, fb);
    b.ahead_m = face.x;
    b.left_m = face.y;
    b.truncated = s.truncated;
    const double ib = s.indicator_present ? pinhole.bearingDeg(s.ind_u_px) : kNaN;
    Vec2 ic;
    if (std::isfinite(ib) && s.has_normal &&
      faceInBody(ib, s.nx, s.ny, s.nz, s.d, m.pitch_deg, prm.ind_dz, ic))
    {
      const Vec2 ind = camToBody(m, ic);
      b.ind_ahead_m = ind.x;
      b.ind_left_m = ind.y;
    }
    if (s.indicator_present && s.indicator_conf >= prm.min_ind_conf) {b.indicator = s.indicator;}
    b.windows = s.windows;
    return b;
  }

  /// The track's view of its latest sighting.
  static void live(BayTrack & t, const FrameBay & b)
  {
    t.live_ahead_m = b.ahead_m;
    t.live_left_m = b.left_m;
    t.live_truncated = b.truncated;
    t.live_ind_ahead_m = b.ind_ahead_m;
    t.live_ind_left_m = b.ind_left_m;
  }

  /// Fold together any two tracks closer than merge_m (with the dock heading: closer
  /// than merge_across_m across it, gap()), the lighter into the heavier, until none
  /// are. Everything is summed; the position is the weighted mean, so the merged track
  /// sits where the better sightings say.
  void mergeClose(double square_deg = kNaN)
  {
    const double limit = std::isfinite(square_deg) ? prm.merge_across_m : prm.merge_m;
    for (bool again = true; again; ) {
      again = false;
      for (std::size_t i = 0; i < tracks.size() && !again; ++i) {
        for (std::size_t j = i + 1; j < tracks.size() && !again; ++j) {
          if (!std::isfinite(gap(tracks[i].p, tracks[j].p, square_deg, limit))) {continue;}
          const std::size_t keep = tracks[i].w >= tracks[j].w ? i : j;
          const std::size_t drop = keep == i ? j : i;
          absorb(tracks[keep], tracks[drop]);
          aliases.push_back({tracks[drop].id, tracks[keep].id});
          tracks.erase(tracks.begin() + static_cast<std::ptrdiff_t>(drop));
          again = true;
        }
      }
    }
  }

private:
  static void absorb(BayTrack & k, const BayTrack & d)
  {
    const double w = k.w + d.w;
    if (w > 0.0) {k.p = (k.p * k.w + d.p * d.w) * (1.0 / w);}
    k.w = w;
    const int vk = k.votes(), vd = d.votes();
    if (vk + vd > 0) {k.green_ema = (k.green_ema * vk + d.green_ema * vd) / (vk + vd);}
    const int nk = vk + k.off, nd = vd + d.off;
    if (nk + nd > 0) {k.nonred_ema = (k.nonred_ema * nk + d.nonred_ema * nd) / (nk + nd);}
    k.off += d.off;
    k.n += d.n;
    k.normal_sum = k.normal_sum + d.normal_sum;
    k.n_normal += d.n_normal;
    k.view_sum = k.view_sum + d.view_sum;
    k.red += d.red;
    k.green += d.green;
    k.other += d.other;
    k.first_seen = std::min(k.first_seen, d.first_seen);
    if (d.last_seen > k.last_seen) {          // the newer camera view wins
      k.live_ahead_m = d.live_ahead_m;
      k.live_left_m = d.live_left_m;
      k.live_truncated = d.live_truncated;
      k.live_ind_ahead_m = d.live_ind_ahead_m;
      k.live_ind_left_m = d.live_ind_left_m;
    }
    k.last_seen = std::max(k.last_seen, d.last_seen);
    if (d.windows_t > k.windows_t) {
      k.windows = d.windows;
      k.lit_window_index = d.lit_window_index;
      k.windows_t = d.windows_t;
    }
  }

  void fold(
    BayTrack & t, const BaySighting & s, Vec2 q, double r, Vec2 cam,
    double heading_deg, const Mount & m, double now) const
  {
    // Position: weighted by 1/r^2, so the metre-accurate close sightings
    // outweigh the early far ones. A truncated face still votes and still
    // refreshes the windows, but does not move the track.
    if (!s.truncated) {
      const double wi = 1.0 / std::max(r * r, 1.0);
      t.p = (t.w > 0.0) ? (t.p * t.w + q * wi) * (1.0 / (t.w + wi)) : q;
      t.w += wi;
      if (s.has_normal) {
        const Vec2 nh = levelled(s.nx, s.ny, s.nz, m.pitch_deg);
        const Vec2 nw = camDirToWorld(heading_deg, m.yaw_deg, nh.x, nh.y);
        t.normal_sum = t.normal_sum + nw;
        ++t.n_normal;
      }
      t.view_sum = t.view_sum + nav::unit(cam - q);
    }
    ++t.n;
    t.last_seen = now;

    if (s.indicator_present && s.indicator_conf >= prm.min_ind_conf) {
      if (s.indicator == Colour::Red || s.indicator == Colour::Green) {
        const bool g = s.indicator == Colour::Green;
        (g ? t.green : t.red) += 1;
        t.green_ema = (1.0 - prm.ema_alpha) * t.green_ema + prm.ema_alpha * (g ? 1.0 : 0.0);
      } else {
        ++t.other;
        if (s.indicator == Colour::Off) {++t.off;}
      }
      if (s.indicator == Colour::Red || s.indicator == Colour::Green ||
        s.indicator == Colour::Off)
      {
        const double nr = s.indicator == Colour::Red ? 0.0 : 1.0;
        t.nonred_ema = (1.0 - prm.ema_alpha) * t.nonred_ema + prm.ema_alpha * nr;
      }
    }
    if (!s.windows.empty()) {
      t.windows = s.windows;
      t.lit_window_index = s.lit_window_index;
      t.windows_t = now;
    }
  }
};

// --------------------------------------------------------------- layout

/// The dock as a line of bays, numbered the handbook's way.
struct DockLayout
{
  bool ok = false;
  std::string why;
  Vec2 centre;
  Vec2 out;                 ///< unit, from the faces toward the approach water
  Vec2 right;               ///< unit, LEFT -> RIGHT for someone FACING the bays
  std::vector<int> ids;     ///< track ids, bay 1 first

  /// 1-based bay number of a track, 0 if it is not one of the numbered bays.
  int numberOf(int track_id) const
  {
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == track_id) {return static_cast<int>(i) + 1;}
    }
    return 0;
  }
};

/// Number the bays: the `bays` best-observed tracks with at least `min_obs`
/// sightings, sorted left to right as seen FACING them.
///
/// "Facing the bays" is looking along -out, so left-to-right is the
/// STARBOARD direction of that look: starboardOf(-out). Backwards, and every bay
/// number is mirrored (bay 1 reported as bay 3) while the boat still docks in
/// the right place.
///
/// Refuses (ok=false, with why) rather than guess when: fewer than `bays`
/// tracks are confirmed; the faces do not sit on one line; or two are closer
/// than `min_spacing_m` (one bay seen twice).
inline DockLayout layout(
  const DockBook & b, int min_obs = 5, int bays = 3, double min_spacing_m = 1.2,
  double max_setback_m = 1.0)
{
  DockLayout L;
  std::vector<const BayTrack *> c;
  for (const auto & t : b.tracks) {if (t.n >= min_obs) {c.push_back(&t);}}
  std::sort(c.begin(), c.end(), [](const BayTrack * a, const BayTrack * z) {return a->n > z->n;});
  if (static_cast<int>(c.size()) < bays) {
    L.why = std::to_string(c.size()) + " of " + std::to_string(bays) + " bays confirmed";
    return L;
  }
  c.resize(static_cast<std::size_t>(bays));

  Vec2 osum, csum;
  for (const auto * t : c) {osum = osum + t->outward(); csum = csum + t->p;}
  if (nav::norm(osum) < 1e-6) {
    L.why = "the bays face opposite ways";
    return L;
  }
  L.out = nav::unit(osum);
  L.centre = csum * (1.0 / static_cast<double>(c.size()));
  L.right = nav::starboardOf(L.out * -1.0);

  std::sort(c.begin(), c.end(), [&L](const BayTrack * a, const BayTrack * z) {
      return nav::dot(a->p - L.centre, L.right) < nav::dot(z->p - L.centre, L.right);
    });
  for (std::size_t i = 0; i < c.size(); ++i) {
    const double setback = std::abs(nav::dot(c[i]->p - L.centre, L.out));
    if (setback > max_setback_m) {
      L.why = "track " + std::to_string(c[i]->id) + " is not in line with the others";
      return L;
    }
    if (i > 0) {
      const double gap = nav::dot(c[i]->p - c[i - 1]->p, L.right);
      if (gap < min_spacing_m) {
        L.why = "tracks " + std::to_string(c[i - 1]->id) + " and " +
          std::to_string(c[i]->id) + " are too close to be two bays";
        return L;
      }
    }
    L.ids.push_back(c[i]->id);
  }
  L.ok = true;
  return L;
}

/// A bay's number (1-based, left to right facing the bays) before layout() knows them all,
/// from the confirmed bays seen LEFT and RIGHT of it and which way the search slides
/// (`slide` < 0 or 0: left, > 0: right). The bays a search has slid past are the ones it
/// has seen. Sliding left - the default, a run started at the dock's RIGHT end - every bay
/// right of ours has been seen and those left of it maybe not: ours is `bays` less those on
/// its right. Sliding right, 1 plus those on its left. Seen either side adding up to all
/// the bays, the left count is exact whichever way. Clamped to 1..bays.
///
/// RoboCommand lights a window in the bay the DockingReport names (rx_reports.proto), so a
/// wrong number means no fire in ours. Counting from the left alone, a run started at the
/// right end reported its first bay, bay 3, as bay 1.
inline int guessBayNumber(int n_left, int n_right, int slide, int bays = 3)
{
  const int n = (n_left + n_right + 1 >= bays || slide > 0) ? 1 + n_left : bays - n_right;
  return std::max(1, std::min(bays, n));
}

// --------------------------------------------------------------- choice

struct VoteParams
{
  int min_votes = 8;            ///< red+green frames before a bay has a verdict
  double min_share = 0.8;       ///< share of the verdict colour, all-time
  double min_ema = 0.5;         ///< and recently: see BayTrack::green_ema
};

enum class Verdict {Unresolved, Red, Green};

/// The call verdict() and verdictOffIsGreen() share: `n` votes, `share` of them for GREEN,
/// `ema` the recent-weighted GREEN share.
inline Verdict voteVerdict(int n, double share, double ema, const VoteParams & v)
{
  if (n < v.min_votes) {return Verdict::Unresolved;}
  if (share >= v.min_share && ema >= v.min_ema) {return Verdict::Green;}
  if (1.0 - share >= v.min_share && ema <= 1.0 - v.min_ema) {return Verdict::Red;}
  return Verdict::Unresolved;
}

inline Verdict verdict(const BayTrack & t, const VoteParams & v)
{
  return voteVerdict(t.votes(), t.greenShare(), t.green_ema, v);
}

/// verdict() with OFF read as GREEN, for a detector that sees the lit GREEN indicator as
/// unlit (the course, 2026-11-12: RED read well, GREEN read OFF). The votes are
/// red + green + off.
inline Verdict verdictOffIsGreen(const BayTrack & t, const VoteParams & v)
{
  const int n = t.red + t.green + t.off;
  return voteVerdict(n, n > 0 ? static_cast<double>(t.green + t.off) / n : 0.0, t.nonred_ema, v);
}

inline char verdictChar(Verdict v)
{
  return v == Verdict::Green ? 'G' : (v == Verdict::Red ? 'R' : '?');
}

/// CommitFirstGreen's colour rule, one number per bay: its GREEN CONFIDENCE is the share of its
/// indicator reads (frames the classifier named a colour at min_ind_conf or more) that said
/// GREEN. GREEN at min_conf or more, once it has min_reads reads; RED when 1 minus it is.
/// off_is_green: OFF reads count as GREEN, for a detector that reads the lit GREEN indicator as
/// unlit (the course, 2026-11-12); without it they count for neither.
struct ColourRule
{
  bool off_is_green = false;
  int min_reads = 3;            ///< reads before the confidence counts: one bad frame is not a bay
  double min_conf = 0.5;        ///< GREEN confidence that commits
};

/// Reads behind greenConfidence: RED + GREEN (+ OFF with off_is_green).
inline int colourReads(const BayTrack & t, bool off_is_green)
{
  return t.red + t.green + (off_is_green ? t.off : 0);
}

/// 0..1, NaN before the first read (see ColourRule).
inline double greenConfidence(const BayTrack & t, bool off_is_green)
{
  const int n = colourReads(t, off_is_green);
  return n > 0 ? static_cast<double>(t.green + (off_is_green ? t.off : 0)) / n :
         std::numeric_limits<double>::quiet_NaN();
}

inline Verdict confidenceVerdict(const BayTrack & t, const ColourRule & r)
{
  if (colourReads(t, r.off_is_green) < std::max(1, r.min_reads)) {return Verdict::Unresolved;}
  const double g = greenConfidence(t, r.off_is_green);
  if (g >= r.min_conf) {return Verdict::Green;}
  return 1.0 - g >= r.min_conf ? Verdict::Red : Verdict::Unresolved;
}

struct Choice
{
  bool ok = false;
  int track_id = -1;
  int bay_number = 0;           ///< 1-based, left to right facing the bays
  std::string why;              ///< the reason when !ok; the evidence when ok
};

/// Exactly one GREEN bay among the numbered ones, or a reason.
///
/// need_others_red: also demand that every other bay has read RED (the course has
/// one safe and two unsafe bays, so GREEN plus two REDs is a consistent picture
/// while GREEN plus two unknowns is one reading of one light). The tree asks
/// strictly first and relaxes only after running out of places to look from.
inline Choice chooseSafeBay(
  const DockBook & b, const DockLayout & L, const VoteParams & v, bool need_others_red)
{
  Choice c;
  if (!L.ok) {
    c.why = "no layout: " + L.why;
    return c;
  }
  std::string tally;
  int greens = 0, reds = 0;
  for (std::size_t i = 0; i < L.ids.size(); ++i) {
    const BayTrack * t = b.find(L.ids[i]);
    if (t == nullptr) {continue;}
    const Verdict vd = verdict(*t, v);
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%s%zu:%c(R%d/G%d)", i ? " " : "", i + 1,
      verdictChar(vd), t->red, t->green);
    tally += buf;
    if (vd == Verdict::Green) {
      ++greens;
      c.track_id = t->id;
      c.bay_number = static_cast<int>(i) + 1;
    } else if (vd == Verdict::Red) {
      ++reds;
    }
  }
  const int others = static_cast<int>(L.ids.size()) - 1;
  if (greens == 0) {
    c.why = "no bay reads GREEN yet [" + tally + "]";
  } else if (greens > 1) {
    c.why = "more than one bay reads GREEN, refusing to guess [" + tally + "]";
  } else if (need_others_red && reds < others) {
    c.why = "one GREEN but not every other bay has read RED [" + tally + "]";
  } else {
    c.ok = true;
    c.why = tally;
    return c;
  }
  c.track_id = -1;
  c.bay_number = 0;
  return c;
}

/// Is the committed bay still GREEN on recent evidence? The berthing legs are
/// guarded by this: the indicator is read best at close range, which is
/// exactly when the boat is committing to the approach.
inline bool stillSafe(const BayTrack & t, double min_ema = 0.3)
{
  return t.green_ema >= min_ema;
}

// --------------------------------------------------------------- where to look

struct Vantage
{
  bool ok = false;
  Vec2 p;            ///< where to look from
  /// Where to drive FIRST: `lead_m` further out along the dock's normal, so the
  /// last leg is driven TOWARD the dock. GuidedSetpoint.yaw is discarded
  /// (position-only type mask), so heading at a vantage is the arrival direction
  /// and the camera faces forward.
  Vec2 lead;
  bool from_bays = false;   ///< placed relative to seen bays (counts as an attempt)
  std::string why;
};

/// Where to look from next.
///
///   bays seen, `attempt` 0     `standoff` in front of the centre of the bays
///   bays seen, `attempt` >= 1  `standoff` in front of the least-read bay,
///                              head on to it, cycling through them
///   nothing seen, blind 0      the approach point, if the goal gave one
///   nothing seen, blind 1..N   a RING SEARCH: `ring_m` out from the approach
///                              point on bearings 360/N apart, so each look
///                              faces a different way (the camera sees ~80 deg)
///   nothing seen, blind > N    REFUSE: the dock is not near the approach point
///
/// `attempt` counts looks taken WITH bays in the book, `blind` those without
/// (Vantage::from_bays says which), so a look spent getting the dock into view
/// does not use up the whole-dock view.
///
/// EVERY VANTAGE IS OUTSIDE THE SLIPS (moving between looks from inside slips
/// crosses a finger): `standoff` must be at least the finger length (2.0 m, build
/// guide) plus half a hull plus margin; the tree uses 5 m, which also keeps all
/// three faces in view and the indicators inside the 6 m the CV reads them at.
/// "In front" is along the bays' outward normal; with nothing seen there is no
/// normal, so the approach point has no lead and its facing is the direction it
/// was driven from.
inline Vantage vantage(
  const DockBook & b, bool have_approach, Vec2 approach, int blind, int attempt,
  double standoff, int min_obs, double lead_m = 3.0, double ring_m = 4.0,
  int ring_looks = 5)
{
  Vantage v;
  std::vector<const BayTrack *> seen;
  for (const auto & t : b.tracks) {if (t.n >= min_obs) {seen.push_back(&t);}}
  if (seen.empty()) {
    if (!have_approach) {
      v.why = "no bay seen and no approach point to look from";
      return v;
    }
    if (blind > ring_looks) {
      v.why = "no bay seen from the approach point or anywhere around it";
      return v;
    }
    v.ok = true;
    v.lead = approach;
    if (blind == 0) {
      v.p = approach;
      v.why = "approach point: no bay seen yet";
    } else {
      // Out from the approach point along a bearing: the boat ARRIVES facing it
      // (GuidedSetpoint.yaw is discarded), the only way to aim its camera.
      const double brg = 360.0 * (blind - 1) / ring_looks;
      v.p = approach + nav::headingVec(brg) * ring_m;
      v.why = "ring search " + std::to_string(blind) + "/" + std::to_string(ring_looks) +
        ": looking " + std::to_string(static_cast<int>(std::lround(brg))) + " deg";
    }
    return v;
  }
  v.from_bays = true;
  Vec2 osum, csum;
  for (const auto * t : seen) {osum = osum + t->outward(); csum = csum + t->p;}
  if (attempt == 0) {
    const Vec2 out = nav::unit(osum);
    const Vec2 centre = csum * (1.0 / static_cast<double>(seen.size()));
    v.ok = true;
    v.p = centre + out * standoff;
    v.lead = centre + out * (standoff + lead_m);
    v.why = "the whole dock, from " + std::to_string(static_cast<int>(std::lround(standoff))) +
      " m";
    return v;
  }
  // Least-read first; ties broken by id so the cycle is deterministic.
  std::sort(seen.begin(), seen.end(), [](const BayTrack * a, const BayTrack * z) {
      return a->votes() != z->votes() ? a->votes() < z->votes() : a->id < z->id;
    });
  const BayTrack * t = seen[static_cast<std::size_t>(attempt - 1) % seen.size()];
  v.ok = true;
  v.p = t->p + t->outward() * standoff;
  v.lead = t->p + t->outward() * (standoff + lead_m);
  v.why = "head on to track " + std::to_string(t->id) + " (" +
    std::to_string(t->votes()) + " votes)";
  return v;
}

// --------------------------------------------------------------- berthing

struct Berth
{
  bool ok = false;
  Vec2 face;         ///< face centre
  Vec2 out;          ///< unit, face -> water
  /// On the centreline, `lead_m` beyond the line-up point. Driven to FIRST, so
  /// the boat reaches the line-up point already heading in instead of turning
  /// 90 degrees inside the run-in.
  Vec2 lead;
  Vec2 predock;      ///< on the centreline, `predock_m` out: line up here
  Vec2 berth;        ///< where the BODY ORIGIN sits when docked
  double berth_m = kNaN;   ///< face -> berth, kept for dockedIn()
  std::string why;
};

/// The lead-in, line-up point and berth for one bay, all on its centreline.
///
/// The bay's own plane normal is preferred over the dock's average (a bay a few
/// degrees off square is entered square to ITS face). `berth_m` is the distance
/// from the face to the body origin when docked: bow offset plus clearance.
inline Berth berthFor(
  const BayTrack & t, const DockLayout & L, double predock_m, double berth_m,
  double lead_m = 4.0)
{
  Berth b;
  b.face = t.p;
  b.out = (t.n_normal > 0) ? t.outward() : (L.ok ? L.out : t.outward());
  if (nav::norm(b.out) < 0.5) {
    b.why = "no idea which way the bay faces";
    return b;
  }
  b.lead = b.face + b.out * (predock_m + lead_m);
  b.predock = b.face + b.out * predock_m;
  b.berth = b.face + b.out * berth_m;
  b.berth_m = berth_m;
  b.ok = true;
  return b;
}

struct DockedCheck
{
  bool docked = false;
  double along = kNaN;          ///< m from the face, along out
  double lateral = kNaN;        ///< m off the centreline, + = right facing the bay
  double heading_err_deg = kNaN;///< bow vs straight into the bay, + = bow right
  std::string why;
};

/// Is the boat in the berth: close enough in, on the centreline, bow in?
///
/// The heading check matters: a boat that arrived sideways is at the right
/// coordinates and not in the bay.
inline DockedCheck dockedIn(
  const Berth & b, Vec2 boat, double heading_deg, double berth_m, double along_tol,
  double lateral_tol, double heading_tol_deg)
{
  DockedCheck d;
  if (!b.ok) {d.why = "no berth"; return d;}
  const Vec2 rel = boat - b.face;
  const Vec2 right = nav::starboardOf(b.out * -1.0);
  d.along = nav::dot(rel, b.out);
  d.lateral = nav::dot(rel, right);
  if (std::isfinite(heading_deg)) {
    const Vec2 bow = nav::headingVec(heading_deg);
    const Vec2 in = b.out * -1.0;
    d.heading_err_deg = std::atan2(nav::cross(bow, in), nav::dot(bow, in)) / nav::kDeg;
  }
  char buf[96];
  std::snprintf(buf, sizeof(buf), "along %.2f m, lateral %+.2f m, heading %+.0f deg",
    d.along, d.lateral, d.heading_err_deg);
  d.why = buf;
  d.docked = std::abs(d.along - berth_m) <= along_tol &&
    std::abs(d.lateral) <= lateral_tol &&
    std::isfinite(d.heading_err_deg) && std::abs(d.heading_err_deg) <= heading_tol_deg;
  return d;
}

/// Centre-to-centre spacing of the numbered bays, as MEASURED (the mean gap
/// between neighbours along the dock); NaN without a layout. The course drawings
/// do not give the slip width, so the boat measures it.
inline double bayPitch(const DockBook & b, const DockLayout & L)
{
  if (!L.ok || L.ids.size() < 2) {return kNaN;}
  double sum = 0.0;
  for (std::size_t i = 1; i < L.ids.size(); ++i) {
    const BayTrack * a = b.find(L.ids[i - 1]);
    const BayTrack * z = b.find(L.ids[i]);
    if (a == nullptr || z == nullptr) {return kNaN;}
    sum += nav::dot(z->p - a->p, L.right);
  }
  return sum / static_cast<double>(L.ids.size() - 1);
}

/// Worst hull corner's distance off the bay's centreline, m: how far the hull
/// really reaches sideways, which a centre-point test does not see (a 1 m hull
/// 15 deg off straight swings its corners 0.13 m further out than its middle).
inline double hullHalfWidthUsed(
  const Berth & b, Vec2 boat, double heading_deg, double hull_length, double hull_beam)
{
  if (!b.ok || !std::isfinite(heading_deg)) {return kNaN;}
  const Vec2 f = nav::headingVec(heading_deg);
  const Vec2 l = nav::portOf(f);
  const Vec2 right = nav::starboardOf(b.out * -1.0);
  double worst = 0.0;
  for (double sf : {-0.5, 0.5}) {
    for (double sl : {-0.5, 0.5}) {
      const Vec2 c = boat + f * (sf * hull_length) + l * (sl * hull_beam);
      worst = std::max(worst, std::abs(nav::dot(c - b.face, right)));
    }
  }
  return worst;
}

/// Already lined up: on the bay's centreline, at least `min_along_m` out from the
/// face, bow pointing in. The lead-in leg is skipped when this holds, so a boat
/// already facing the bay from the survey vantage is not sent back behind itself.
inline bool linedUp(
  const Berth & b, Vec2 boat, double heading_deg, double min_along_m, double lateral_tol,
  double heading_tol_deg)
{
  if (!b.ok || !std::isfinite(heading_deg)) {return false;}
  const DockedCheck d = dockedIn(b, boat, heading_deg, b.berth_m, 0.0, 0.0, 0.0);
  return d.along >= min_along_m && std::abs(d.lateral) <= lateral_tol &&
         std::abs(d.heading_err_deg) <= heading_tol_deg;
}

// --------------------------------------------------------------- the code

/// What the light asked for, in RoboCommand's terms.
struct Request
{
  bool ok = false;
  int resource = kWireUnknown;   ///< RoboCommand Color: which tin
  int delivery = kWireUnknown;   ///< RoboCommand Color: which circle
  std::string why;
};

/// DockObservation (target_pattern, target_colours) -> the request.
///
///   Disruptive  "code"  [c1, c2]: resource c1, delivery c2. The timing layer
///               defines c1 as the colour AFTER the 2 s off, which is the
///               handbook's "1st color"; c1 may equal c2.
///   Advanced    "flash" [c]: any tin, delivered to the c circle.
///
/// Anything else is not a request yet. A "code" with an unlit colour in it is
/// refused, not sent as COLOR_UNKNOWN - the validator would drop the frame.
inline Request requestFrom(const std::string & pattern, const std::vector<Colour> & colours)
{
  Request r;
  if (pattern == "code") {
    if (colours.size() != 2 || !isLit(colours[0]) || !isLit(colours[1])) {
      r.why = "code without two lit colours";
      return r;
    }
    r.resource = wireColour(colours[0]);
    r.delivery = wireColour(colours[1]);
  } else if (pattern == "flash") {
    if (colours.size() != 1 || !isLit(colours[0])) {
      r.why = "flash without one lit colour";
      return r;
    }
    r.resource = kWireAny;
    r.delivery = wireColour(colours[0]);
  } else {
    r.why = "pattern '" + pattern + "' is not a request";
    return r;
  }
  r.ok = true;
  r.why = std::string(wireColourName(r.resource)) + " -> " + wireColourName(r.delivery);
  return r;
}

/// The same request, held steady: the SAME answer for `hold_s` and `min_frames`
/// on top of the timing layer's two full cycles, because the report cannot be
/// taken back. A frame with no request does not contradict; a different request
/// restarts the clock.
struct RequestHold
{
  Request cand;
  double since = kNaN;
  int frames = 0;

  void reset() {*this = RequestHold{};}

  /// Returns true once `cand` has held for long enough.
  bool update(const Request & r, double t, double hold_s, int min_frames)
  {
    if (r.ok) {
      if (!cand.ok || r.resource != cand.resource || r.delivery != cand.delivery) {
        cand = r;
        since = t;
        frames = 0;
      }
      ++frames;
    }
    return cand.ok && std::isfinite(since) && t - since >= hold_s && frames >= min_frames;
  }
};

/// "Did the fire go out?", independent of the timing layer's one-frame "hit"
/// (DockObservation.last_event is set on ONE frame; a dropped message loses it):
/// armed by RED, a hit once the target window has held GREEN `min_green_s`. Either
/// source ends the spraying. need_red = false (a window sprayed although it never read
/// RED): GREEN held min_green_s is the hit unarmed too, and `armed` still means "read RED".
struct HitWatch
{
  bool armed = false;
  bool hit = false;
  double green_since = kNaN;

  void reset() {*this = HitWatch{};}

  void update(Colour target_state, double t, double min_green_s, bool need_red = true)
  {
    if (target_state == Colour::Red) {
      armed = true;
      green_since = kNaN;
    } else if (target_state == Colour::Green && (armed || !need_red)) {
      if (!std::isfinite(green_since)) {green_since = t;}
      if (t - green_since >= min_green_s) {hit = true;}
    } else if (target_state == Colour::Off) {
      green_since = kNaN;
    }
    // Unknown: the colour rule abstained. Keep whatever we had.
  }
};

/// "Is a window of our bay lit RED?", from the windows' own states on the frames that saw
/// our bay - not the timing layer's "steady" verdict, which takes seconds to call and
/// follows whichever bay reads GREEN. Per window: how long it has read RED with no gap
/// over gap_s and no definite other colour (OFF, GREEN, BLUE). UNKNOWN (the colour rule
/// abstained) neither breaks it nor extends it.
struct RedWatch
{
  double gap_s = 0.5;
  struct Run {int index; double since, last;};
  std::vector<Run> runs;

  void reset() {runs.clear();}

  /// One frame's windows of our bay, stamped t (the camera's clock).
  void update(const std::vector<WindowSighting> & ws, double t)
  {
    for (const auto & w : ws) {
      if (w.index < 0 || w.state == Colour::Unknown) {continue;}
      Run * r = nullptr;
      for (auto & x : runs) {if (x.index == w.index) {r = &x;}}
      if (r == nullptr) {runs.push_back({w.index, kNaN, kNaN}); r = &runs.back();}
      if (w.state != Colour::Red) {
        r->since = kNaN;
      } else {
        if (!std::isfinite(r->since) || t - r->last > gap_s) {r->since = t;}
        r->last = t;
      }
    }
  }

  /// The window that has read RED longest, for at least hold_s and still within gap_s of
  /// t; -1 if none.
  int lit(double t, double hold_s) const
  {
    int best = -1;
    double longest = -1.0;
    for (const auto & r : runs) {
      if (!std::isfinite(r.since) || t - r.last > gap_s) {continue;}
      const double held = r.last - r.since;
      if (held >= hold_s && held > longest) {best = r.index; longest = held;}
    }
    return best;
  }

  /// When window `index` last read RED, on the camera's clock (NaN: never).
  double lastRed(int index) const
  {
    for (const auto & r : runs) {
      if (r.index == index) {return r.last;}
    }
    return kNaN;
  }
};

/// The window to spray when none of our bay's has read RED: of the windows with an aim
/// point, the one picked fewest times before (`picked`, every earlier pick), so successive
/// tries alternate between the two; on a tie the one nearest the camera's axis. -1: none.
inline int pickAnyWindow(const std::vector<WindowSighting> & ws, const std::vector<int> & picked)
{
  int best = -1;
  long best_n = 0;
  double best_off = 0.0;
  for (const auto & w : ws) {
    if (w.index < 0 || !w.has_position) {continue;}
    const long n = static_cast<long>(std::count(picked.begin(), picked.end(), w.index));
    const double off = std::fabs(w.y);
    if (best < 0 || n < best_n || (n == best_n && off < best_off)) {
      best = w.index;
      best_n = n;
      best_off = off;
    }
  }
  return best;
}

// --------------------------------------------------------------- reports
//
// The JSON each runner publishes, built HERE so the ROS node and the off-ROS sim
// runner send byte-identical text. Field names are rx_reports.proto's; enums
// travel as protobuf NAMES (the OCS re-serialises with ParseDict, which takes
// names).

/// DockingReport. bay_id is 1-based, left to right facing the bays.
inline std::string dockingReportJson(int bay_id)
{
  return "{\"bay_id\":" + std::to_string(bay_id) + "}";
}

/// FirefightingReport.
inline std::string firefightingReportJson(int window_id)
{
  return "{\"window_id\":" + std::to_string(window_id) + "}";
}

/// ResourceDeliveryRequest, for RoboCommand.
inline std::string resourceRequestJson(const Request & r)
{
  return std::string("{\"task\":\"TASK_COORDINATED_LOGISTICS\",\"resource_color\":\"") +
         wireColourName(r.resource) + "\",\"delivery_circle_color\":\"" +
         wireColourName(r.delivery) + "\"}";
}

/// The water cannon command: fire or not, aimed at a point in camera_link.
inline std::string cannonJson(bool fire, double x, double y, double z)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf),
    "{\"fire\":%s,\"frame_id\":\"camera_link\",\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}",
    fire ? "true" : "false", x, y, z);
  return buf;
}

}  // namespace dock
}  // namespace crusader_bt

#endif  // CRUSADER_BT__DOCK_MATH_HPP_
