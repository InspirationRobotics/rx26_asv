// dock_math.hpp — every calculation Task 3 needs, and nothing else.
//
// Same contract as nav_math.hpp, for the same reason: header-only, stdlib only,
// no ROS, no BehaviorTree.CPP. It compiles and runs on a laptop in a second,
//
//     g++ -std=c++17 -O2 -I include -o t test/test_dock_math.cpp && ./t
//
// and it is where every number that can be silently wrong lives. The Task 3
// leaves (src/task3_leaves.cpp) read the Context, call in here, and publish.
//
// WHAT IS IN HERE, in mission order:
//
//   colours    three numbering schemes meet in this task and they DISAGREE.
//              The CV's DockBay/DockWindow put OFF at 1 and RED at 2;
//              RoboCommand's Color and RXL_COLOR put RED at 1. A cast between
//              them sends "GREEN" when the light was RED, so every crossing is
//              a named function with a test.
//   placing    a DockBay sighting (bearing + face plane, camera frame) as a
//              point in the world, so it survives the boat moving.
//   the book   world-anchored bay tracks. The CV's bay_index is "left to right
//              in THIS frame (not a persistent id)" - a boat that sees bays 2
//              and 3 is told 0 and 1. Identity therefore comes from WHERE a bay
//              is, never from its index in a frame.
//   layout     the dock as a line of three: which way it faces, which bay is
//              "1" (left, handbook: "bay numbers referenced left-to-right when
//              facing the bays").
//   choice     indicator votes -> exactly one GREEN bay, or a reason why not.
//   berthing   the line-up point on the bay's centreline, the berth itself,
//              and "am I docked".
//   the code   DockObservation's (pattern, colours) -> RoboCommand's
//              ResourceDeliveryRequest, and a check that it held still.
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

// RoboCommand's Color (rx_common.proto) and the radio's RXL_COLOR share ONE
// numbering, and it is NOT the CV's: there is no OFF, so RED is 1, not 2.
constexpr int kWireUnknown = 0;   // never send - "not yet identified"
constexpr int kWireRed = 1;
constexpr int kWireGreen = 2;
constexpr int kWireBlue = 3;
constexpr int kWireAny = 4;       // Advanced tier: any tin will do

/// Colour -> RoboCommand Color / RXL_COLOR. OFF and UNKNOWN map to 0, which the
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
};

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
/// finite. The ray is horizontal because DockBay carries only a bearing, and a
/// bay face is vertical: the horizontal ray reaches the face at the right
/// horizontal distance whatever height the face centre is at.
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
};

/// One bay face in one frame (DockBay, reduced to what we use).
struct BaySighting
{
  double bearing_deg = kNaN;     ///< face centre, camera frame, + LEFT
  double range_m = kNaN;         ///< plane range, else size range, else NaN
  bool has_normal = false;       ///< plane normal below is valid
  double nx = 0.0, ny = 0.0;     ///< plane normal, camera frame, toward the camera
  bool truncated = false;        ///< cut by the image edge: centre is biased
  bool indicator_present = false;
  Colour indicator = Colour::Unknown;
  double indicator_conf = 0.0;
  std::vector<WindowSighting> windows;
  int lit_window_index = -1;
};

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
  double gate_m = 1.5;
  /// Two TRACKS closer than this are one bay seen twice, and are merged. The
  /// same number as layout()'s min_spacing_m: anything it would refuse as "too
  /// close to be two bays" is folded together here instead of stalling the
  /// survey. A biased early sighting 1.6 m off - outside the gate, inside
  /// this - did exactly that for 70 s in the sim (2026-09-24).
  double merge_m = 1.8;
  int max_tracks = 6;           ///< room for a false positive or two
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
  /// Recent-weighted share of GREEN among red/green readings. Close-range
  /// readings arrive last, so an early far misread is overturned here first.
  double green_ema = 0.5;
  double first_seen = 0.0;
  double last_seen = 0.0;
  std::vector<WindowSighting> windows;   ///< from the latest sighting
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

  /// Fold one frame in. Returns, per sighting, the track id it went to (-1 if
  /// it placed nothing: no range, too far, or a truncated face with no track
  /// to join).
  ///
  /// ASSOCIATION IS GREEDY BY DISTANCE, ONE SIGHTING PER TRACK PER FRAME. Two
  /// bays in one frame are two bays, even if the gate would let both join the
  /// same track - otherwise a range error at the edge of the gate merges them.
  std::vector<int> ingest(const Frame & f, Vec2 boat, double heading_deg, const Mount & m)
  {
    ++frames;
    std::vector<int> out(f.bays.size(), -1);
    if (!std::isfinite(heading_deg)) {return out;}   // cannot place anything

    const Vec2 cam = bodyToWorld(boat, heading_deg, m.x, m.y);
    struct Placed {std::size_t i; Vec2 q; double r;};
    std::vector<Placed> placed;
    for (std::size_t i = 0; i < f.bays.size(); ++i) {
      const auto & s = f.bays[i];
      if (!std::isfinite(s.range_m) || !std::isfinite(s.bearing_deg)) {continue;}
      if (s.range_m <= 0.0 || s.range_m > prm.max_range_m) {continue;}
      const double b = s.bearing_deg * nav::kDeg;
      const Vec2 dir = camDirToWorld(heading_deg, m.yaw_deg, std::cos(b), std::sin(b));
      placed.push_back({i, cam + dir * s.range_m, s.range_m});
    }

    // Every (sighting, track) pair inside the gate, nearest first.
    struct Pair {double d; std::size_t pi; std::size_t ti;};
    std::vector<Pair> pairs;
    for (std::size_t pi = 0; pi < placed.size(); ++pi) {
      for (std::size_t ti = 0; ti < tracks.size(); ++ti) {
        const double d = nav::norm(placed[pi].q - tracks[ti].p);
        if (d <= prm.gate_m) {pairs.push_back({d, pi, ti});}
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {return a.d < b.d;});
    std::vector<bool> pused(placed.size(), false), tused(tracks.size(), false);
    for (const auto & pr : pairs) {
      if (pused[pr.pi] || tused[pr.ti]) {continue;}
      pused[pr.pi] = tused[pr.ti] = true;
      const Placed & pl = placed[pr.pi];
      fold(tracks[pr.ti], f.bays[pl.i], pl.q, pl.r, cam, heading_deg, m, f.t);
      out[pl.i] = tracks[pr.ti].id;
    }
    // Unmatched: a new bay, unless the face is cut off (its centre is biased
    // toward the image edge, and a track born from it starts in the wrong place).
    for (std::size_t pi = 0; pi < placed.size(); ++pi) {
      if (pused[pi]) {continue;}
      const Placed & pl = placed[pi];
      if (f.bays[pl.i].truncated) {continue;}
      if (static_cast<int>(tracks.size()) >= prm.max_tracks) {continue;}
      BayTrack t;
      t.id = next_id++;
      t.first_seen = f.t;
      tracks.push_back(t);
      fold(tracks.back(), f.bays[pl.i], pl.q, pl.r, cam, heading_deg, m, f.t);
      out[pl.i] = tracks.back().id;
    }
    mergeClose();
    return out;
  }

  /// Fold together any two tracks closer than merge_m, the lighter into the
  /// heavier, until none are. Everything is summed; the position is the
  /// weighted mean, so the merged track sits where the better sightings say.
  void mergeClose()
  {
    for (bool again = true; again; ) {
      again = false;
      for (std::size_t i = 0; i < tracks.size() && !again; ++i) {
        for (std::size_t j = i + 1; j < tracks.size() && !again; ++j) {
          if (nav::norm(tracks[i].p - tracks[j].p) >= prm.merge_m) {continue;}
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
    k.n += d.n;
    k.normal_sum = k.normal_sum + d.normal_sum;
    k.n_normal += d.n_normal;
    k.view_sum = k.view_sum + d.view_sum;
    k.red += d.red;
    k.green += d.green;
    k.other += d.other;
    k.first_seen = std::min(k.first_seen, d.first_seen);
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
        const Vec2 nw = camDirToWorld(heading_deg, m.yaw_deg, s.nx, s.ny);
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
/// STARBOARD direction of that look: starboardOf(-out). Get this backwards and
/// every bay number is mirrored - bay 1 reported as bay 3 - while the boat
/// still docks in the right place, which is the kind of error nobody catches
/// until the score comes back.
///
/// Refuses (ok=false, with why) rather than guess when: fewer than `bays`
/// tracks are confirmed; the faces do not sit on one line; or two are closer
/// than `min_spacing_m` (one bay seen twice).
inline DockLayout layout(
  const DockBook & b, int min_obs = 5, int bays = 3, double min_spacing_m = 1.8,
  double max_setback_m = 2.0)
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

// --------------------------------------------------------------- choice

struct VoteParams
{
  int min_votes = 8;            ///< red+green frames before a bay has a verdict
  double min_share = 0.8;       ///< share of the verdict colour, all-time
  double min_ema = 0.5;         ///< and recently: see BayTrack::green_ema
};

enum class Verdict {Unresolved, Red, Green};

inline Verdict verdict(const BayTrack & t, const VoteParams & v)
{
  if (t.votes() < v.min_votes) {return Verdict::Unresolved;}
  const double g = t.greenShare();
  if (g >= v.min_share && t.green_ema >= v.min_ema) {return Verdict::Green;}
  if (1.0 - g >= v.min_share && t.green_ema <= 1.0 - v.min_ema) {return Verdict::Red;}
  return Verdict::Unresolved;
}

inline char verdictChar(Verdict v)
{
  return v == Verdict::Green ? 'G' : (v == Verdict::Red ? 'R' : '?');
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
/// need_others_red: also demand that every other bay has read RED. The
/// handbook's course has one safe bay and two unsafe ones, so a GREEN plus two
/// REDs is a consistent picture; a GREEN plus two unknowns is one reading of
/// one light. The tree asks strictly first and only relaxes after it has run
/// out of places to look from.
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
  /// Where to drive FIRST: `lead_m` further out along the dock's normal, so
  /// the last leg is driven TOWARD the dock and the boat stops facing it.
  ///
  /// This is not a nicety. GuidedSetpoint.yaw is discarded by telemetry_bridge
  /// (position-only type mask), so the heading at a vantage is whatever the
  /// arrival direction was - and the camera faces forward. A vantage reached
  /// sideways is a vantage from which the dock is out of frame.
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
/// (Vantage::from_bays says which this one is), so a look spent getting the
/// dock into view does not use up the whole-dock view.
///
/// EVERY VANTAGE IS OUTSIDE THE SLIPS. There is no "close look" at an
/// indicator from 5 m: with fingers ~6 m long, 5 m from the face is INSIDE a
/// slip, and moving between two such looks crosses a finger. Seen in the sim,
/// 2026-09-24 (two hull contacts). `standoff` must be at least the finger
/// length plus half a hull; the tree uses 10 m. Head-on from there is the
/// closest look there is without entering a bay.
///
/// "In front" is along the bays' outward normal, so every vantage is on the
/// water side of the dock and faces it. With nothing seen there is no normal,
/// so the approach point has no lead: its facing is the direction it was
/// driven from, which is why it belongs on the line from the start to the dock.
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
      // Out from the approach point along a bearing, so the boat ARRIVES
      // facing that bearing: GuidedSetpoint.yaw is discarded, and arrival is
      // the only way this hull chooses where its camera points.
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
  /// the boat reaches the line-up point already heading in: a boat that
  /// arrives there from the side has to turn 90 degrees inside the run-in and
  /// enters the slip crooked.
  Vec2 lead;
  Vec2 predock;      ///< on the centreline, `predock_m` out: line up here
  Vec2 berth;        ///< where the BODY ORIGIN sits when docked
  double berth_m = kNaN;   ///< face -> berth, kept for dockedIn()
  std::string why;
};

/// The lead-in, line-up point and berth for one bay, all on its centreline.
///
/// The bay's own plane normal is preferred over the dock's average: a bay a
/// few degrees off square is still entered square to ITS face. `berth_m` is
/// the distance from the face to the body origin (the pose point) when docked
/// - bow offset plus clearance, not the clearance alone.
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
/// The heading check is not decoration. A boat that arrived at the berth point
/// sideways is at the right coordinates and not in the bay.
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

/// Centre-to-centre spacing of the numbered bays, as MEASURED: the mean gap
/// between neighbours along the dock. NaN without a layout. With thin fingers
/// this is the slip width, which the course drawings have not given us - so
/// the boat measures it rather than trusting a number typed into a tree.
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
/// really reaches sideways, which a centre-point test does not see. A 4.9 m
/// hull 10 deg off straight swings its corners 0.4 m further out than its
/// middle.
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

/// Already lined up: on the bay's centreline, at least `min_along_m` out from
/// the face, bow pointing in. The lead-in leg is skipped when this holds.
///
/// Without it, a boat already facing the bay from the survey vantage - which
/// is exactly where it is when the chosen bay is the middle one - is sent BACK
/// to a lead-in point behind it: it turns round to get there, and turns round
/// again to come in. Seen in the sim, 2026-09-24.
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

/// The same request, held steady.
///
/// The timing layer already wants two full cycles before it says "code". This
/// adds: the SAME answer for `hold_s` and `min_frames` more, because the
/// report cannot be taken back and one more five-second period is cheap out of
/// sixty. A frame with no request (pattern momentarily "pending") does not
/// contradict; a different request restarts the clock.
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

/// "Did the fire go out?", independent of the timing layer's one-frame "hit".
///
/// DockObservation.last_event is set on ONE frame; a dropped message loses it.
/// This watches the target window's own per-frame state: armed by RED, a hit
/// once GREEN has held `min_green_s`. Either source ends the spraying.
struct HitWatch
{
  bool armed = false;
  bool hit = false;
  double green_since = kNaN;

  void reset() {*this = HitWatch{};}

  void update(Colour target_state, double t, double min_green_s)
  {
    if (target_state == Colour::Red) {
      armed = true;
      green_since = kNaN;
    } else if (target_state == Colour::Green && armed) {
      if (!std::isfinite(green_since)) {green_since = t;}
      if (t - green_since >= min_green_s) {hit = true;}
    } else if (target_state == Colour::Off) {
      green_since = kNaN;
    }
    // Unknown: the colour rule abstained. Keep whatever we had.
  }
};

// --------------------------------------------------------------- reports
//
// The JSON each runner publishes, built HERE so the ROS node and the off-ROS
// sim runner send byte-identical text. Field names are rx_reports.proto's, and
// enums travel as their protobuf NAMES, as the Task 1 report's do: the OCS
// re-serialises with ParseDict, which takes names, and a number there is one
// renumbering away from meaning something else.

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

/// The same request for the UAV, as NUMBERS in RXL_COLOR's numbering (which is
/// RoboCommand's), because it is bound for a three-byte radio message rather
/// than for ParseDict. `seq` lets the aircraft tell a repeat from a new ask.
inline std::string uavRequestJson(const Request & r, int seq)
{
  return "{\"seq\":" + std::to_string(seq) + ",\"resource_color\":" +
         std::to_string(r.resource) + ",\"delivery_color\":" + std::to_string(r.delivery) + "}";
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
