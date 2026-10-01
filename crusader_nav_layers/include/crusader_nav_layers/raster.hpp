// raster.hpp - hazards to costmap cells. Pure: stdlib only, no ROS, no Nav2.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_raster.cpp
//
// docs/nav2_avoidance_spec.md section 3.3. A cell (i, j) of a costmap whose
// origin is (ox, oy) covers [ox + i*res, ox + (i+1)*res) x [oy + j*res, ...),
// and its centre is half a cell in. `mark(i, j)` is called once per cell to
// draw; the caller decides what drawing means (the layer writes LETHAL).
//
// CONSERVATIVE ON PURPOSE. A cell is marked when its CENTRE is within
// `r + kHalfDiag * res` of the surface, or when it contains the centre of the
// circle. Every cell that touches the disk has its centre within half a cell
// diagonal of the disk, so no drawn hazard is ever smaller than the real one.
// The cost is up to 0.07 m of extra lethal padding at 0.1 m cells, which the
// spec's "0.80 m, -0.07 / +0.07 m" clearance statement already accounts for.
//
// The clip rectangle [i0, i1) x [j0, j1) is what LayeredCostmap passes to
// updateCosts (exclusive upper bound). It is further clipped to [0, nx) x
// [0, ny), so a hazard far outside the window costs nothing.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace crusader_nav_layers
{

/// Half the diagonal of a cell, in cells. 0.7072 rather than 0.70711 so that
/// floating-point rounding can never turn "touching" into "just outside".
constexpr double kHalfDiag = 0.7072;

struct Pt
{
  double x;
  double y;
};

struct Aabb
{
  double x0, y0, x1, y1;
};

/// One hazard as the layer draws it. `r` is the circle radius INCLUDING keepout.
struct Shape
{
  bool polygon = false;
  double x = 0.0, y = 0.0;      // circle centre
  double r = 0.0;               // circle radius + keepout
  std::vector<Pt> poly;         // convex, any winding
  double keepout = 0.0;         // polygon padding
};

namespace detail
{

inline double clampd(double v, double lo, double hi) {return std::min(std::max(v, lo), hi);}

/// Inclusive cell-index range [lo, hi] covering world interval [a, b], clipped
/// to the window [w0, w1). Empty (lo > hi) when nothing overlaps.
inline void cellRange(
  double a, double b, double o, double res, int w0, int w1, int * lo, int * hi)
{
  const double flo = std::floor((a - o) / res);
  const double fhi = std::floor((b - o) / res);
  // Compare in double BEFORE casting: an infinite or huge bound must not
  // overflow int.
  *lo = static_cast<int>(clampd(flo, static_cast<double>(w0), static_cast<double>(w1)));
  *hi = static_cast<int>(clampd(fhi, static_cast<double>(w0 - 1), static_cast<double>(w1 - 1)));
}

/// The cells a world box [x0, x1] x [y0, y1] can touch, clipped to the pass
/// rectangle [i0, i1) x [j0, j1) AND to the map [0, nx) x [0, ny). Inclusive.
struct Window
{
  int ilo, ihi, jlo, jhi;
};

inline Window window(
  double x0, double x1, double y0, double y1, double res, double ox, double oy, int nx, int ny,
  int i0, int j0, int i1, int j1)
{
  Window w;
  cellRange(x0, x1, ox, res, std::max(i0, 0), std::min(i1, nx), &w.ilo, &w.ihi);
  cellRange(y0, y1, oy, res, std::max(j0, 0), std::min(j1, ny), &w.jlo, &w.jhi);
  return w;
}

inline double segDist(double px, double py, Pt a, Pt b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0.0 ? ((px - a.x) * dx + (py - a.y) * dy) / len2 : 0.0;
  t = clampd(t, 0.0, 1.0);
  return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

}  // namespace detail

/// Signed distance from (px, py) to a convex polygon: negative inside, positive
/// outside, either winding. Needs >= 3 vertices; returns +inf otherwise.
inline double signedDistance(const std::vector<Pt> & poly, double px, double py)
{
  const std::size_t n = poly.size();
  if (n < 3) {return INFINITY;}
  double area2 = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const Pt a = poly[k], b = poly[(k + 1) % n];
    area2 += a.x * b.y - b.x * a.y;
  }
  const double wind = area2 >= 0.0 ? 1.0 : -1.0;     // +1 = counter-clockwise
  double inside_max = -INFINITY;                       // max signed edge distance
  double outside_min = INFINITY;
  bool inside = true;
  for (std::size_t k = 0; k < n; ++k) {
    const Pt a = poly[k], b = poly[(k + 1) % n];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    // Left of a CCW edge is inside, so the outward distance is minus the cross.
    const double d = len > 0.0 ?
      -wind * ((b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x)) / len : 0.0;
    inside_max = std::max(inside_max, d);
    if (d > 0.0) {inside = false;}
    outside_min = std::min(outside_min, detail::segDist(px, py, a, b));
  }
  return inside ? inside_max : outside_min;
}

/// Mark every cell whose centre is within r + kHalfDiag*res of (cx, cy), or that
/// contains (cx, cy). `r` already includes any keepout. Returns cells marked.
template<class Mark>
std::size_t rasterCircle(
  double cx, double cy, double r, double res, double ox, double oy, int nx, int ny,
  int i0, int j0, int i1, int j1, Mark && mark)
{
  if (!(std::isfinite(cx) && std::isfinite(cy) && std::isfinite(r) && r >= 0.0 &&
    res > 0.0))
  {
    return 0;
  }
  const double reach = r + kHalfDiag * res;
  const detail::Window w = detail::window(
    cx - reach, cx + reach, cy - reach, cy + reach, res, ox, oy, nx, ny, i0, j0, i1, j1);
  std::size_t n = 0;
  const double reach2 = reach * reach;
  for (int j = w.jlo; j <= w.jhi; ++j) {
    const double yc = oy + (j + 0.5) * res;
    for (int i = w.ilo; i <= w.ihi; ++i) {
      const double xc = ox + (i + 0.5) * res;
      const double dx = xc - cx, dy = yc - cy;
      const bool contains = std::floor((cx - ox) / res) == i && std::floor((cy - oy) / res) == j;
      if (dx * dx + dy * dy <= reach2 || contains) {
        mark(i, j);
        ++n;
      }
    }
  }
  return n;
}

/// Mark every cell whose centre's signed distance to the convex polygon is
/// <= keepout + kHalfDiag*res. Needs >= 3 finite vertices, else marks nothing.
template<class Mark>
std::size_t rasterConvex(
  const std::vector<Pt> & poly, double keepout, double res, double ox, double oy, int nx, int ny,
  int i0, int j0, int i1, int j1, Mark && mark)
{
  if (poly.size() < 3 || !std::isfinite(keepout) || keepout < 0.0 || !(res > 0.0)) {return 0;}
  double x0 = INFINITY, y0 = INFINITY, x1 = -INFINITY, y1 = -INFINITY;
  for (const Pt & p : poly) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return 0;}
    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
    y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
  }
  const double reach = keepout + kHalfDiag * res;
  const detail::Window w = detail::window(
    x0 - reach, x1 + reach, y0 - reach, y1 + reach, res, ox, oy, nx, ny, i0, j0, i1, j1);
  std::size_t n = 0;
  for (int j = w.jlo; j <= w.jhi; ++j) {
    const double yc = oy + (j + 0.5) * res;
    for (int i = w.ilo; i <= w.ihi; ++i) {
      if (signedDistance(poly, ox + (i + 0.5) * res, yc) <= reach) {
        mark(i, j);
        ++n;
      }
    }
  }
  return n;
}

template<class Mark>
std::size_t rasterShape(
  const Shape & s, double res, double ox, double oy, int nx, int ny,
  int i0, int j0, int i1, int j1, Mark && mark)
{
  return s.polygon ?
         rasterConvex(s.poly, s.keepout, res, ox, oy, nx, ny, i0, j0, i1, j1, mark) :
         rasterCircle(s.x, s.y, s.r, res, ox, oy, nx, ny, i0, j0, i1, j1, mark);
}

/// Cells of padding that make shapeBounds hold every cell the raster can mark:
/// a marked centre is up to kHalfDiag cells from the surface and the cell itself
/// reaches half a cell further along an axis, so 0.7072 + 0.5 = 1.21, rounded up.
constexpr double kBoundsPadCells = 1.5;

/// World-frame box every cell the raster can mark for `s` lies inside. This is
/// what the layer reports to the costmap as dirty, so a box that is too small
/// leaves a ghost lethal cell behind when the hazard goes away.
inline Aabb shapeBounds(const Shape & s, double res)
{
  const double pad = kBoundsPadCells * res;
  if (!s.polygon) {
    return {s.x - s.r - pad, s.y - s.r - pad, s.x + s.r + pad, s.y + s.r + pad};
  }
  Aabb b{INFINITY, INFINITY, -INFINITY, -INFINITY};
  for (const Pt & p : s.poly) {
    b.x0 = std::min(b.x0, p.x); b.x1 = std::max(b.x1, p.x);
    b.y0 = std::min(b.y0, p.y); b.y1 = std::max(b.y1, p.y);
  }
  const double k = s.keepout + pad;
  return {b.x0 - k, b.y0 - k, b.x1 + k, b.y1 + k};
}

/// Hazard message fields -> Shape, or false with `why` filled in. Refuses
/// anything it cannot place exactly: a hazard nobody can place is left out, not
/// drawn somewhere plausible (Hazard.msg). kind 0 = CIRCLE, 1 = POLYGON.
inline bool makeShape(
  int kind, double x, double y, double radius, const std::vector<double> & px,
  const std::vector<double> & py, double keepout, Shape * out, std::string * why)
{
  auto bad = [&](const char * w) {*why = w; return false;};
  if (!std::isfinite(keepout) || keepout < 0.0) {return bad("keepout_m must be finite and >= 0");}
  Shape s;
  s.keepout = keepout;
  if (kind == 0) {
    if (!std::isfinite(x) || !std::isfinite(y)) {return bad("circle centre is not finite");}
    if (!std::isfinite(radius) || radius < 0.0) {return bad("radius_m must be finite and >= 0");}
    s.x = x; s.y = y; s.r = radius + keepout;
  } else if (kind == 1) {
    if (px.size() != py.size()) {return bad("polygon_x and polygon_y differ in length");}
    if (px.size() < 3) {return bad("polygon needs at least 3 vertices");}
    s.polygon = true;
    for (std::size_t k = 0; k < px.size(); ++k) {
      if (!std::isfinite(px[k]) || !std::isfinite(py[k])) {return bad("polygon vertex is not finite");}
      s.poly.push_back({px[k], py[k]});
    }
  } else {
    return bad("unknown kind");
  }
  *out = std::move(s);
  return true;
}

}  // namespace crusader_nav_layers
