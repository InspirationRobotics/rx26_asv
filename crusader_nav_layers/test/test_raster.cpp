// test_raster - prove the hazard rasteriser with no ROS, no Nav2, no colcon.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_raster.cpp && /tmp/t
//
// Every check is against geometry worked out independently of the code: the
// brute-force "does this cell's square touch the disk" test, and hand-placed
// polygons. The invariant that matters is NEVER UNDERSHOOT: a drawn hazard
// must cover every cell the real object touches, or the planner threads a gap
// that is not there.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "crusader_nav_layers/raster.hpp"

using namespace crusader_nav_layers;   // NOLINT(build/namespaces) - a test
using Cells = std::set<std::pair<int, int>>;

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static void chk_near(const std::string & name, double got, double want, double tol)
{
  const bool pass = std::fabs(got - want) <= tol;
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s", pass ? "ok" : "FAIL", name.c_str());
  if (pass) {
    std::printf("\n");
  } else {
    std::printf("   (got %.6f, want %.6f)\n", got, want);
  }
}

struct Grid
{
  double res, ox, oy;
  int nx, ny;
};

static Cells drawCircle(
  const Grid & g, double cx, double cy, double r, int i0 = 0, int j0 = 0,
  int i1 = 1 << 20, int j1 = 1 << 20)
{
  Cells c;
  rasterCircle(
    cx, cy, r, g.res, g.ox, g.oy, g.nx, g.ny, i0, j0, i1, j1,
    [&](int i, int j) {c.insert({i, j});});
  return c;
}

static Cells drawPoly(const Grid & g, const std::vector<Pt> & p, double keepout)
{
  Cells c;
  rasterConvex(
    p, keepout, g.res, g.ox, g.oy, g.nx, g.ny, 0, 0, g.nx, g.ny,
    [&](int i, int j) {c.insert({i, j});});
  return c;
}

/// Distance from (px, py) to the nearest point of the cell square.
static double distToCell(const Grid & g, int i, int j, double px, double py)
{
  const double x0 = g.ox + i * g.res, y0 = g.oy + j * g.res;
  const double dx = std::max({x0 - px, 0.0, px - (x0 + g.res)});
  const double dy = std::max({y0 - py, 0.0, py - (y0 + g.res)});
  return std::hypot(dx, dy);
}

static bool inRect(const std::pair<int, int> & c, int i0, int j0, int i1, int j1)
{
  return c.first >= i0 && c.first < i1 && c.second >= j0 && c.second < j1;
}

int main()
{
  std::printf("circle: never undershoots, never overshoots by more than half a cell\n");
  {
    // Awkward on purpose: negative, unaligned origin (a rolling window), odd
    // centres, and radii including 0 and one smaller than a cell.
    const Grid g{0.1, -40.03, -39.97, 800, 800};
    int missed = 0, extra = 0;
    const double radii[] = {0.0, 0.03, 0.3, 0.52, 1.1, 3.0};
    for (double r : radii) {
      for (int k = 0; k < 40; ++k) {
        const double cx = -10.0 + 0.4871 * k, cy = 7.0 - 0.3137 * k;
        const Cells got = drawCircle(g, cx, cy, r);
        // Only look near the circle: the far field cannot be marked, and
        // 640k cells x 240 draws is slower than it is worth.
        const int ci = static_cast<int>((cx - g.ox) / g.res), cj = static_cast<int>((cy - g.oy) / g.res);
        const int span = static_cast<int>((r + 1.0) / g.res) + 2;
        for (int j = cj - span; j <= cj + span; ++j) {
          for (int i = ci - span; i <= ci + span; ++i) {
            const bool marked = got.count({i, j}) != 0;
            if (distToCell(g, i, j, cx, cy) <= r && !marked) {++missed;}   // touches the disk
            const double xc = g.ox + (i + 0.5) * g.res, yc = g.oy + (j + 0.5) * g.res;
            if (marked && std::hypot(xc - cx, yc - cy) > r + 0.7072 * g.res + 1e-9) {++extra;}
          }
        }
      }
    }
    chk("every cell touching the disk is marked (" + std::to_string(missed) + " missed)", missed == 0);
    chk("no marked centre beyond r + 0.7072*res (" + std::to_string(extra) + " extra)", extra == 0);
  }

  std::printf("circle: the cell containing the centre is always marked\n");
  {
    const Grid g{0.1, 0.0, 0.0, 100, 100};
    const Cells c = drawCircle(g, 5.0499999, 3.0000001, 0.0);
    chk("r = 0 marks the containing cell", c.count({50, 30}) == 1);
    chk("r = 0 marks only a few cells", c.size() <= 9);
  }

  std::printf("circle: a 0.3 m buoy at 0.1 m cells\n");
  {
    const Grid g{0.1, 0.0, 0.0, 200, 200};
    const Cells c = drawCircle(g, 10.0, 10.0, 0.3);
    chk_near(
      "area is pi r^2 plus about half a cell of margin",
      static_cast<double>(c.size()) * 0.01, M_PI * 0.37 * 0.37, 0.06);
  }

  std::printf("clipping: the pass rectangle is half-open and the map bounds hold\n");
  {
    const Grid g{0.1, 0.0, 0.0, 100, 100};
    const Cells all = drawCircle(g, 5.0, 5.0, 1.0);
    const Cells clip = drawCircle(g, 5.0, 5.0, 1.0, 45, 40, 50, 55);
    bool inside = true, complete = true;
    for (const auto & c : clip) {inside = inside && inRect(c, 45, 40, 50, 55);}
    for (const auto & c : all) {
      if (inRect(c, 45, 40, 50, 55)) {complete = complete && clip.count(c) == 1;}
    }
    chk("nothing marked outside [i0,i1) x [j0,j1)", inside && !clip.empty());
    chk("everything the unclipped draw marks inside the rectangle is marked", complete);
    const Cells edge = drawCircle(g, 0.0, 0.0, 2.0);
    bool in_map = true;
    for (const auto & c : edge) {in_map = in_map && inRect(c, 0, 0, g.nx, g.ny);}
    chk("a circle straddling the map corner stays inside the map", in_map && !edge.empty());
    chk("a circle far outside marks nothing", drawCircle(g, 500.0, 500.0, 1.0).empty());
    chk("an empty pass rectangle marks nothing", drawCircle(g, 5.0, 5.0, 1.0, 50, 50, 50, 60).empty());
    chk("an infinite radius is refused, not looped", drawCircle(g, 5.0, 5.0, INFINITY).empty());
    chk("a NaN centre is refused", drawCircle(g, NAN, 5.0, 1.0).empty());
  }

  std::printf("signedDistance: a 2 x 1 rectangle\n");
  {
    const std::vector<Pt> ccw{{0, 0}, {2, 0}, {2, 1}, {0, 1}};
    const std::vector<Pt> cw{{0, 0}, {0, 1}, {2, 1}, {2, 0}};
    chk_near("centre is 0.5 inside", signedDistance(ccw, 1.0, 0.5), -0.5, 1e-12);
    chk_near("0.2 from an edge, inside", signedDistance(ccw, 0.2, 0.5), -0.2, 1e-12);
    chk_near("0.3 off the long side", signedDistance(ccw, 1.0, 1.3), 0.3, 1e-12);
    chk_near(
      "corner distance is Euclidean, not the max of the axes",
      signedDistance(ccw, 2.3, 1.4), 0.5, 1e-12);
    chk_near("clockwise winding, outside", signedDistance(cw, 2.3, 1.4), 0.5, 1e-12);
    chk_near("clockwise winding, inside", signedDistance(cw, 0.2, 0.5), -0.2, 1e-12);
    chk("fewer than 3 vertices is +inf", std::isinf(signedDistance({{0, 0}, {1, 1}}, 0.0, 0.0)));
  }

  std::printf("polygon: marks the footprint plus keepout, never less\n");
  {
    const Grid g{0.1, -10.0, -10.0, 200, 200};
    const std::vector<Pt> tri{{0.0, 0.0}, {2.0, 0.3}, {0.7, 1.9}};     // convex, CCW
    for (double keepout : {0.0, 0.25}) {
      const Cells c = drawPoly(g, tri, keepout);
      bool under = false, over = false;
      for (int j = 0; j < g.ny; ++j) {
        for (int i = 0; i < g.nx; ++i) {
          const double xc = g.ox + (i + 0.5) * g.res, yc = g.oy + (j + 0.5) * g.res;
          const double sd = signedDistance(tri, xc, yc);
          const bool marked = c.count({i, j}) != 0;
          if (sd <= keepout && !marked) {under = true;}    // centre inside the padded shape
          if (marked && sd > keepout + 0.7072 * g.res + 1e-9) {over = true;}
        }
      }
      const std::string k = "keepout " + std::to_string(keepout) + ": ";
      chk(k + "no centre inside the shape is missed", !under);
      chk(k + "nothing beyond keepout + half a cell", !over);
    }
    chk(
      "a clockwise triangle draws the same cells",
      drawPoly(g, {{0.0, 0.0}, {0.7, 1.9}, {2.0, 0.3}}, 0.25) == drawPoly(g, tri, 0.25));
    chk("two vertices draw nothing", drawPoly(g, {{0, 0}, {1, 1}}, 0.5).empty());
    chk("a NaN vertex draws nothing", drawPoly(g, {{0, 0}, {1, 0}, {NAN, 1}}, 0.5).empty());
    chk("negative keepout draws nothing", drawPoly(g, tri, -0.1).empty());
  }

  std::printf("makeShape: refuses what it cannot place\n");
  {
    Shape s;
    std::string why;
    chk(
      "good circle: r includes keepout",
      makeShape(0, 1.0, 2.0, 0.3, {}, {}, 0.1, &s, &why) && !s.polygon &&
      std::fabs(s.r - 0.4) < 1e-12);
    chk("good polygon", makeShape(1, 0, 0, 0, {0, 1, 0}, {0, 0, 1}, 0.0, &s, &why) && s.polygon);
    chk("NaN centre refused", !makeShape(0, NAN, 0, 0.3, {}, {}, 0.0, &s, &why));
    chk("negative radius refused", !makeShape(0, 0, 0, -0.3, {}, {}, 0.0, &s, &why));
    chk("negative keepout refused", !makeShape(0, 0, 0, 0.3, {}, {}, -1.0, &s, &why));
    chk("ragged polygon refused", !makeShape(1, 0, 0, 0, {0, 1, 0}, {0, 0}, 0.0, &s, &why));
    chk("2-vertex polygon refused", !makeShape(1, 0, 0, 0, {0, 1}, {0, 0}, 0.0, &s, &why));
    chk(
      "infinite vertex refused",
      !makeShape(1, 0, 0, 0, {0, 1, INFINITY}, {0, 0, 1}, 0.0, &s, &why));
    chk("unknown kind refused", !makeShape(7, 0, 0, 0.3, {}, {}, 0.0, &s, &why) && !why.empty());
  }

  std::printf("shapeBounds: contains everything the raster can mark\n");
  {
    const Grid g{0.1, -3.0, -3.0, 60, 60};
    Shape c, p;
    std::string why;
    makeShape(0, 0.37, -0.52, 0.3, {}, {}, 0.2, &c, &why);
    makeShape(1, 0, 0, 0, {-1.0, 1.0, 0.8}, {-0.5, -0.4, 1.1}, 0.15, &p, &why);
    for (const Shape * s : {&c, &p}) {
      const Aabb b = shapeBounds(*s, g.res);
      bool inside = true;
      std::size_t n = rasterShape(
        *s, g.res, g.ox, g.oy, g.nx, g.ny, 0, 0, g.nx, g.ny, [&](int i, int j) {
          const double x0 = g.ox + i * g.res, y0 = g.oy + j * g.res;
          inside = inside && x0 >= b.x0 - 1e-9 && x0 + g.res <= b.x1 + 1e-9 &&
          y0 >= b.y0 - 1e-9 && y0 + g.res <= b.y1 + 1e-9;
        });
      chk(
        std::string(s->polygon ? "polygon" : "circle") + " bounds hold every marked cell",
        inside && n > 0);
    }
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
