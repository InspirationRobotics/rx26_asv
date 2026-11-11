// Off-ROS tests for task1_memory.hpp (the Core tier's ENTRY/EXIT between runs) and the
// passage planner's anchor snap that the remembered and MANUAL points rely on.
//   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_task1_memory.cpp
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/global_passage.hpp"
#include "crusader_bt/task1_memory.hpp"

using namespace crusader_bt;
using nav::Beacon;
using nav::Buoy;
using nav::Vec2;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { \
    if (cond) {++g_pass;} else { \
      ++g_fail; std::printf("FAIL %s:%d: %s  ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__); std::printf("\n"); \
    } \
} while (0)

static const nav::LatLon kOrigin{1.28050, 103.85570};

static Buoy buoy(int id, double x, double y, Beacon s, int hits = 50)
{
  Buoy b;
  b.id = id;
  b.p = {x, y};
  b.state = s;
  b.hits = hits;
  return b;
}

static void testRules()
{
  CHECK(!t1mem::validLatLon(0.0, 0.0), "a cleared plan's (0, 0) is no point");
  CHECK(!t1mem::validLatLon(NAN, 103.0), "NaN");
  CHECK(!t1mem::validLatLon(91.0, 103.0), "out of range");
  CHECK(t1mem::validLatLon(1.2805, 103.8557), "a real point");

  std::vector<nav::PlanBuoy> blues{{1, {0, 0}, Beacon::FlashingBlue}, {2, {0, 12}, Beacon::SteadyBlue}};
  CHECK(!t1mem::hasSides(blues), "ENTRY/EXIT only: Core");
  blues.push_back({3, {2, 5}, Beacon::FlashingRed});
  CHECK(t1mem::hasSides(blues), "a red: the plan's own field");

  const Buoy seen = buoy(7, 3.0, 4.0, Beacon::FlashingBlue);
  t1mem::Remembered mem{true, nav::toLatLon({10.0, 10.0}, kOrigin), 40, "t"};
  t1mem::Anchor a = t1mem::pick(true, {1.0, 2.0}, &seen, mem, kOrigin);
  CHECK(a.have && a.src == t1mem::Source::Plan && a.p.x == 1.0, "the plan's point wins");
  a = t1mem::pick(false, {}, &seen, mem, kOrigin);
  CHECK(a.have && a.src == t1mem::Source::Seen && a.p.x == 3.0, "then the one seen now");
  a = t1mem::pick(false, {}, nullptr, mem, kOrigin);
  CHECK(a.have && a.src == t1mem::Source::Remembered && std::fabs(a.p.x - 10.0) < 1e-6 &&
    std::fabs(a.p.y - 10.0) < 1e-6, "then the remembered one (%.3f, %.3f)", a.p.x, a.p.y);
  a = t1mem::pick(false, {}, nullptr, t1mem::Remembered{}, kOrigin);
  CHECK(!a.have && a.src == t1mem::Source::None, "else none");

  const Buoy few = buoy(8, 3.0, 4.0, Beacon::FlashingBlue, 29);
  CHECK(!t1mem::shouldRemember(&few, {}, kOrigin, 30, 0.5), "29 hits is not enough");
  CHECK(t1mem::shouldRemember(&seen, {}, kOrigin, 30, 0.5), "nothing remembered: remember");
  t1mem::Remembered here{true, nav::toLatLon({3.3, 4.0}, kOrigin), 50, "t"};
  CHECK(!t1mem::shouldRemember(&seen, here, kOrigin, 30, 0.5), "0.3 m off: keep the file");
  t1mem::Remembered far{true, nav::toLatLon({4.0, 4.0}, kOrigin), 50, "t"};
  CHECK(t1mem::shouldRemember(&seen, far, kOrigin, 30, 0.5), "1 m off: refine");
  CHECK(!t1mem::shouldRemember(nullptr, {}, kOrigin, 30, 0.5), "nothing seen");
}

static void testFile()
{
  t1mem::Remembered e{true, {1.28053593, 103.85565058}, 57, "2026-11-11T09:30:00"};
  t1mem::Remembered x{true, {1.28069822, 103.85565281}, 41, ""};
  const std::string text = t1mem::serialize(e, x);
  t1mem::Remembered e2, x2;
  t1mem::parse(text, e2, x2);
  CHECK(e2.have && std::fabs(e2.ll.lat - e.ll.lat) < 1e-9 && e2.hits == 57 &&
    e2.saved == e.saved, "entry round trip");
  CHECK(x2.have && std::fabs(x2.ll.lon - x.ll.lon) < 1e-9 && x2.saved.empty(), "exit round trip");

  t1mem::parse("# junk\nentry 0 0 5 t\nexit nan 103 4 t\nentry 1.2805 103.8557\n", e2, x2);
  CHECK(!e2.have && !x2.have, "(0, 0), NaN and a short line are skipped, never half-read");
  t1mem::parse(t1mem::serialize(e, {}), e2, x2);
  CHECK(e2.have && !x2.have, "only an ENTRY remembered");

  CHECK(t1mem::json({}) == "null", "nothing remembered is null");
  CHECK(t1mem::json(e).find("\"hits\":57") != std::string::npos, "%s", t1mem::json(e).c_str());
}

static void testAnchorSnap()
{
  const Vec2 point{0.0, 0.0};
  // after the mission-start colour reset the ENTRY buoy is Unknown until it is seen again
  std::vector<Buoy> bs{buoy(1, 0.9, 0.0, Beacon::Unknown), buoy(2, -0.5, 0.0, Beacon::FlashingRed)};
  auto a = gp::detail::anchorOf(bs, point, Beacon::FlashingBlue, 1.5);
  CHECK(a.second == 1 && a.first.x == 0.9, "an uncoloured buoy anchors (id %d)", a.second);
  bs.push_back(buoy(3, 1.4, 0.0, Beacon::FlashingBlue));
  a = gp::detail::anchorOf(bs, point, Beacon::FlashingBlue, 1.5);
  CHECK(a.second == 3, "a blue beats a nearer uncoloured buoy (id %d)", a.second);
  std::vector<Buoy> reds{buoy(4, 0.3, 0.0, Beacon::FlashingRed), buoy(5, 0.4, 0.0, Beacon::Off),
    buoy(6, 0.5, 0.0, Beacon::SteadyBlue)};
  a = gp::detail::anchorOf(reds, point, Beacon::FlashingBlue, 1.5);
  CHECK(a.second == -1 && a.first.x == 0.0, "never a red, a black or the other blue");
  std::vector<Buoy> far{buoy(7, 1.6, 0.0, Beacon::Unknown)};
  a = gp::detail::anchorOf(far, point, Beacon::FlashingBlue, 1.5);
  CHECK(a.second == -1, "beyond the snap: the point itself");
}

int main()
{
  testRules();
  testFile();
  testAnchorSnap();
  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
