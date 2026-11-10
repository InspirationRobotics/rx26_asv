// test_hold_heading — HoldHeading's decision: hold the heading, drift past the radius, return, hysteresis.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_hold_heading.cpp && /tmp/t
#include <cmath>
#include <cstdio>
#include <string>

#include "crusader_bt/hold_heading.hpp"

using namespace crusader_bt;            // NOLINT(build/namespaces) -- a test
using holdheading::Command;
using holdheading::Params;
using nav::Vec2;

static int g_fails = 0;

static void chk(const std::string & name, bool pass)
{
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static bool near(double a, double b, double tol = 1e-6) {return std::fabs(a - b) < tol;}

int main()
{
  Params p;
  p.heading_deg = 90.0;
  const Vec2 P{10.0, 20.0};

  std::printf("1. inside the radius: hold the heading, stopped\n");
  Command c = holdheading::decide(P, P, false, p);
  chk("on P: heading 90, speed 0", near(c.heading_deg, 90.0) && c.speed_mps == 0.0 && !c.returning);
  c = holdheading::decide({11.9, 20.0}, P, false, p);
  chk("1.9 m off with radius 2: still holding", !c.returning && c.speed_mps == 0.0);

  std::printf("\n2. past the radius: head back to P\n");
  c = holdheading::decide({10.0, 17.0}, P, false, p);   // 3 m south of P: P is due north
  chk("bearing to P is north", near(c.heading_deg, 0.0) && c.returning);
  chk("speed capped at return_speed 0.3", near(c.speed_mps, 0.3));
  c = holdheading::decide({7.0, 20.0}, P, false, p);    // 3 m west: P is due east
  chk("west of P -> bearing 090", near(c.heading_deg, 90.0));
  c = holdheading::decide({13.0, 20.0}, P, false, p);   // east of P: P is due west
  chk("east of P -> bearing 270", near(c.heading_deg, 270.0));

  std::printf("\n3. hysteresis\n");
  c = holdheading::decide({10.0, 18.5}, P, true, p);    // 1.5 m: inside radius, outside radius/2
  chk("returning keeps returning at 1.5 m", c.returning && c.speed_mps > 0.0);
  chk("... slowing: gain*dist = 0.45 > 0.3, so still 0.3", near(c.speed_mps, 0.3));
  c = holdheading::decide({10.0, 19.5}, P, true, p);    // 0.5 m: inside radius/2
  chk("returning stops inside radius/2 and holds heading", !c.returning && near(c.heading_deg, 90.0) && c.speed_mps == 0.0);
  c = holdheading::decide({10.0, 18.5}, P, false, p);
  chk("holding does not start returning at 1.5 m", !c.returning);

  std::printf("\n4. speed limits\n");
  p.return_speed_mps = 2.0;
  c = holdheading::decide({10.0, 10.0}, P, false, p);
  chk("return_speed above 0.4 is clamped to 0.4", near(c.speed_mps, 0.4));
  p.return_speed_mps = 0.3;
  p.radius_m = 0.5;
  c = holdheading::decide({10.0, 19.4}, P, false, p);   // 0.6 m: gain*dist = 0.18
  chk("close in, speed tapers with distance", near(c.speed_mps, 0.18));

  std::printf("\n5. heading wrap\n");
  p.heading_deg = 450.0;
  chk("450 -> 90", near(holdheading::decide(P, P, false, p).heading_deg, 90.0));
  p.heading_deg = -90.0;
  chk("-90 -> 270", near(holdheading::decide(P, P, false, p).heading_deg, 270.0));

  std::printf("\n%s\n", g_fails == 0 ? "ALL PASS" : "FAILED");
  return g_fails == 0 ? 0 : 1;
}
