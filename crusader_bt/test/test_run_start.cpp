// test_run_start — RoboCommand's RunStart: what the runner recognises and what WaitForRunStart counts.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_run_start.cpp && /tmp/t
#include <cstdio>
#include <string>

#include "crusader_bt/run_start.hpp"

using namespace crusader_bt::runstart;   // NOLINT(build/namespaces) -- a test

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

/// parse() result as "<hit>:<id>".
static std::string p(const std::string & json)
{
  int id = 99;
  const bool hit = parse(json, &id);
  return std::string(hit ? "hit" : "miss") + ":" + std::to_string(hit ? id : 99);
}

int main()
{
  std::printf("1. what counts as a run start\n");
  chk("run_start with a run_id", p("{\"run_start\": {\"run_id\": 3}}") == "hit:3");
  chk("camelCase keys", p("{\"runStart\":{\"runId\":12}}") == "hit:12");
  chk("no run id is -1, still a hit", p("{\"run_start\": {}}") == "hit:-1");
  chk("run_start as a flag, run_id beside it", p("{\"run_start\": true, \"run_id\": 7}") == "hit:7");
  chk("run_id before the key", p("{\"run_id\": 5, \"run_start\": true}") == "hit:5");
  chk("a quoted number", p("{\"run_start\": {\"run_id\": \"8\"}}") == "hit:8");
  chk("null run_id is -1", p("{\"run_start\": {\"run_id\": null}}") == "hit:-1");
  chk("a negative id is no id", p("{\"run_start\": {\"run_id\": -4}}") == "hit:-1");
  chk("whitespace around the colon", p("{\"run_start\" : { \"run_id\" :  21 }}") == "hit:21");
  chk("run_id with no colon is no id", p("{\"run_start\": {\"run_id\" 3}}") == "hit:-1");

  std::printf("\n2. what does not\n");
  chk("the docking confirmation", p("{\"readiness_confirm\": {}}") == "miss:99");
  chk("a longer key", p("{\"run_start_refused\": {\"run_id\": 3}}") == "miss:99");
  chk("an empty message", p("") == "miss:99");
  chk("not JSON at all", p("run_start") == "miss:99");

  std::printf("\n3. the latch\n");
  {
    Latch l;
    chk("nothing latched counts for nothing", !l.counts(100.0, 900.0));
    l.set(50.0, 3);
    chk("latched before the wait began, inside max_age", l.counts(100.0, 900.0));
    chk("... but not once max_age has passed", !l.counts(100.0, 40.0));
    chk("the edge: received exactly max_age before counts", l.counts(100.0, 50.0));
    chk("received after the wait began", l.counts(10.0, 0.0));
    chk("max_age 0 refuses anything older than the wait", !l.counts(100.0, 0.0));
    l.set(120.0, 4);
    chk("a newer one replaces it, id and time", l.id == 4 && l.t_s == 120.0);
    chk("... and counts for the old wait too", l.counts(100.0, 0.0));
  }

  std::printf("\n4. the OCS clears the latch\n");
  {
    auto dir = [](const std::string & action) {
        return parseDirective("{\"ocs_directive\": {\"action\": \"" + action +
               "\", \"declaration_seq\": 7, \"run_id\": 3}}");
      };
    chk("standby_auto is a new declaration", dir("standby_auto") == Directive::StandbyAuto);
    chk("run_end is the end of the run", dir("run_end") == Directive::RunEnd);
    chk("any other action leaves the latch alone", dir("hold_position") == Directive::None);
    chk("a longer action is not run_end", dir("run_end_soon") == Directive::None);
    chk("whitespace around the colon",
      parseDirective("{\"ocs_directive\":{\"action\" :  \"run_end\"}}") == Directive::RunEnd);
    chk("an action with no directive key is nothing",
      parseDirective("{\"action\": \"run_end\"}") == Directive::None);
    chk("the relayed RunStart is no directive",
      parseDirective("{\"run_start\": {\"run_id\": 3}}") == Directive::None);
    chk("an ocs_run snapshot is no directive",
      parseDirective("{\"ocs_run\": {\"state\": \"RUNNING\", \"run_id\": 3}}") == Directive::None);
    chk("a directive with no action is nothing", parseDirective("{\"ocs_directive\": {}}") == Directive::None);

    Latch l;
    l.set(50.0, 3);
    l.clear();
    chk("clear() forgets the RunStart", !l.counts(100.0, 3600.0) && !l.have && l.id == -1);
    l.set(60.0, 4);
    chk("a RunStart after the clear counts", l.counts(100.0, 3600.0) && l.id == 4);
  }

  std::printf("\n5. run_start_max_age_s as the runner applies it: counts(now, max_age)\n");
  {
    Latch l;
    l.set(1000.0, 5);
    chk("an hour-old RunStart counts at max_age 3600", l.counts(4600.0, 3600.0));
    chk("... one second older does not", !l.counts(4601.0, 3600.0));
    chk("a RunStart long before START counts while inside the age", l.counts(2800.0, 3600.0));
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
