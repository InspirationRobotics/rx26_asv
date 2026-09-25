#!/usr/bin/env python3
"""build.py — compile the Task 3 tree OFF-ROS, with nothing but g++ and Python.

    python tools/task3_sim/build.py            # fetch-check, BT.CPP, tests, runner
    python tools/task3_sim/build.py test       # the two stdlib-only math tests
    python tools/task3_sim/build.py runner     # the off-ROS runner the sim drives
    python tools/task3_sim/build.py --clean    # forget every object file

WHY THIS EXISTS. The Task 1 rig is WSL2 -> Docker -> ArduRover SITL + ROS 2,
and a laptop without hardware virtualisation cannot start WSL2 at all. The tree
itself does not need any of that: every leaf reads the Context and calls a
function the runner wired up. So this builds the SAME leaves.cpp, the SAME
task3_leaves.cpp and the SAME XML the boat runs, against BehaviorTree.CPP
compiled from source, and swaps bt_runner_node's ROS plumbing for JSON lines on
stdin/stdout (crusader_bt/offros/offros_runner.cpp).

WHY NOT CMAKE. It is not installed on the dev laptops, and BT.CPP's own
CMakeLists pulls in ZeroMQ and SQLite for Groot and its SQLite logger, neither
of which the runner uses. The source list below is BT_SOURCE from BT.CPP 4.9.0's
CMakeLists.txt minus exactly those two.

BehaviorTree.CPP 4.9.0 is the version crusader_bt was verified against
(context.hpp, 2026-09-07). It is fetched into .deps/, which is gitignored:
nothing third-party is committed and nothing is installed system-wide.

Objects are cached in .build/ and rebuilt only when the source is newer, so the
library costs about two minutes once and nothing after that.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tarfile
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
BT_PKG = os.path.join(REPO, "crusader_bt")
DEPS = os.path.join(HERE, ".deps")
BUILD = os.path.join(HERE, ".build")

BTCPP_VERSION = "4.9.0"
BTCPP_URL = ("https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/"
             f"{BTCPP_VERSION}.tar.gz")
BTCPP_DIR = os.path.join(DEPS, f"BehaviorTree.CPP-{BTCPP_VERSION}")

# BT_SOURCE from BT.CPP 4.9.0's CMakeLists.txt, without groot2_publisher.cpp
# (ZeroMQ) and bt_sqlite_logger.cpp (SQLite). The platform file is added below.
BTCPP_SOURCES = """
src/action_node.cpp src/basic_types.cpp src/behavior_tree.cpp src/blackboard.cpp
src/bt_factory.cpp src/decorator_node.cpp src/condition_node.cpp
src/control_node.cpp src/shared_library.cpp src/tree_node.cpp
src/script_parser.cpp src/script_tokenizer.cpp src/json_export.cpp
src/xml_parsing.cpp
src/actions/test_node.cpp src/actions/sleep_node.cpp src/actions/updated_action.cpp
src/decorators/delay_node.cpp src/decorators/inverter_node.cpp
src/decorators/repeat_node.cpp src/decorators/retry_node.cpp
src/decorators/subtree_node.cpp src/decorators/timeout_node.cpp
src/decorators/updated_decorator.cpp
src/controls/if_then_else_node.cpp src/controls/fallback_node.cpp
src/controls/parallel_node.cpp src/controls/parallel_all_node.cpp
src/controls/reactive_sequence.cpp src/controls/reactive_fallback.cpp
src/controls/sequence_node.cpp src/controls/sequence_with_memory_node.cpp
src/controls/switch_node.cpp src/controls/try_catch_node.cpp
src/controls/while_do_else_node.cpp
src/loggers/bt_cout_logger.cpp src/loggers/bt_file_logger_v2.cpp
src/loggers/bt_minitrace_logger.cpp src/loggers/bt_observer.cpp
3rdparty/tinyxml2/tinyxml2.cpp 3rdparty/minitrace/minitrace.cpp
""".split()

EXE = ".exe" if os.name == "nt" else ""


def say(msg):
    print(msg, flush=True)


def gxx():
    """The compiler, or a message that says what to install."""
    exe = shutil.which("g++")
    if exe is None:
        sys.exit("g++ not found. On Windows install MSYS2's mingw-w64 toolchain "
                 "(C:\\msys64\\mingw64\\bin on PATH); on Linux, build-essential.")
    return exe


def fetch():
    """Download and unpack BT.CPP into .deps/ if it is not already there."""
    if os.path.isdir(os.path.join(BTCPP_DIR, "include")):
        return
    os.makedirs(DEPS, exist_ok=True)
    tgz = os.path.join(DEPS, f"BehaviorTree.CPP-{BTCPP_VERSION}.tar.gz")
    if not os.path.isfile(tgz):
        say(f"fetching {BTCPP_URL}\n      -> {tgz}")
        urllib.request.urlretrieve(BTCPP_URL, tgz)
    say(f"unpacking {os.path.basename(tgz)}")
    with tarfile.open(tgz) as t:
        t.extractall(DEPS)


def newer(src, obj):
    return not os.path.isfile(obj) or os.path.getmtime(src) > os.path.getmtime(obj)


def compile_many(jobs, flags):
    """Compile (src, obj) pairs in parallel. Exits on the first failure."""
    todo = [(s, o) for s, o in jobs if newer(s, o)]
    if not todo:
        return
    cc = gxx()

    def one(pair):
        src, obj = pair
        os.makedirs(os.path.dirname(obj), exist_ok=True)
        cmd = [cc, *flags, "-c", src, "-o", obj]
        r = subprocess.run(cmd, capture_output=True, text=True)
        return src, r

    workers = max(1, (os.cpu_count() or 2))
    failed = None
    with ThreadPoolExecutor(workers) as pool:
        for src, r in pool.map(one, todo):
            rel = os.path.relpath(src, REPO if src.startswith(REPO) else DEPS)
            if r.returncode != 0:
                say(f"  FAILED {rel}\n{r.stderr}")
                failed = failed or rel
            else:
                say(f"  cc {rel}")
                if r.stderr.strip():
                    say(r.stderr.rstrip())     # warnings: shown, not fatal
    if failed:
        sys.exit(f"compile failed: {failed}")


def btcpp():
    """libbtcpp.a, built once."""
    fetch()
    lib = os.path.join(BUILD, "libbtcpp.a")
    plat = "src/shared_library_WIN.cpp" if os.name == "nt" else "src/shared_library_UNIX.cpp"
    srcs = [os.path.join(BTCPP_DIR, s) for s in BTCPP_SOURCES + [plat]]
    objs = [os.path.join(BUILD, "btcpp", os.path.relpath(s, BTCPP_DIR) + ".o") for s in srcs]
    flags = ["-std=c++17", "-O2", "-w",
             f'-DBTCPP_LIBRARY_VERSION="{BTCPP_VERSION}"',
             "-I", os.path.join(BTCPP_DIR, "include"),
             "-I", os.path.join(BTCPP_DIR, "3rdparty"),
             "-I", os.path.join(BTCPP_DIR, "3rdparty", "tinyxml2"),
             "-I", os.path.join(BTCPP_DIR, "3rdparty", "minitrace"),
             "-I", os.path.join(BTCPP_DIR, "3rdparty", "minicoro"),
             "-I", os.path.join(BTCPP_DIR, "3rdparty", "flatbuffers")]
    if os.name == "nt":
        flags += ["-D_CRT_SECURE_NO_WARNINGS", "-DWIN32_LEAN_AND_MEAN"]
    compile_many(list(zip(srcs, objs)), flags)
    if not os.path.isfile(lib) or any(os.path.getmtime(o) > os.path.getmtime(lib) for o in objs):
        say("  ar libbtcpp.a")
        if os.path.isfile(lib):
            os.remove(lib)
        ar = shutil.which("ar") or shutil.which("gcc-ar")
        subprocess.run([ar, "rcs", lib, *objs], check=True)
    return lib


def run_test(name):
    """Build and run one stdlib-only test from crusader_bt/test/."""
    src = os.path.join(BT_PKG, "test", f"{name}.cpp")
    exe = os.path.join(BUILD, name + EXE)
    headers = [os.path.join(BT_PKG, "include", "crusader_bt", h)
               for h in ("nav_math.hpp", "dock_math.hpp")]
    stale = newer(src, exe) or any(os.path.isfile(h) and newer(h, exe) for h in headers)
    if stale:
        os.makedirs(BUILD, exist_ok=True)
        say(f"  cc {name}")
        subprocess.run([gxx(), "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
                        "-I", os.path.join(BT_PKG, "include"), src, "-o", exe],
                       check=True)
    say(f"== {name}")
    r = subprocess.run([exe])
    if r.returncode != 0:
        sys.exit(f"{name} FAILED")


def _move_aside(exe):
    """Windows will not let the linker overwrite an exe that is RUNNING - a
    sim left open is enough, and ld says only "cannot open output file". It
    will let it be RENAMED, so move it out of the way and clear any old ones
    that are no longer running."""
    here = os.path.dirname(exe)
    for f in os.listdir(here):
        if f.startswith("offros_runner.old."):
            try:
                os.remove(os.path.join(here, f))
            except OSError:
                pass                        # still running; next time
    if os.path.isfile(exe):
        try:
            os.replace(exe, "%s.old.%d" % (exe[:-4], int(time.time() * 1000)))
        except OSError as e:
            sys.exit("cannot move %s aside (%s) - close the sim and rebuild" % (exe, e))


def runner():
    """The off-ROS runner: BT.CPP + the real leaves + JSON-lines plumbing."""
    lib = btcpp()
    shim = os.path.join(BT_PKG, "offros", "shim")
    srcs = [os.path.join(BT_PKG, "src", "leaves.cpp"),
            os.path.join(BT_PKG, "src", "task3_leaves.cpp"),
            os.path.join(BT_PKG, "offros", "offros_runner.cpp")]
    objs = [os.path.join(BUILD, "crusader_bt", os.path.basename(s) + ".o") for s in srcs]
    # Every object depends on every crusader_bt header, so touch-rebuild them all
    # whenever a header moves. Cheap: three files.
    hdrs = [os.path.join(dp, f) for dp, _, fs in os.walk(os.path.join(BT_PKG, "include")) for f in fs]
    hdrs += [os.path.join(shim, "rclcpp", "rclcpp.hpp")]
    newest_hdr = max(os.path.getmtime(h) for h in hdrs)
    for o in objs:
        if os.path.isfile(o) and os.path.getmtime(o) < newest_hdr:
            os.remove(o)
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
             "-I", shim,                       # FIRST: the shim is rclcpp here
             "-I", os.path.join(BT_PKG, "include"),
             "-I", os.path.join(BTCPP_DIR, "include")]
    compile_many(list(zip(srcs, objs)), flags)
    exe = os.path.join(BUILD, "offros_runner" + EXE)
    if not os.path.isfile(exe) or any(os.path.getmtime(o) > os.path.getmtime(exe)
                                      for o in objs + [lib]):
        say("  ld offros_runner")
        if os.name == "nt":
            _move_aside(exe)
        link = [gxx(), *objs, lib, "-o", exe]
        if os.name == "nt":
            # Static, so the exe runs from a plain cmd.exe without MSYS2's DLLs
            # on PATH -- which is how the sim launches it.
            link += ["-static", "-static-libgcc", "-static-libstdc++"]
        else:
            link += ["-pthread", "-ldl"]
        subprocess.run(link, check=True)
    say(f"runner: {os.path.relpath(exe, REPO)}")
    return exe


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", nargs="?", default="all",
                    choices=["all", "test", "btcpp", "runner"])
    ap.add_argument("--clean", action="store_true", help="delete .build/ first")
    a = ap.parse_args()
    if a.clean and os.path.isdir(BUILD):
        shutil.rmtree(BUILD)
        say("cleaned .build/")
    if a.what in ("all", "test"):
        run_test("test_nav_math")
        run_test("test_dock_math")
    if a.what == "btcpp":
        btcpp()
    if a.what in ("all", "runner"):
        runner()


if __name__ == "__main__":
    main()
