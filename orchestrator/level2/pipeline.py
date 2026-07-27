"""Level-2 mechanism research pipeline — the 4-round LLM dialogue
(Explore -> Critique -> Specify -> Generate), producing a candidate mechanism
module that then goes through validate_and_revert. The pipeline WRITES
candidates; only validate.py ever ACTIVATES one.

Explore reads the real target sources (apf_core, planner, progress_monitor) and
the Level-1 failure trace, per CLAUDE.md — and may draw on combinatorial
optimization, bandits, DOE, and real-time scheduling literature.
"""
import json
import re
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
SOURCES = [
    _REPO / "rx26_asv" / "api" / "navigation" / "apf_core.py",
    _REPO / "rx26_asv" / "api" / "navigation" / "progress_monitor.py",
    _REPO / "rx26_asv" / "api" / "mission" / "planner.py",
]

SYSTEM = """You are the Level-2 mechanism-research loop of an autoresearch \
harness for Crusader, a RobotX USV. You design NEW Python mechanisms (avoidance \
correction terms, local-minima escape behaviors, arbitration logic) — not \
parameter values. Hard constraints: mechanisms are ADVISORY (never command \
motors); never open a MAVLink/serial connection; any thread you spawn must \
have an Event-based stop invoked from shutdown(); no external dependencies \
beyond the Python stdlib and the provided modules; never weaken arming/safety \
behavior. Objectives, in priority order: 1 minimize collision probability \
(incl. comms-reported virtual obstacles), 2 prevent/escape local minima \
BEFORE full stalls, 3 maximize mission completion."""

CONTRACT_SPEC = """The generated module MUST expose a module-level MECHANISM \
object with: .name (unique str), .make_advisor(scenario, apf_params) returning \
an advisor object with .advise(t, x, y, heading, goal_xy, keepouts, \
moving_objects) -> (goal_x, goal_y, speed_scale) and .mechanism_name set to \
.name; optional .startup()/.shutdown(). Import ApfAdvisor for reuse via:
    import sys; from pathlib import Path
    _ORCH = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_ORCH)); sys.path.insert(0, str(_ORCH.parent))
    from episodes.apf_advisor import ApfAdvisor
Return the final module inside a single ```python fenced block."""


def _read_sources(max_chars=6000):
    out = []
    for p in SOURCES:
        try:
            out.append(f"### {p.name}\n{p.read_text()[:max_chars]}")
        except OSError:
            out.append(f"### {p.name}: unavailable")
    return "\n\n".join(out)


def _failure_trace(history, limit=15):
    rows = [r for r in history if not r.get("kept")][-limit:]
    return json.dumps(rows, default=str, indent=1)[:8000]


def extract_python_block(text: str) -> str:
    m = re.findall(r"```python\n(.*?)```", text, re.DOTALL)
    if not m:
        raise ValueError("Generate round produced no ```python block")
    return m[-1]


def run_research(llm, history, out_dir, mechanism_name: str = None) -> Path:
    """Runs the 4 rounds and writes the candidate module. Returns its path.
    The caller MUST then run validate_and_revert — this function never
    activates anything."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    explore = llm.complete_text(SYSTEM, (
        "ROUND 1 — EXPLORE. Here are the target sources and the Level-1 "
        "failure trace. Identify the structural weakness Level-1 parameter "
        "tuning cannot fix, and propose 2-3 candidate mechanism ideas.\n\n"
        f"{_read_sources()}\n\nFAILURE TRACE (discarded iterations):\n"
        f"{_failure_trace(history)}"))

    critique = llm.complete_text(SYSTEM, (
        "ROUND 2 — CRITIQUE. Attack each idea from round 1: failure modes, "
        "interaction with ArduRover AVOID_*, thread-safety, blended-score "
        "gaming risk. Pick the single most promising idea.\n\nROUND 1:\n"
        + explore))

    specify = llm.complete_text(SYSTEM, (
        "ROUND 3 — SPECIFY. Write a precise implementation spec for the chosen "
        "idea: interface, state, edge cases, teardown. " + CONTRACT_SPEC
        + "\n\nROUND 2:\n" + critique))

    generate = llm.complete_text(SYSTEM, (
        "ROUND 4 — GENERATE. Produce the complete module per the spec. "
        + CONTRACT_SPEC + "\n\nROUND 3:\n" + specify))

    code = extract_python_block(generate)
    name = mechanism_name or f"candidate_{abs(hash(code)) % 10**8}"
    path = out_dir / f"{name}.py"
    path.write_text(code, encoding="utf-8")     # LLM output may carry unicode;
    (out_dir / f"{name}.dialogue.json").write_text(json.dumps({  # never cp1252
        "explore": explore, "critique": critique,
        "specify": specify, "generate": generate}, indent=1), encoding="utf-8")
    return path
