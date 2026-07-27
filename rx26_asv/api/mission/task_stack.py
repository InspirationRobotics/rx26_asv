"""Task stack — the Mission-4 'push, run interrupt, pop and resume' pattern.

Rules (CLAUDE.md, non-negotiable):
  * Only FULL-SUSPEND interrupts (Core-tier assistance requests) push/pop.
  * Keep-out zones and moving objects are NEVER stack entries — they are
    persistent avoidance-layer modifications and must not pause the mission.
  * A suspend captures RESUMABLE state (TaskContext) — resume continues from
    where the task left off, never restarts. Contexts are serializable dicts so
    resume fidelity can be asserted by the evaluator, not just assumed.

No ROS imports; unit-tested.
"""
import json
from dataclasses import dataclass, field


def _json_default(o):
    """Numpy scalars/arrays -> native types at the serialization boundary
    (audit finding #6): tasks compute with geo/np helpers, and a np.float64
    leaking into a context must not detonate json.dumps mid-interrupt."""
    # tolist() first: arrays -> lists, and numpy scalars also expose it
    # (returning a native scalar); item() on a multi-element array raises
    if hasattr(o, "tolist") and callable(o.tolist):
        return o.tolist()
    if hasattr(o, "item") and callable(o.item):
        return o.item()
    raise TypeError(f"TaskContext state contains non-serializable "
                    f"{type(o).__name__}: {o!r}")


@dataclass
class TaskContext:
    """Resumable snapshot of a task. `state` is task-defined but must be
    JSON-serializable — it is logged verbatim for resume-fidelity scoring.

    Producers must honor the TASK CONTRACT (tasks/waypoint_mission.py):
    suspend() that builds one of these is a pure, idempotent snapshot."""
    task_name: str
    state: dict = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps({"task_name": self.task_name, "state": self.state},
                          sort_keys=True, default=_json_default)

    @staticmethod
    def from_json(s: str) -> "TaskContext":
        """Schema-validated: this is the evaluator's resume-fidelity ground
        truth, so a malformed record must say WHICH record and WHY, not raise
        a bare KeyError mid-scoring (audit finding #8)."""
        try:
            d = json.loads(s)
        except json.JSONDecodeError as e:
            raise ValueError(f"malformed TaskContext JSON ({e}): {s[:80]!r}") from e
        if not isinstance(d, dict) or "task_name" not in d or "state" not in d:
            raise ValueError(
                f"TaskContext JSON missing task_name/state keys: {s[:80]!r}")
        if not isinstance(d["task_name"], str) or not isinstance(d["state"], dict):
            raise ValueError(
                f"TaskContext JSON has wrong field types: {s[:80]!r}")
        return TaskContext(d["task_name"], d["state"])


class TaskStack:
    def __init__(self):
        self._stack = []          # [TaskContext]
        self.push_log = []        # [(reason, ctx_json)] — audit trail

    def push(self, ctx: TaskContext, reason: str):
        self._stack.append(ctx)
        self.push_log.append((reason, ctx.to_json()))

    def pop(self) -> TaskContext:
        if not self._stack:
            raise IndexError("task stack empty — pop without matching push "
                             "(resume logic bug, fail loudly)")
        return self._stack.pop()

    @property
    def depth(self) -> int:
        return len(self._stack)
