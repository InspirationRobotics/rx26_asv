"""mission_spec — the run's behaviour tree, as PLAIN DATA.

Stdlib only: no py_trees, no BehaviorTree.CPP, no rclpy. A deliberate bet.

THE TREE'S SHAPE IS A TEAM DECISION; THE LIBRARY THAT TICKS IT IS AN
IMPLEMENTATION DETAIL. Which missions run, in what order, and what happens when
one fails must be reviewable before anyone installs anything, and switching
libraries later must not mean retyping the tree.

Written as data:

  * `print_tree.py --ascii` and `--xml` render it on a laptop with nothing
    installed;
  * the BehaviorTree.CPP XML is generated, so it cannot drift from what runs;
  * choosing py_trees or BehaviorTree.CPP is ~100 lines of renderer, not a
    rewrite.

THE ONE STRUCTURAL RULE, and it is a scoring rule rather than a style one:
**scoring is per task, so a failed Task 1 must never cost Task 3.** Every mission
sits under a decorator that turns FAILURE into "carry on", and `check()`
enforces that.
"""

# Node kinds: a small set that maps onto BOTH BehaviorTree.CPP and py_trees
# without a per-library special case.
SEQUENCE = "Sequence"          # all children in order; first FAILURE fails
FALLBACK = "Fallback"          # first SUCCESS wins (py_trees calls it Selector)
FORCE_SUCCESS = "ForceSuccess"  # decorator: FAILURE -> SUCCESS, carry on
RETRY = "Retry"                # decorator: re-run a failing child N times
MISSION = "Mission"            # leaf: one ROS action = one mission
CONDITION = "Condition"        # leaf: a guard, no side effects

DECORATORS = (FORCE_SUCCESS, RETRY)
COMPOSITES = (SEQUENCE, FALLBACK)
LEAVES = (MISSION, CONDITION)


class Node:
    """One tree node. `kind` is one of the constants above."""

    __slots__ = ("kind", "name", "children", "attrs")

    def __init__(self, kind, name, children=(), **attrs):
        if kind not in COMPOSITES + DECORATORS + LEAVES:
            raise ValueError("unknown node kind: " + str(kind))
        self.kind = kind
        self.name = name
        self.children = list(children)
        self.attrs = dict(attrs)

    def __repr__(self):
        return "<{} {!r} x{}>".format(self.kind, self.name, len(self.children))


def Sequence(name, *children):
    return Node(SEQUENCE, name, children)


def Fallback(name, *children):
    return Node(FALLBACK, name, children)


def ForceSuccess(child):
    return Node(FORCE_SUCCESS, "carry_on", [child])


def Retry(attempts, child):
    return Node(RETRY, "retry_x{}".format(attempts), [child],
                num_attempts=attempts)


def Mission(name, action, action_type, **attrs):
    """A mission leaf. `action` is the ROS action NAME the server advertises.

    It must match crusader_params.yaml's `action_name` for that server; a typo
    here produces a leaf that waits forever for a server that will never appear,
    which looks exactly like a slow mission. check() catches it off-boat.
    """
    return Node(MISSION, name, (), action=action, action_type=action_type,
                **attrs)


def Condition(name, **attrs):
    return Node(CONDITION, name, (), **attrs)


# --------------------------------------------------------------- THE RUN TREE

def build(missions=None):
    """The competition run.

    missions: subset of mission names to include, for a bench run that should
      not attempt everything. None = all of them.

    Structure and the reasoning behind it:

      Sequence
      |-- is_autonomous            guard; the run does not start in MANUAL
      |-- carry_on( retry_x2( SafePassage ) )
      |-- carry_on( retry_x1( CoordinatedLogistics ) )

    The guard is a CONDITION rather than something each mission checks, because a
    tree that ticks missions while the pilot has the boat fights the pilot. (Each
    mission ALSO checks the mode itself, see safe_passage_server, since the pilot
    can take the boat back mid-mission.)

    Retry counts differ on purpose: Safe Passage is cheap to re-attempt from the
    same approach point, while Coordinated Logistics has consumed a tin by the
    time it fails and a blind retry can make things worse.

    Task 2 is Graey's; the USV's Task 2 role (localize the GREEN indicator for the
    UUV) is deliberately absent rather than an empty leaf that renders as
    capability we do not have.
    """
    catalogue = {
        "SafePassage": (
            Mission("safe_passage", "/crsd/safe_passage",
                    "crusader_msgs/action/SafePassage"), 2),
    }
    wanted = list(catalogue) if missions is None else list(missions)
    unknown = [m for m in wanted if m not in catalogue]
    if unknown:
        raise ValueError("unknown mission(s): {} — known: {}".format(
            sorted(unknown), sorted(catalogue)))

    children = [Condition("is_autonomous",
                          topic="/crsd/fcu_status",
                          note="mode in shared.autonomous_modes")]
    for name in wanted:
        leaf, attempts = catalogue[name]
        children.append(ForceSuccess(Retry(attempts, leaf)))
    return Sequence("run", *children)


# ------------------------------------------------------------------- CHECKING

def walk(node, depth=0, parent=None):
    yield node, depth, parent
    for c in node.children:
        for item in walk(c, depth + 1, node):
            yield item


def check(root):
    """Structural problems, as a list of strings. Empty list = the tree is sane.

    Runs with nothing installed and no boat: each defect would otherwise show up
    as a mission that silently never runs, on the water.
    """
    problems = []
    seen_actions = {}
    mission_count = 0

    for node, _depth, _parent in walk(root):
        if node.kind in COMPOSITES and not node.children:
            problems.append("{} {!r} has no children — it will return SUCCESS "
                            "immediately and look like it worked"
                            .format(node.kind, node.name))
        if node.kind in DECORATORS and len(node.children) != 1:
            problems.append("decorator {!r} has {} children; decorators take "
                            "exactly one".format(node.name,
                                                 len(node.children)))
        if node.kind in LEAVES and node.children:
            problems.append("leaf {!r} has children".format(node.name))

        if node.kind == MISSION:
            mission_count += 1
            action = node.attrs.get("action")
            if not action:
                problems.append("mission {!r} names no action".format(node.name))
            elif action in seen_actions:
                problems.append(
                    "action {} is used by both {!r} and {!r} — two leaves "
                    "driving one server means the second goal is REJECTED and "
                    "that branch fails for a reason nobody will guess"
                    .format(action, seen_actions[action], node.name))
            else:
                seen_actions[action] = node.name
            if not node.attrs.get("action_type"):
                problems.append("mission {!r} names no action TYPE"
                                .format(node.name))

    if mission_count == 0:
        problems.append("the tree contains no missions at all")

    # THE scoring rule: a mission whose failure can end the run.
    for name in _unprotected_missions(root):
        problems.append(
            "mission {!r} is not under a {} — scoring is PER TASK, so its "
            "failure would end the run and cost every task after it"
            .format(name, FORCE_SUCCESS))
    return problems


def _unprotected_missions(root):
    """Missions with no ForceSuccess between them and a Sequence ancestor.

    A Fallback ancestor also absorbs a failure (its next child runs), so it
    counts as protection too — this is about whether ONE mission's failure can
    terminate the run, not about the decorator specifically.
    """
    bad = []

    def visit(node, protected):
        if node.kind == MISSION:
            if not protected:
                bad.append(node.name)
            return
        for c in node.children:
            visit(c, protected or node.kind in (FORCE_SUCCESS, FALLBACK))

    visit(root, False)
    return bad


# ------------------------------------------------------------------ SIMULATION

SUCCESS, FAILURE = "SUCCESS", "FAILURE"


def simulate(root, leaf_results, trace=None):
    """Tick the tree once with each leaf's result decided in advance.

    leaf_results: {leaf_name: SUCCESS|FAILURE}. A leaf not named defaults to
      SUCCESS. A FAILURE under a Retry is retried and then fails again — the
      point is to see what the STRUCTURE does with a mission that cannot be
      made to work, not to model a flaky one.
    trace: optional list; each entry appended as (depth, name, result).

    Returns the root's result.

    THIS IS NOT A BEHAVIOUR TREE ENGINE, and must not grow into one. It answers
    one question worth answering before a run:

        if SafePassage fails, does the rest of the run still happen?

    Scoring is per task, so the answer must be yes. py_trees and BehaviorTree.CPP
    both implement these semantics; this reproduces them for the four node kinds
    the tree uses, so the question can be answered before the library choice is
    settled.
    """
    def tick(node, depth):
        if node.kind in LEAVES:
            r = leaf_results.get(node.name, SUCCESS)
        elif node.kind == SEQUENCE:
            r = SUCCESS
            for c in node.children:
                if tick(c, depth + 1) == FAILURE:
                    r = FAILURE
                    break                      # memory sequence: stop at the first failure
        elif node.kind == FALLBACK:
            r = FAILURE
            for c in node.children:
                if tick(c, depth + 1) == SUCCESS:
                    r = SUCCESS
                    break
        elif node.kind == FORCE_SUCCESS:
            tick(node.children[0], depth + 1)
            r = SUCCESS
        elif node.kind == RETRY:
            attempts = int(node.attrs.get("num_attempts", 1))
            r = FAILURE
            for _ in range(max(1, attempts)):
                r = tick(node.children[0], depth + 1)
                if r == SUCCESS:
                    break
        else:                                   # unreachable: Node() validates kind
            raise AssertionError("unhandled kind " + node.kind)
        if trace is not None:
            trace.append((depth, node.name, r))
        return r

    return tick(root, 0)
