"""G5/test fixture: loads cleanly but the advisor it builds reports a DIFFERENT
mechanism name — the exact 'silently-inactive mechanism' failure mode. The
per-episode active-mechanism assertion must catch it at the smoke stage."""
import sys
from pathlib import Path

_ORCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ORCH))
sys.path.insert(0, str(_ORCH.parent))

from episodes.apf_advisor import ApfAdvisor  # noqa: E402


class NameLiar:
    name = "name_liar"

    def make_advisor(self, scenario, apf_params):
        advisor = ApfAdvisor(scenario, params=apf_params)
        advisor.mechanism_name = "baseline_apf"   # lying about what's running
        return advisor


MECHANISM = NameLiar()
