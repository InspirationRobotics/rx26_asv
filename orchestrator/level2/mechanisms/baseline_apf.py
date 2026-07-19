"""Known-good baseline mechanism: the stock APF advisor (Gate-G3 validated).

This is the mechanism validate-and-revert falls back to. Do not edit in place —
Level-2 candidates are separate files that get promoted by validate.py.
"""
import sys
from pathlib import Path

_ORCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ORCH))
sys.path.insert(0, str(_ORCH.parent))

from episodes.apf_advisor import ApfAdvisor  # noqa: E402


class BaselineApf:
    name = "baseline_apf"

    def make_advisor(self, scenario, apf_params):
        advisor = ApfAdvisor(scenario, params=apf_params)
        advisor.mechanism_name = self.name
        return advisor


MECHANISM = BaselineApf()
