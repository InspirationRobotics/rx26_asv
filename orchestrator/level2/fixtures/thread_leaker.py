"""G5 revert-drill fixture: contract-valid mechanism whose startup() spawns a
thread that shutdown() deliberately fails to stop. The thread-audit stage must
catch the orphan and revert. (The thread self-exits after ~0.5 s so the drill
doesn't pollute the rest of the run.)"""
import sys
import threading
import time
from pathlib import Path

_ORCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ORCH))
sys.path.insert(0, str(_ORCH.parent))

from episodes.apf_advisor import ApfAdvisor  # noqa: E402


class ThreadLeaker:
    name = "thread_leaker"

    def __init__(self):
        self._thread = None

    def startup(self):
        # deliberately NOT Event-based, deliberately not joined in shutdown
        self._thread = threading.Thread(target=lambda: time.sleep(0.5),
                                        daemon=True)
        self._thread.start()

    def shutdown(self):
        pass                              # the bug under test: no join, no stop

    def make_advisor(self, scenario, apf_params):
        advisor = ApfAdvisor(scenario, params=apf_params)
        advisor.mechanism_name = self.name
        return advisor


MECHANISM = ThreadLeaker()
