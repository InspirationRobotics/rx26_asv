"""Level-2 mechanism contract + loader.

A mechanism is a Python module exposing a module-level `MECHANISM` object with:
    name: str                                   — unique mechanism id
    make_advisor(scenario, apf_params) -> obj   — advisor with .advise(...) AND
                                                  .mechanism_name == name
    startup() / shutdown()                      — optional; any threads spawned
                                                  MUST be stopped by shutdown()
                                                  (Event-based, deterministic)

The active mechanism is recorded in mechanisms/active.json; validate-and-revert
(validate.py) is the ONLY writer of that file. load_mechanism() fails loudly on
any contract violation — a mechanism that loads-but-lies is caught later by the
per-episode active-mechanism assertion in level1/suite.py.
"""
import importlib.util
import json
import sys
from pathlib import Path

MECHANISMS_DIR = Path(__file__).resolve().parent / "mechanisms"
ACTIVE_FILE = MECHANISMS_DIR / "active.json"


class ContractError(RuntimeError):
    pass


def load_mechanism(path):
    """Import a mechanism module (unique module name per load) and validate the
    contract. Raises ContractError / ImportError loudly — never returns a
    half-valid mechanism."""
    path = Path(path)
    mod_name = f"rx26_mechanism_{path.stem}_{abs(hash(str(path))) % 10**8}"
    spec = importlib.util.spec_from_file_location(mod_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(mod_name, None)
        raise
    mech = getattr(module, "MECHANISM", None)
    if mech is None:
        raise ContractError(f"{path.name}: no module-level MECHANISM object")
    name = getattr(mech, "name", None)
    if not isinstance(name, str) or not name:
        raise ContractError(f"{path.name}: MECHANISM.name missing/empty")
    if not callable(getattr(mech, "make_advisor", None)):
        raise ContractError(f"{path.name}: MECHANISM.make_advisor not callable")
    return mech


def active_mechanism_path() -> Path:
    d = json.loads(ACTIVE_FILE.read_text())
    return MECHANISMS_DIR / d["module"]


def load_active():
    return load_mechanism(active_mechanism_path())


def set_active(module_filename: str):
    """validate.py promotion path only — never call from anywhere else."""
    if not (MECHANISMS_DIR / module_filename).exists():
        raise FileNotFoundError(module_filename)
    ACTIVE_FILE.write_text(json.dumps({"module": module_filename}, indent=2) + "\n")
