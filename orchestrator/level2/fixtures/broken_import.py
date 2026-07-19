"""G5 revert-drill fixture: crashes at import time. validate_and_revert must
catch this at the load stage and revert — never activate."""
raise RuntimeError("deliberately broken mechanism (G5 revert drill)")
