"""Minimal TrackingResult-compatible prediction adapter."""

from __future__ import annotations

from typing import Any, Mapping


def predict(sequence: Any, config: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Return an empty TrackingResult-compatible prediction payload.

    This is intentionally a thin placeholder for future PyTorch trackers. It
    fixes the MATLAB/Python contract without committing to a network design.
    """

    return {
        "metadata": {
            "algorithm": "python-placeholder",
            "task_type": "learning",
            "backend": "python",
        },
        "truth": None,
        "measurements": sequence,
        "estimates": [],
        "metrics": {},
        "config": dict(config or {}),
    }
