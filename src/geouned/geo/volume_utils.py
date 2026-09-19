"""
geo/volume_utils.py

The one form of "these two volumes agree" used everywhere: a relative tolerance measured against a reference volume that is
never smaller than `VOLUME_REF` (see constants.py). Pure Python.
"""

from __future__ import annotations

from .constants import VOLUME_REF


def volume_within(value: float, expected: float, rel_tol: float, reference: float | None = None) -> bool:
    """True if `|value - expected| <= rel_tol * max(|reference|, VOLUME_REF)`.

    `reference` is the volume the change is relative to; it defaults to `expected` (the volume that was there before)."""
    ref = expected if reference is None else reference
    return abs(value - expected) <= rel_tol * max(abs(ref), VOLUME_REF)
