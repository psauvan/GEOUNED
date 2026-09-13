class Options:
    splitTolerance = 1.0e-2


class Tolerances:
    """Tolerances for classifying a raw GQ/SQ quadric's 10 coefficients
    into a specific surface type (cylinder/cone/ellipsoid/hyperboloid/
    paraboloid/...) via eigenvalue decomposition of its quadratic form --
    see `MCNP_parser/MCNPinput.py::getGQAxis` and its own callers
    (`get_cylinder_parameters`/`get_cone_parameters`/
    `get_hyperboloid_parameters`/`get_ellipsoid_parameters`/`gq2params`).

    Every one of these checks MUST be a *relative* tolerance, never an
    absolute one and never exact (`== 0`) equality: the GQ equation is
    invariant under multiplying all 10 coefficients by any nonzero
    scalar, which scales every eigenvalue of the quadratic form's matrix
    -- and the reduced constant `k` computed from it -- by that same
    scalar. A fixed absolute tolerance that classifies one normalization
    of a card correctly will misclassify a differently-normalized card
    representing the exact same surface. Confirmed live against the only
    real GQ fixture in this repo (`tests/csg_files/cylinder_box.mcnp`,
    2026-09 investigation): its eigenvalues are `[-5.55e-17, 1.0, 1.0]`
    for a mathematically clean circular cylinder, but the old `e0 == 0`
    exact-equality test failed on that `-17` order-of-magnitude residual
    and misrouted classification to "hyperboloid" -- it only produced
    the right answer by accident, via the unrelated `cylinder_ratio`
    fallback below."""

    # Relative tolerance (fraction of the eigenvalues' own magnitude,
    # i.e. max(abs(eigenvalues))) below which an eigenvalue -- or the
    # reduced constant `k`, which scales identically under the
    # GQ-coefficient-normalization invariance above -- is treated as
    # exactly zero. Decides a genuinely flat direction (cylinder/cone/
    # paraboloid) vs. a real, if small, curvature there (ellipsoid/
    # hyperboloid). Tight on purpose: a real design's own "is this axis
    # flat" is essentially never ambiguous, so this tolerance only needs
    # to absorb `numpy.linalg.eigh`'s own floating-point residual and a
    # few significant figures of GQ-card rounding, not genuine geometry.
    gq_eigen_zero_rel = 1.0e-6

    # Relative tolerance (fraction of the larger of the two eigenvalues
    # being compared) below which two eigenvalues are treated as equal
    # -- decides a circular cross-section/axis (cylinder, cone, sphere)
    # vs. a genuinely elliptic/hyperbolic one. Looser than
    # `gq_eigen_zero_rel` on purpose: this is the specific check real
    # MCNP-card decimal rounding most often perturbs (a *rotated*
    # circular cylinder written to 6-8 significant figures no longer has
    # two IDENTICAL eigenvalues, just two very close ones -- rounding
    # the 10 coefficients perturbs the whole matrix, not just one
    # entry), so it needs real margin to forgive that. Still far tighter
    # than any genuinely, intentionally elliptic design in practice --
    # nobody designs a part with a fractional-percent elliptic cross-
    # section by accident, so there's a wide, safe gap between "rounding
    # noise" and "real ellipticity" for this value to sit in.
    gq_eigen_equal_rel = 1.0e-5

    # Ratio of the implied major/minor radius above which a hyperboloid
    # or ellipsoid classification is abandoned in favor of a plain
    # cylinder -- a near-zero eigenvalue (however it arose) implies an
    # enormous radius on one axis, which for any real, finite geometry
    # is indistinguishable from a straight cylinder wall. Already
    # existed as a hardcoded `cylTan = 1e3` local in both
    # `get_hyperboloid_parameters` and `get_ellipsoid_parameters`;
    # centralized here, value unchanged.
    cylinder_ratio = 1.0e3

    # Minor radius (mm) below which a hyperboloid is instead treated as
    # practically a cone. Already existed as a hardcoded `coneRad = 0.1`
    # local in `get_hyperboloid_parameters`; centralized here, value
    # unchanged.
    cone_min_radius = 0.1
