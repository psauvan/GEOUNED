class Options:
    splitTolerance = 1.0e-2

class BoxSettings:
    """Settings for changing the way the CAD to CSG conversion is done

    Args:
        universe_radius (float, optional): Radius of the CAD universe. Used
        to generate solid boundboxes. If this parameter is adjusted to the 
        aproximate size of the full geometry to convert, boundbox solid 
        generation will be faster. Units mm. Defaults to 1.0e8.
        insolid_tolerance (float, optional): Distance from the solid nearest
        surface to a point for which a point outside the solid is assumed 
        inside the solid. Used only for boundbox generation. Units mm. 
        Defaults to 0.1 .
    """

    def __init__(
        self,
        universe_radius: float = 1.0e8,  # units mm
        insolid_tolerance: float = 0.1,  # units mm
    ):

        self.universe_radius = universe_radius
        self.insolid_tolerance = insolid_tolerance

    @property
    def universe_radius(self):
        return self._universe_radius

    @universe_radius.setter
    def universe_radius(self, universe_radius: float):
        if not isinstance(universe_radius, float):
            raise TypeError(f"geoReverse.Settings.universe_radius should be a float, not a {type(universe_radius)}")
        self._universe_radius = universe_radius

    @property
    def insolid_tolerance(self):
        return self._insolid_tolerance

    @insolid_tolerance.setter
    def insolid_tolerance(self, insolid_tolerance: float):
        if not isinstance(insolid_tolerance, float):
            raise TypeError(f"geoReverse.Settings.insolid_tolerance should be a float, not a {type(insolid_tolerance)}")
        self._insolid_tolerance = insolid_tolerance
