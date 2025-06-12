import FreeCAD
import typing

from .Utils.myBoxClass import myBox

class Options:
    splitTolerance = 1.0e-2

class BoxSettings:
    """Parameters used in the solids boundbox generation. Optimized dimensions can reduce
    the translation time.

    Args:
        universe_radius (float, optional): Maximum radius of the CAD universe.
            Solids with coordinates x^2+y^2+z*2 > universe_radius^2 will be cut or not represented.
            Units mm. Defaults to 1.0e6.
        insolid_tolerance (float, optional): Maximum distance from the nearest
            surface of the solid, for which a point outside the solid is assumed
            inside the solid. Used only for boundbox generation. Units mm.
            Defaults to 1.
        box_dimensions (None,tuple,list, optional): dimensions of the universe box in which solids
            will be converted to CAD. Dimensions are (Xmin, Ymin, Zmin, Xmax, Ymax, Zmax) of the box.
            If no box dimensions is provided, the universe dimension is given by the universe_radius parameter.
            Defaul to None.
    """

    def __init__(
        self,
        universe_radius: float = 1.0e6,  # units mm
        insolid_tolerance: float = 1,  # units mm
        box_dimensions: typing.Union[None, list, tuple] = None,
    ):

        self.universe_radius = universe_radius
        self.insolid_tolerance = insolid_tolerance
        self.box_dimensions = box_dimensions
        self.set_universe_box()

    @property
    def universe_radius(self):
        return self._universe_radius

    @universe_radius.setter
    def universe_radius(self, universe_radius: float):
        if not isinstance(universe_radius, (float, int)):
            raise TypeError(f"geoReverse.Settings.universe_radius should be a float, not a {type(universe_radius)}")
        self._universe_radius = universe_radius

    @property
    def insolid_tolerance(self):
        return self._insolid_tolerance

    @insolid_tolerance.setter
    def insolid_tolerance(self, insolid_tolerance: float):
        if not isinstance(insolid_tolerance, (float, int)):
            raise TypeError(f"geoReverse.Settings.insolid_tolerance should be a float, not a {type(insolid_tolerance)}")
        self._insolid_tolerance = insolid_tolerance

    @property
    def box_dimensions(self):
        return self._box_dimensions

    @box_dimensions.setter
    def box_dimensions(self, box_dimensions: typing.Union[None, list, tuple]):
        if box_dimensions is None:
            self._box_dimensions = None
        else:
            if not isinstance(box_dimensions, (list, tuple)):
                raise TypeError(f"geoReverse.Settings.box_dimensions should be a list or tuple, not a {type(box_dimensions)}")
            for x in box_dimensions:
                if not isinstance(x, (float, int)):
                    raise TypeError(f"geoReverse.Settings.box_dimensions elements should be floats, not a {type(x)}")

            for i in range(3):
                vmin, vmax = box_dimensions[i], box_dimensions[i + 3]
                if vmin >= vmax:
                    raise TypeError(
                        f"geoReverse.Settings.box_dimensions bad box limits. Limits should be (Xmin, Ymin, Zmin, Xmax, Ymax, Zmax)."
                    )

            self._box_dimensions = box_dimensions

    @property
    def universe_box(self):
        return self._universe_box

    def set_universe_box(self):
        if self.box_dimensions is None:
            self._universe_box = myBox(
                FreeCAD.BoundBox(
                    -self.universe_radius,
                    -self.universe_radius,
                    -self.universe_radius,
                    self.universe_radius,
                    self.universe_radius,
                    self.universe_radius,
                ),
                "Forward",
            )
        else:
            self._universe_box = myBox(FreeCAD.BoundBox(*self.box_dimensions), "Forward")
            radius = max(map(abs, self.box_dimensions))
            self.universe_radius = radius
