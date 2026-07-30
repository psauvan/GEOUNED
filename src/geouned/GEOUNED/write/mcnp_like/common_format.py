####################################################################
# Shared writer plumbing for the MCNP-lineage text formats         #
# (MCNP, Serpent, PHITS) -- confirmed duplicated across all three, #
# see CLAUDE.md for the analysis this came from.                   #
####################################################################
import logging

from ..functions import get_cell_surf_summary as _get_cell_surf_summary
from ..functions import simplify_planes as _simplify_planes
from ..functions import sorted_surfaces as _sorted_surfaces

logger = logging.getLogger("general_logger")


class CommonInputWriter:
    """Mixin for the MCNP-lineage text writers. Each subclass sets
    `inline_comment_char`/`line_comment_char` (the format's own comment
    syntax) and, to use `write_surfaces`, `_surface_formatter`/
    `_format_name`.
    """

    inline_comment_char = "$"
    line_comment_char = "C"

    def get_cell_surf_summary(self):
        self.__solidCells__, self.__cells__, self.__materials__ = _get_cell_surf_summary(self.Cells)

    def sorted_surfaces(self, Surfaces):
        return _sorted_surfaces(Surfaces)

    def simplify_planes(self, Surfaces):
        _simplify_planes(Surfaces)

    def get_solid_cell_volume(self):
        solidList = []
        volumeList = []
        for m in self.Cells:
            if m.CellType == "solid" and m.__id__ is not None:
                solidList.append(m.label)
                volumeList.append(m.Volume * 1e-3)
        return solidList, volumeList

    def write_cell_block(self):
        for cell in self.Cells:
            self.write_cells(cell)

    def write_surface_block(self):
        for surf in self.Surfaces:
            self.write_surfaces(surf)

    def write_surfaces(self, surface):
        """Write the surfaces in `self._format_name` format"""

        surf_def = self._surface_formatter(
            surface.bVar.__int__(),
            surface.Type,
            surface.Surf,
            self.options,
            self.tolerances,
            self.numeric_format,
        )
        if surf_def:
            surf_def += "\n"
            self.inpfile.write(surf_def)
        else:
            logger.info(f"Surface {surface.Type} cannot be written in {self._format_name} input")

    def comment_format(self, cComment, mComment=None):

        comment = ""
        char = self.inline_comment_char
        if mComment:
            mComment = mComment.split("\n")
            for c in mComment:
                if c:
                    comment += f"{'':11s}{char}{c}\n"

        if cComment.strip() != "":
            cComment = cComment.strip().split("\n")
            for c in cComment:
                if c:
                    comment += f"{'':11s}{char}{c}\n"
        return comment

    def comment_line(self, lineComment):
        lineComment = lineComment.strip().split("\n")
        char = self.line_comment_char
        comment = ""
        if lineComment:
            comment = f"{char} \n"
            for c in lineComment:
                if c:
                    comment += f"{char} {c}\n"
            comment += f"{char} \n"
        return comment
