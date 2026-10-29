"""
懸吊幾何工具包
"""

# ============================================================================
# geometry model
# ============================================================================

from .anti_rate import (
    ConstantAnti,
    MultiConstantAnti,
    TableAnti,
)


from .roll_center import (
    ConstantRollCenter,
    MultiConstantRollCenter,
    TableRollCenter,
)

from .camber_gain import (
    TableCamber,
    ConstantCamber,
)

# ============================================================================
# Geometry force model
# ============================================================================

from .Geometry_force import (
    GeometryModel,
)


# ============================================================================
# Builder / Factory
# ============================================================================

from .Geo_builder import (
    create_fast_geometry,
    create_basic_geometry,
    create_table_geometry,
    create_analytic_geometry,
)



# ============================================================================
# Public API
# ============================================================================

__all__ = [

    # Anti
    "ConstantAnti",
    "MultiConstantAnti",
    "TableAnti",


    # Roll center
    "ConstantRollCenter",
    "MultiConstantRollCenter",
    "TableRollCenter",


    # camber
    "TableCamber",
    "ConstantCamber",

    # Main model
    "GeometryModel",


    # Builder
    "create_fast_geometry",
    "create_basic_geometry",
    "create_table_geometry",
    "create_analytic_geometry",

]


__version__ = "6.1.2"
__author__ = "Danny"

__doc__ = """
懸吊幾何工具包

包含:
- Roll center model
- Anti dive / anti squat model
- Suspension geometry force transfer
- Geometry factory

Example
-------
from geometry import (
    GeometryModel,
    ConstantAnti,
    ConstantRollCenter
)
"""