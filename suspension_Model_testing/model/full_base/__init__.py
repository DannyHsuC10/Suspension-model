"""
彈簧阻尼系統 KCsys 工具包
"""

from .KC_builder import (
    LinearKC,
    NullKC,
)

from .Geo_builder import (
    create_fast_geometry,
    create_basic_geometry,
)

# ============================================================================
# 公開接口定義
# ============================================================================
__all__ = [

    # 便利函數
    'LinearKC',
    'NullKC',

    # Builder
    "create_fast_geometry",
    "create_basic_geometry",

]