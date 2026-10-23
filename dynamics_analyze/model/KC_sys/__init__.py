"""
彈簧阻尼系統 KCsys 工具包
"""

# ============================================================================
# 導入共享基礎
# ============================================================================
from .MotionRatio import (
    ConstantMR,
    LookupMR,
)

from .damper import (
    NullDamper,
    LinearDamper,
    HLSpeedSwitchDamper,
    HSWeightedDamper,
    VGainDamper,
    LookupDamper,
)

from .spring import (
    NullSpring,
    LinearSpring,
    SeriesSpring,
    LookupSpring,
)

# ============================================================================
# 導入可選公式
# ============================================================================
from .Shock_Absorbers import (
    KCSystem,
)

from .KC_builder import (
    LinearKC,
    SeriesKC,
    Ohlins,
    LookupKC,
    NullKC,

)

# ============================================================================
# 公開接口定義
# ============================================================================



__all__ = [
    # 數據結構
    'ConstantMR',
    'LookupMR',

    'NullDamper',
    'LinearDamper',
    'HLSpeedSwitchDamper',
    'HSWeightedDamper',
    'VGainDamper',
    'LookupDamper',
    
    'NullSpring',
    'LinearSpring',
    'SeriesSpring',
    'LookupSpring',

    'KCSystem',
    # 便利函數
    'LinearKC',
    'SeriesKC',
    'Ohlins',
    'LookupKC',
    'NullKC',


]

__version__ = '6.1.2'
__author__ = 'Danny'
__doc__ = """
彈簧阻尼系統 KCsys 工具包
"""
