from . import LinearKC, LookupKC, NullKC, Ohlins, SeriesKC

"""
避震器樣式與設定範例
"""

# 測試車輛設定
heave_f = LinearKC(k=30000,c=1500,name="heave_f")
heave_r = LinearKC(k=28000,c=1200,name="heave_r")
roll_f = LinearKC(k=25000,c=1200,name="roll_f")
roll_r = LinearKC(k=22000,c=1200,name="roll_r")


roll_c = NullKC()
warp = LinearKC(k=30000,c=1500,name="warp")

corner_fl = NullKC()
corner_fr = NullKC()
corner_rl = NullKC()
corner_rr = NullKC()

tire_fl = LinearKC(k=80000,c=50,name="tire_fl")
tire_fr = LinearKC(k=80000,c=50,name="tire_fr")
tire_rl = LinearKC(k=80000,c=50,name="tire_rl")
tire_rr = LinearKC(k=80000,c=50,name="tire_rr")


# Öhlins Style
# 線性彈簧 + 高低速阻尼
ohlins = Ohlins(

    k=30000,

    low_comp=1800,
    high_comp=700,

    low_reb=2500,
    high_reb=1000,

    transition=0.05,

    mr_value=1.00,

    name="Ohlins"
)


# LP03 Template
# Tender + Main Spring
LP03_template = SeriesKC(

    main=50000,
    low=15000,

    travel=0.03,

    low_comp=1800,
    high_comp=700,

    low_reb=2500,
    high_reb=1000,

    transition=0.05,

    mr_value=1.00,

    travel_lim_compress=0.08,
    travel_lim_stretch=0.08,

    name="LP03 Template"
)

# Full Lookup
# CAD + Dyno 驗證模型
lookup = LookupKC(

    # Spring
    spring_displacement=[
        -0.08,
        -0.05,
        -0.02,
        0.00,
        0.02,
        0.05,
        0.08,
    ],

    spring_force=[
        6000,
        3000,
        1000,
            0,
        -1000,
        -3500,
        -7000,
    ],

    # Damper
    damper_velocity=[
        -1.0,
        -0.5,
        -0.1,
        0.0,
        0.1,
        0.5,
        1.0,
    ],

    damper_force=[
        3500,
        2600,
        800,
            0,
        -600,
        -2200,
        -3500,
    ],

    # Motion Ratio
    mr_displacement=[
        -0.08,
        -0.04,
        0.00,
        0.04,
        0.08,
    ],

    mr_ratio=[
        0.88,
        0.91,
        0.95,
        0.98,
        1.02,
    ],

    travel_lim_compress=0.08,
    travel_lim_stretch=0.08,

    name="Lookup"
)

