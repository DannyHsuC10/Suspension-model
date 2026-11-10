"""The four families and their supported physical topologies."""
from .road_profiles import ISO_ROUGHNESS

FAMILIES = {
    "quarter": "1/4 模型",
    "LR": "1/2 左右模型",
    "FR": "1/2 前後模型",
    "full": "全車模型",
}
MODELS = {
    "quarter": {
        "single": "1 DOF · 等效單彈簧",
        "one_eighth": "1 DOF · 等效彈簧阻尼",
        "double": "2 DOF · 雙彈簧",
        "quarter": "2 DOF · 雙彈簧阻尼",
    },
    "LR": {
        "2mass_2dof": "2 DOF · 雙質點／無輪胎自由度",
        "2mass": "4 DOF · 雙質點＋輪胎",
        "half_2dof": "2 DOF · 車身升沉／側傾",
        "half": "4 DOF · 車身＋輪胎／RC",
        "rollcenter_all": "5 DOF · 車架傳力動態／RC",
    },
    "FR": {
        "half_2dof": "2 DOF · 車身升沉／俯仰",
        "half": "4 DOF · 車身＋輪胎／Anti",
    },
    "full": {
        "rigid": "7 DOF · 剛性全車",
        "flex": "8 DOF · 前後側傾／車架扭轉",
    },
}
DEFAULT_MODELS = {"quarter": "quarter", "LR": "half", "FR": "half", "full": "flex"}
CORNERS = ("FL", "FR", "RL", "RR")
SCENARIOS = {"bump": "平滑凸塊", "step": "路面階躍", "sine": "正弦路面",
             "random": "隨機路面", "cornering": "側向加速度", "accelerate": "縱向加速度",
             "bump_cornering": "凸塊＋側向加速度", "combined": "凸塊＋縱橫向加速度",
             **{key: f"ISO 8608 Class {key[-1].upper()}" for key in ISO_ROUGHNESS}}
TARGETS = {"all": "全部輪位", "left": "左側", "right": "右側", "front": "前軸",
           "rear": "後軸", "FL": "左前 FL", "FR": "右前 FR", "RL": "左後 RL", "RR": "右後 RR"}


def scenarios(family, model):
    result = ["bump", "step", "sine", "random", *ISO_ROUGHNESS]
    if family in ("LR", "full"):
        result.append("cornering")
    if family == "LR":
        result.append("bump_cornering")
    if family in ("FR", "full"):
        result.append("accelerate")
    if family == "full":result.append("combined")
    return result


def targets(family):
    return {"quarter": ["all"], "LR": ["all", "left", "right"],
            "FR": ["all", "front", "rear"], "full": list(TARGETS)}[family]


NOTES = """所有模型以靜態平衡點為零，位移向上為正。升沉為簧上重心位移；側傾正值表示左側上升，俯仰正值表示車頭上升。ax 正值為加速、負值為煞車；ay 正值產生左側上升的慣性側傾。

Motion ratio 採 MR = 彈簧行程 / 輪端行程，輪端 k、c = 元件 k、c × MR²。這與 VehicleLab 的 MR 倒數定義不同；新工具介面均明確標示。所有數值輸入使用欄位標示的 SI 單位，圖表位移則換成 mm，角度換成 deg。

1/4：single / one_eighth 使用懸吊與輪胎串聯 ride rate，質量為 ms+mu；前者沒有阻尼。double / quarter 分別為無阻尼與含阻尼的雙質量系統。

左右：雙質點模型沒有獨立車身慣量，側傾由兩個角落位移推導，側向慣性力矩以左右反向的等效力施加。車身模型使用 z、roll 與 Ix。模態 e_h=dL+dR、e_r=dL−dR，輪端力 = ½ Tᵀ diag(k_h,k_r) T d，加上各角直推元件。所有左右版本使用相同模態倍率。車架傳力版增加一個受 RC 力驅動的虛擬質量，因此實際為 5 DOF。

前後：本族為一側半車，總簧上質量 2ms，前後各一個 mu。前後通道 k_heave 與 k_corner 相加，2 DOF 與 4 DOF 共用彈簧倍率。無輪胎版本直接以路面作基座。有輪胎版本使用前後軸 Anti 比例與縱向加速度，將垂向力施加在車身與輪胎之間。

全車：剛性版 q=[z,roll,pitch,zu_FL,zu_FR,zu_RL,zu_RR]。可扭轉版將 roll 分為前後兩個，並以 body_twist 的 N·m/rad 與 N·m·s/rad 耦合。十個懸吊通道保留原始 heave_f/r、roll_f/r、roll_c、warp 與四角直推；e=T(zs−zu)，輪端力=Tᵀ diag(k·MR²)e。車身力矩以 BᵀF 取得，使用半輪距，確保功與能量一致。前、後輪各有一個每輪簧下質量，同軸左右相同。簧上質量為總質量減去 2(mu_f + mu_r)。

RC 與 Anti 為常數幾何傳力近似，並非即時雙 A 臂運動學。輪荷包含靜態重量及動態輪胎力；負輪荷會明確提示，此線性模型不切換成離地接觸狀態。沒有輪胎自由度的版本不輸出虛構輪荷。

計算使用線性 Mq̈+Cq̇+Kq=f(t)，RK4 依系統特徵值自動細分。所有時間點包含初始與終止點。路面階躍不施加理想速度脈衝；隨機路面使用固定種子的線性插值。動畫只讀計算結果，畫面更新不重新積分。

原始 Model_testing 保留作研究參考。新核心修正了原始前後模型輪胎阻尼方向、heave 多除一次 2、左右右輪引用左路面速度，以及全車側傾力臂、簧上質量與狀態落後一步等問題。因此舊結果匯入後標記為「舊資料」，不視為新核心的重算結果。
"""
