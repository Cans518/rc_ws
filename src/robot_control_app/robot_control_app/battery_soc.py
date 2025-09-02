import numpy as np

# === 自定义3S锂电池电压-SOC经验对照表（静置开路电压）===
# 更符合实际锂电池的非线性特性：高/低段变化快，中间平缓
VOLTAGE_3S = np.array([
    12.6, 12.5, 12.4, 12.3, 12.2, 12.1, 12.0, 11.9, 11.8,
    11.7, 11.6, 11.5, 11.4, 11.3, 11.2, 11.1, 11.0, 10.5, 10.0, 9.0
])

SOC_TABLE = np.array([
    100, 98,  95,  90,  85,  80,  75,  70,  65,
     60,  55,  50,  45,  40,  35,  30,  25,  15,  5,  0
])

# 构建插值函数（电压 -> SOC）
def create_soc_interpolator():
    # 由于电压是降序，SOC是升序，需翻转数组以满足 interp 要求
    v_flip = np.flip(VOLTAGE_3S)
    soc_flip = np.flip(SOC_TABLE)
    # 使用线性插值
    return lambda voltage: np.interp(voltage, v_flip, soc_flip)

# 初始化插值函数
_voltage_to_soc = create_soc_interpolator()

def get_battery_status(voltage):
    """
    输入3S电池电压，返回SOC、报警状态、是否应停止放电
    
    参数:
        voltage (float): 当前3S电池总电压（建议静置测量）
    
    返回:
        dict: 包含 SOC、报警标志、关机标志、状态描述
    """
    # 限制电压范围，防止越界
    voltage_clamped = np.clip(voltage, VOLTAGE_3S[-1], VOLTAGE_3S[0])
    
    # 查表+插值得到SOC
    soc = _voltage_to_soc(voltage_clamped)
    
    # 根据SOC判断状态
    if soc < 10.0:
        should_shutdown = True
        alarm = True
        status = "CRITICAL: Shutting down (SOC < 10%)"
    elif soc < 20.0:
        should_shutdown = False
        alarm = True
        status = "WARNING: Low battery (SOC < 20%)"
    else:
        should_shutdown = False
        alarm = False
        status = "NORMAL"
    
    return {
        'voltage': round(voltage, 2),
        'soc': round(soc, 1),
        'alarm': alarm,
        'should_shutdown': should_shutdown,
        'status': status
    }

def get_battery_status_mv(mv):
    """
    根据3S电池电压（单位：毫伏）返回电量状态
    
    参数:
        mv (int or float): 电池总电压，单位毫伏（mV），例如 11200 表示 11.2V
    
    返回:
        dict: 包含电压(V)、SOC、报警、停机标志、状态描述
    """
    # 毫伏 → 伏特
    voltage_v = mv / 1000.0
    
    # 限制电压范围，防止查表越界
    voltage_clamped = np.clip(voltage_v, VOLTAGE_3S[-1], VOLTAGE_3S[0])
    
    # 插值得到 SOC
    soc = _voltage_to_soc(voltage_clamped)
    
    # 判断状态
    if soc < 10.0:
        should_shutdown = True
        alarm = True
        status = "CRITICAL: Shutting down (SOC < 10%)"
    elif soc < 20.0:
        should_shutdown = False
        alarm = True
        status = "WARNING: Low battery (SOC < 20%)"
    else:
        should_shutdown = False
        alarm = False
        status = "NORMAL"
    
    return {
        'voltage_mV': int(mv),
        'voltage_V': round(voltage_v, 2),
        'soc': round(soc, 1),
        'alarm': alarm,
        'should_shutdown': should_shutdown,
        'status': status
    }

# === 示例使用 ===
if __name__ == "__main__":
    # 模拟一系列电压读数
    test_voltages = [12.6, 12.0, 11.6, 11.4, 11.3, 11.25, 11.15, 11.05, 10.8, 9.0]
    
    print(f"{'电压(V)':<8} {'SOC(%)':<8} {'报警':<6} {'停机':<6} {'状态'}")
    print("-" * 60)
    
    for v in test_voltages:
        status = get_battery_status(v)
        alarm_str = "是" if status['alarm'] else "否"
        shutdown_str = "是" if status['should_shutdown'] else "否"
        print(f"{status['voltage']:<8} {status['soc']:<8} {alarm_str:<6} {shutdown_str:<6} {status['status']}")

    # 模拟输入：毫伏（mV）
    test_mv_values = [
        12600,  # 12.6V
        12000,  # 12.0V
        11390,  # 11.4V
        11233,  # 11.3V
        11250,  # 11.25V
        11150,  # 11.15V
        11050,  # 11.05V
        10800,  # 10.8V
        9000    # 9.0V
    ]
    
    print(f"{'电压(mV)':<8} {'电压(V)':<6} {'SOC(%)':<8} {'报警':<6} {'停机':<6} {'状态'}")
    print("-" * 70)
    
    for mv in test_mv_values:
        status = get_battery_status_mv(mv)
        alarm_str = "是" if status['alarm'] else "否"
        shutdown_str = "是" if status['should_shutdown'] else "否"
        print(f"{status['voltage_mV']:<8} {status['voltage_V']:<6} {status['soc']:<8} "
              f"{alarm_str:<6} {shutdown_str:<6} {status['status']}")