
from remote5g_autonomy.options import FollowOptions
from remote5g_autonomy.control import FollowController

# 模拟驾驶参数
options = FollowOptions(
    drive=True,
    continuous=True,
    speed=0.3,
    required_good_frames=3
)

# 创建控制器，不创建 ROS 节点
controller = FollowController(options, now=0.0)

# 模拟里程计
controller.odometry((0.0, 0.0), now=0.0)

def check(label, now):
    decision = controller.step(now)
    print(
        label,
        "速度:", decision.speed,
        "转向:", decision.steering,
        "有效帧:", controller.good_frames
    )

check("初始状态", 0.0)

# 模拟相机连续识别到车道
controller.observe(0.1, -0.035, 0.1)
check("第1帧", 0.1)

controller.observe(0.1, -0.035, 0.2)
check("第2帧", 0.2)

controller.observe(0.1, -0.035, 0.3)
check("第3帧", 0.3)

# 模拟一次车道识别失败
controller.observe(None, 0.0, 0.4)
check("丢线", 0.4)

# 模拟车道恢复
controller.observe(0.1, -0.035, 0.5)
check("恢复第1帧", 0.5)

controller.observe(0.1, -0.035, 0.6)
check("恢复第2帧", 0.6)

controller.observe(0.1, -0.035, 0.7)
check("恢复第3帧", 0.7)

