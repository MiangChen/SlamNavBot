import numpy as np

from llm.voice_unitree import run_once, speak, speaker_unitree
from robot.robot_h1_noentity import RobotG1

from g1_interface import sport_client
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

ip_g1 = "eno1"
try:
    ChannelFactoryInitialize(0, ip_g1)
except Exception as e:
    print("Exception init speaker:", e)

if __name__ == "__main__":

    # usd_path = f'{PATH_PROJECT}/scene/flatroom2.usd'
    # usd_abs_path = os.path.abspath(usd_path)
    # env = Env(simulation_app, usd_abs_path)
    # env.reset()
    flag_llm = True
    g1 = RobotG1()
    while flag_llm == True:
        input("按下 Enter 开始一次录音（3.5秒）...")
        # x, y, z = run_once(file_path="coffee.wav")
        x, y, z = run_once(ip_speaker=ip_g1)
        # x, y, z = 0, 3.4, 0
        print(f"获取到的坐标: ({x}, {y}, {z})")

        # 添加坐标确认环节
        while True:
            confirmation = input("确认坐标是否正确？(yes/no): ").strip().lower()
            if confirmation in ['no', 'n']:
                print("坐标未确认，将重新录音...")
                # 跳出确认循环，重新开始录音循环
                break
            else:
                print("pass ")
                break

        # 如果用户选择no，则跳过导航步骤，重新开始录音
        if confirmation in ['no', 'n'] or z == None:
            speaker_unitree.speak("你好，暂时不支持导航到该地点，请重新输入")
            continue

        # 执行导航
        speaker_unitree.speak("正在为您规划路径")
        flag_navigation = True
        g1.navigate_to(pos_target=np.array([x, y, z], dtype=np.float32), load_from_file=True)
        i = 0
        speaker_unitree.speak("我们开始出发吧")
        sport_client.WaveHand()



        while flag_navigation == True:

            flag_navigation = g1.flag_action_navigation
            g1.on_physics_step()  # navigation -> move along path -> move to -> base command
            i += 1
            if i % 30 == 0:
                i = 0
                # print(
                #     f"g1.target = {g1.target_pose}\tg1.pose = {g1.pose}\t g1.yaw = {g1.yaw}\t g1.base_command= {g1.base_command}")
                # print(flag_navigation)
            sport_client.Move(vx=g1.base_command[0], vy=0, vyaw= -1 * g1.base_command[2], time=0.5)  #

        speaker_unitree.speak("我们到目标点了，有需求请重新吩咐我")
        sport_client.WaveHand()
