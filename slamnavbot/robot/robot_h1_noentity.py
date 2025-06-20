# from isaacsim.core.api.scenes import Scene
from typing import Tuple
from map.map_grid_map import GridMap
from controller.controller_pid import ControllerPID
# from robot.robot_base import RobotBase
# from robot.robot_trajectory import Trajectory
# from robot.robot_cfg_h1 import RobotCfgH1
# from controller.controller_policy_h1 import H1FlatTerrainPolicy
from path_planning.path_planning_astar import AStar
import numpy as np


class RobotG1():
    def __init__(self) -> None:
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(1, 0, 0.1, target=0)
        self.map_grid = GridMap()
        self.flag_world_reset = True
        self.base_command = [0, 0, 0]
        self.pose = [0, 0, 0]
        self.yaw = 0  # -pi ~ pi
        self.target_pose = [0, 0, 0]
        return

    def get_world_poses(self) -> Tuple[np.ndarray, np.ndarray]:
        from G1_workspace.Filtering import read_and_filter_poses
        flag = False
        count = 0  # 如果连续5次都获取不料位置，就重新开始
        while flag == False and count < 5:
            dict_data = read_and_filter_poses(target_confidence=2)
            count += 1
            if dict_data != None:
                self.dict_data = dict_data
                flag = True
        #print("pose", dict_data)
        self.pose = self.dict_data["position"]
        self.quat = self.dict_data["quat"]
        from utils.transform import transform_object_rotate_then_translate, translate_pos, rotate, quaternion_to_euler_scipy, extract_yaw_from_rotation, R
        self.angle = quaternion_to_euler_scipy(self.quat, 'xyzw', euler_order='xyz')
        self.pose, self.angle = transform_object_rotate_then_translate(
            self.pose, self.angle,
            translate_pos,
            rotate,
            rotation_order='xyz',
        )

        self.yaw = extract_yaw_from_rotation(R.from_euler('xyz', self.angle, degrees=True))
        # print("new pose", self.pose, self.yaw)
        return self.pose, self.yaw

    def move_along_path(self, path: list = None, flag_reset: bool = False) -> bool:
        """
        让机器人沿着一个list的路径点运动
        需求: 在while外面 能够记录已经到达的点, 每次到达某个目标点的 10cm附近,就认为到了, 然后准备下一个点
        """
        if flag_reset == True:
            self.path_index = 0
            self.path = path

        if self.path_index < len(self.path):  # 当index==len的时候, 就已经到达目标了
            print("self.path index", self.path_index, "len self. path", len(self.path))
            flag_reach = self.move_to(self.path[self.path_index])
            if flag_reach == True:
                self.path_index += 1
                self.target_pose = self.path[self.path_index-1]
            return False
        else:
            self.flag_action_navigation = False
            self.state_skill_complete = True
            return True

    def move_to(self, target_pos):
        import numpy as np
        """
        让2轮差速小车在一个2D平面上运动到目标点
        缺点：不适合3D，无法避障，地面要是平的
        速度有两个分两，自转的分量 + 前进的分量
        """
        pos, yaw = self.get_world_poses()  # self.get_world_pose()  ## type np.array
        # 获取2D方向的朝向，逆时针是正
        # yaw = self.quaternion_to_yaw(quat)
        # 获取机器人和目标连线的XY平面上的偏移角度
        vector = [target_pos[0] - pos[0], target_pos[1] - pos[1]]
        from utils.transform import calculate_angle_with_y_axis
        robot_to_target_angle = calculate_angle_with_y_axis(vector) # np.arctan2(target_pos[1] - pos[1], target_pos[0] - pos[0])
        # 差速, 和偏移角度成正比，通过pi归一化
        delta_angle = robot_to_target_angle - yaw
        delta_angle = delta_angle/180*3.14
        if abs(delta_angle) < 10/180*3.14:  # 角度控制死区
            delta_angle = 0
        elif delta_angle < -np.pi:  # 当差距abs超过pi后, 就代表从这个方向转弯不好, 要从另一个方向转弯
            delta_angle += 2 * np.pi
        elif delta_angle > np.pi:
            delta_angle -= 2 * np.pi
        if np.linalg.norm(target_pos[0:2] - pos[0:2]) < 0.5:
            self.action = [0, 0]
            return True  # 已经到达目标点附近50cm, 停止运动
        # print(delta_angle, np.linalg.norm(target_pos[0:2] - pos[0:2]))
        k_rotate = 1 / np.pi
        v_rotation = self.pid_angle.compute(delta_angle, dt=1 / 2)
        max_v_rotation = 0.2
        if v_rotation > max_v_rotation:
            v_rotation = max_v_rotation
        elif v_rotation < -max_v_rotation:
            v_rotation = -max_v_rotation
        # dang xuan zhuan sudu tai da, xian xuanzhuan , hou tingzhi

        # 前进速度，和距离成正比
        k_forward = 1
        v_forward = 0.3
        v_forward = (1 - abs(v_rotation) / max_v_rotation) * v_forward + 0.1 # xie lv wei K

        self.base_command = [v_forward, 0,  v_rotation]
        return False  # 还没有到达

    def navigate_to(self, pos_target: np.ndarray = None, orientation_target: np.ndarray = None,
                    reset_flag: bool = False, load_from_file: bool = False) -> None:
        """
        让机器人导航到某一个位置,
        不需要输入机器人的起始位置, 因为机器人默认都是从当前位置出发的;
        本质是使用A*等算法, 规划一系列的路径
        Args:
            target_pos: 机器人的目标位置
            target_orientaton: 机器人的目标方向
            reset_flag:

        Returns:
        """
        # 小车/人形机器人的导航只能使用2d的
        # todo: 未来能够爬坡
        if pos_target is None:
            raise ValueError("no target position")
        elif pos_target[2] != 0:
            raise ValueError("小车的z轴高度得在平面上")
        pos_robot = self.get_world_poses()[0]

        pos_index_target = self.map_grid.compute_index(pos_target)
        pos_index_robot = self.map_grid.compute_index(pos_robot)
        pos_index_robot[-1] = 0  # todo : 这也是因为机器人限制导致的

        # 用于把机器人对应位置的设置为空的, 不然会找不到路线
        if load_from_file == True:
            self.map_grid.value_map = np.load("./value_map.npy")
            self.map_grid.pos_map = np.load("./pos_map.npy")
        grid_map = self.map_grid.value_map

        grid_map[pos_index_robot] = self.map_grid.empty_cell

        planner = AStar(grid_map, obs_value=1.0, free_value=0.0, directions="eight", penalty_factor=1 )
        path = planner.find_path(tuple(pos_index_robot), tuple(pos_index_target), render=True)  # shape 100 x 100
        print("Path = ", path)

        real_path = np.zeros_like(path, dtype=np.float32)
        for i in range(path.shape[0]):  # 把index变成连续实际世界的坐标
            real_path[i] = self.map_grid.pos_map[tuple(path[i])]
            real_path[i][-1] = 0
        print("Real Path = ", real_path)
        self.move_along_path(real_path, flag_reset=True)

        # 标记一下, 开始运动
        self.flag_action_navigation = True

        # 标记当前的动作
        self.state_skill = 'navigate_to'
        self.sstate_skill_complete = False

        return

    def on_physics_step(self, step_size=None):
        if self.flag_world_reset == True:
            if self.flag_action_navigation == True:
                self.move_along_path()  # 每一次都计算下速度, self.base_command
            #     self.action = self.controller_policy.forward(step_size, self.base_command, self.robot_entity)
            else:
                self.base_command = [0, 0, 0]  # self.controller_policy.forward(step_size, [0, 0, 0], self.robot_entity)
        return


if __name__ == '__main__':
    h1 = RobotH1()
