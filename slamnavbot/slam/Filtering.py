import json
import time
import os
from datetime import datetime


def read_and_filter_poses(target_confidence: int = 0.9):
    # 获取程序启动时间，用于文件名
    start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    # 文件路径
    json_path = "/home/zqh/yyx_G1/G1_workspace/G1_output/g1_poses.json"
    # 创建记录文件
    log_dir = "/home/zqh/yyx_G1/G1_workspace/G1_output/pose_logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file = os.path.join(log_dir, f"pos_{start_time}.txt")

    # 记录上次读取的修改时间
    last_mtime = 0

    # 写入文件头部信息
    with open(log_file, 'w', encoding='utf-8') as f:
        f.write(f"Pose记录文件 - 开始时间: {start_time}\n")
        f.write("=" * 50 + "\n\n")

    try:
        # 检查文件是否存在
        if not os.path.exists(json_path):
            time.sleep(0.1)

        # 获取文件的修改时间
        current_mtime = os.path.getmtime(json_path)

        # 如果文件有更新
        if current_mtime != last_mtime:
            last_mtime = current_mtime

            # 读取JSON文件
            with open(json_path, 'r') as f:
                try:
                    data = json.load(f)

                    # 提取数据
                    confidence = data.get('confidence', 0)
                    # 如果confidence是列表，取第一个元素
                    if isinstance(confidence, list):
                        confidence = confidence[0] if confidence else 0

                    pose = data.get('pose', {})
                    translation = pose.get('translation', [0, 0, 0])
                    quaternion = pose.get('quaternion', [0, 0, 0, 1])

                    # 当confidence大于3时打印信息并记录
                    if isinstance(confidence, (int, float)) and confidence > target_confidence:
                        # 获取当前时间
                        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                        # 构建输出信息
                        dict_data = {"time": current_time, "confidence": confidence, "position": translation, "quat": quaternion}
                        return dict_data

                except json.JSONDecodeError:
                    print("JSON解析错误，文件可能正在被写入")
                except Exception as e:
                    print(f"处理数据时发生错误: {str(e)}")
                    print(f"错误的数据: {data}")

        # 短暂休眠，避免CPU占用过高
        time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n程序停止")
        print(f"记录已保存到: {log_file}")
    except Exception as e:
        print(f"发生错误: {str(e)}")
        time.sleep(0.1)


if __name__ == "__main__":
    while True:
        read_and_filter_poses()
