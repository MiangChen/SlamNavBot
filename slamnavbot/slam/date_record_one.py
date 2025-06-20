import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
from datetime import datetime

is_recording = False
# 基础输出文件夹
folder = r"/home/zqh/yyx_G1/G1_workspace/G1_output"

# 创建输出文件夹
def create_output_folders():
    # 创建主输出文件夹
    if not os.path.exists(folder):
        os.makedirs(folder)
        print(f"创建主输出文件夹: {folder}")
    
    # 创建子文件夹
    subfolders = ['color', 'depth_color', 'depth']
    for subfolder in subfolders:
        subfolder_path = os.path.join(folder, subfolder)
        if not os.path.exists(subfolder_path):
            os.makedirs(subfolder_path)
            print(f"创建子文件夹: {subfolder_path}")

def shot(pos, frame, img):
    # 确保文件夹路径以斜杠结尾
    if not folder.endswith('/'):
        folder_path = folder + '/'
    else:
        folder_path = folder
    path = folder_path + img + pos + ".jpg"
    cv2.imwrite(path, frame)
    print("最新帧已更新保存至: " + path)

try:
    # 创建输出文件夹
    create_output_folders()
    
    # 检查是否有RealSense设备连接
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("没有检测到RealSense设备")
        exit(1)

    # 获取第一个设备
    device = devices[0]
    print(f"使用设备: {device.get_info(rs.camera_info.name)}")
    print(f"序列号: {device.get_info(rs.camera_info.serial_number)}")

    # 等待一会儿确保设备初始化完成
    time.sleep(2)

    pipeline = rs.pipeline()
    config = rs.config()

    # 使用设备的序列号来确保我们连接到正确的设备
    serial_number = device.get_info(rs.camera_info.serial_number)
    config.enable_device(serial_number)

    # 配置深度和颜色流
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    print("正在启动数据流...")
    # 尝试启动流
    profile = pipeline.start(config)
    print("数据流启动成功")

    # 等待设备预热
    time.sleep(2)

    # 创建对齐对象
    align_to = rs.stream.color
    align = rs.align(align_to)

    print("开始捕获图像...")
    print("按 'r' 开始/停止记录")
    print("按 'q' 退出程序")
    
    while True:
        try:
            # 设置帧等待超时时间为1000ms
            frames = pipeline.wait_for_frames(1000)
            
            # 将深度框与颜色框对齐
            aligned_frames = align.process(frames)
            
            # 获取对齐后的深度帧
            aligned_depth_frame = aligned_frames.get_depth_frame()
            if not aligned_depth_frame:
                continue
                
            # 处理深度帧
            depth_frame = 50*np.asanyarray(aligned_depth_frame.get_data())
            #cv2.imshow('0 depth', depth_frame)
            
            # 将深度图转化为伪彩色图方便观看
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frame, alpha=0.008),
                cv2.COLORMAP_JET)
            # cv2.imshow('1 depth_color', depth_colormap)

            # 处理彩色帧
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue
            color_frame = np.asanyarray(color_frame.get_data())
            cv2.imshow('2 color', color_frame)

            # 处理键盘事件
            key = cv2.waitKey(1)

            # 按'q'退出
            if key == ord('q'):
                print("用户请求退出")
                break

            # 按'r'开始/停止记录
            if key == ord('r'):
                is_recording = not is_recording
                if is_recording:
                    print("开始记录...")
                else:
                    print("停止记录...")

            # 如果正在记录，保存当前帧
            if is_recording:
                time.sleep(0.05)
                shot('g1_images', color_frame, 'color/')
                shot('latest', depth_colormap, 'depth_color/')
                shot('latest', depth_frame, 'depth/')

        except Exception as e:
            print(f"处理帧时发生错误: {str(e)}")
            continue

except Exception as e:
    print(f"初始化过程中发生错误: {str(e)}")
finally:
    print("正在关闭相机...")
    try:
        pipeline.stop()
    except Exception as e:
        print(f"关闭相机时发生错误: {str(e)}")
    
    # 关闭所有窗口
    cv2.destroyAllWindows()
    print("程序已退出") 