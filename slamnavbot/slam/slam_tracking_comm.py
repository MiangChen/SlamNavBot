import paramiko
import time
import os
from pathlib import Path

class ImageTransfer:
    def __init__(self, hostname, username, password, port=22):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.port = port
        self.ssh = None
        self.sftp = None
        
    def connect(self):
        """建立SSH连接"""
        try:
            self.ssh = paramiko.SSHClient()
            self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh.connect(self.hostname, self.port, self.username, self.password)
            self.sftp = self.ssh.open_sftp()
            print("成功连接到远程服务器")
        except Exception as e:
            print(f"连接失败: {str(e)}")
            raise

    def upload_image(self, local_path, remote_path):
        """上传图片到远程服务器"""
        try:
            self.sftp.put(local_path, remote_path)
            print(f"成功上传图片: {local_path} -> {remote_path}")
        except Exception as e:
            print(f"上传失败: {str(e)}")

    def download_result(self, remote_path, local_path):
        """从远程服务器下载处理后的数据"""
        try:
            self.sftp.get(remote_path, local_path)
            print(f"成功下载文件: {remote_path} -> {local_path}")
            return True
        except Exception as e:
            print(f"下载失败: {str(e)}")
            return False

    def get_remote_file_mtime(self, remote_path):
        """获取远程文件的最后修改时间"""
        try:
            stat = self.sftp.stat(remote_path)
            return stat.st_mtime
        except Exception as e:
            print(f"获取远程文件状态失败: {str(e)}")
            return None

    def file_exists(self, remote_path):
        """检查远程文件是否存在"""
        try:
            self.sftp.stat(remote_path)
            return True
        except FileNotFoundError:
            return False
        except Exception as e:
            print(f"检查文件存在性失败: {str(e)}")
            return False
        
    def file_completed(self, remote_path):
        """检查远程文件是否完成"""
        try:
            # 获取文件大小
            current_stat = self.sftp.stat(remote_path)
            time.sleep(0.001)
            last_stat = self.sftp.stat(remote_path)
            return current_stat.st_size > 0 and current_stat.st_size == last_stat.st_size
        except Exception as e:
            print(f"检查文件完成性失败: {str(e)}")
            return False

    def close(self):
        """关闭连接"""
        if self.sftp:
            self.sftp.close()
        if self.ssh:
            self.ssh.close()

def main():
    # 配置参数
    SERVER_CONFIG = {
        'hostname': '103.237.29.211',
        'username': 'chuangzhi',
        'password': 'Cz9024)dc!',
        'port': 9024
    }
    
    LOCAL_IMAGE_PATH = r"/home/zqh/yyx_G1/G1_workspace/G1_output/color/g1_images.jpg"  # 需要上传的图片路径
    LOCAL_RESULT_PATH = r"/home/zqh/yyx_G1/G1_workspace/G1_output/g1_poses.json"  # 本地保存处理后结果的路径
    # For mac/linux 
    REMOTE_IMAGE_PATH = '/home/chuangzhi/Project_EmbodiedAI_g1/MASt3R-SLAM/communication/g1_images.jpg'  # 远程服务器上的图片保存路径
    # For windows
    # REMOTE_IMAGE_PATH = '/home/chuangzhi/Project_EmbodiedAI_g1/MASt3R-SLAM/communication'
    REMOTE_RESULT_PATH = '/home/chuangzhi/Project_EmbodiedAI_g1/MASt3R-SLAM/communication/g1_poses.json'  # 远程服务器上处理后的结果路径

    # 创建本地结果保存目录
    os.makedirs(os.path.dirname(LOCAL_RESULT_PATH), exist_ok=True)
    
    # 创建传输对象
    transfer = ImageTransfer(**SERVER_CONFIG)
    
    # 记录上次下载的文件修改时间
    last_input_mtime = 0
    last_result_mtime = None
    
    try:
        # 连接到服务器
        transfer.connect()
        
        while True:
            # 上传图片
            start_time = time.time()
            if os.path.exists(LOCAL_IMAGE_PATH) and os.path.getmtime(LOCAL_IMAGE_PATH) != last_input_mtime:
                last_input_mtime = os.path.getmtime(LOCAL_IMAGE_PATH)
                print(f"Latest input mtime: {last_input_mtime}")
                transfer.upload_image(LOCAL_IMAGE_PATH, REMOTE_IMAGE_PATH)
                
                # 检查远程结果文件是否有更新，设置超时时间为5秒
                wait_start = time.time()
                while time.time() - wait_start < 0.6:  # 5秒超时
                    if transfer.file_exists(REMOTE_RESULT_PATH):
                        current_mtime = transfer.get_remote_file_mtime(REMOTE_RESULT_PATH)
                        
                        if current_mtime is not None and current_mtime != last_result_mtime:
                            # 文件有更新，下载处理后的结果
                            if transfer.download_result(REMOTE_RESULT_PATH, LOCAL_RESULT_PATH):
                                last_result_mtime = current_mtime
                                end_time = time.time()
                                print(f"processing time: {end_time - start_time} s")
                                break
                            else:
                                print("下载失败，保持上次的修改时间")
                    time.sleep(0.1)  # 等待100ms再检查
                else:
                    print("等待结果超时，继续处理下一帧")
            
            # 短暂休眠，避免CPU占用过高
            time.sleep(0.05)  # 50ms的休眠时间
            
    except KeyboardInterrupt:
        print("\nprogram stopped")
    finally:
        transfer.close()

if __name__ == "__main__":
    main()
