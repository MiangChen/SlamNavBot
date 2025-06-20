# 1. 启动远程远程服务器的tracking程序
**在远程4090服务器上运行，最好打开tmux窗口运行程序！以免ssh连接中断导致程序被打断**

simplified_localization.py的逻辑是，只要服务器某一路径下的图片有更新，就利用tracking module估计最新图片的pose，写入服务器某一固定路径

```bash
conda activate mast3r-slam
cd /home/chuangzhi/Project_EmbodiedAI_g1/MASt3R-SLAM
python simplified_localization.py --realtime
```


# 2. 在本地运行通讯程序
## 2.1 环境准备
```bash
pip install paramiko
```

## 2.2 本地与远程通讯
本地通讯程序的逻辑是，上传本地某一路径下的图片到服务器的指定路径，当服务器完成tracking，pose文件发生更新，即对pose文件进行下载，写入本地某一固定路径

**注意修改slam_tracking_comm.py的LOCAL_IMAGE_PATH和LOCAL_RESULT_PATH的值！！！**

```bash
python slam_tracking_comm.py
```

# 3. 数据格式
```json
{
  "pose": {
    "translation": [
      0.7425799369812012,
      -1.7064168453216553,
      7.1487016677856445
    ],
    "quaternion": [
      0.0015596403973177075,
      0.017249582335352898,
      0.009006285108625889,
      0.9998095035552979
    ]
  },
  "confidence": 4.184745788574219
}
```
- translation: (x, y, z)
- quaternion: (qx, qy, qz, qw)
- confidence: 表示当前pose估计的可信度，越高越好