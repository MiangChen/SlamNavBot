1.按照SLAM_README.md中的方法修改好文件路径后（注意路径前面的r，windows用户要用这个，苹果用户删掉就好了），在ate_record_one.py文件中修改图片保存路径
```
folder = r"C:\Users\22754\OneDrive\Desktop\G1_workspace\G1_output"
```
2.运行流程：
使用tape—c口进行相机的连接，先运行slam_tracking_comm.py,再运行date_record_one.py(其实先后顺序没关系)，再运行filitering.py，即可记录数据。
