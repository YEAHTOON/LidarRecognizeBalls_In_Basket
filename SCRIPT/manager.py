import sys
import os
import time
import select
import fcntl
import subprocess
import signal
import yztpyfun.base as yb
import yztpyfun.pclass as yp


# 基本工作路径
basic_path = "/home/yezhiteng/PROJECTS/PRODUCTS"

# 通信进程
com_path = "./COMMUNICATION/build/"
com_name = "com"

# 检测进程
test_path = "./COMMUNICATION/client/build/"
test_name = "cli"

# 雷达进程
lidar_path = "./ENVIRONMENT_SENSE/build/"
lidar_config_path = "./ENVIRONMENT_SENSE/config.json"
lidar_name = "mid"
# lidar_path = "./COMMUNICATION/client2/build/"
# # lidar_config_path = "./ENVIRONMENT_SENSE/config.json"
# lidar_name = "cli"

#yolo进程
yolo_path = "./RECOGNIZETION_SYSTEM/code/yolov5/"
yolo_name = "rgn"

# python如果没有变量指向对象，对象会被释放，所以要设置全局的变量名
communication_process = 0   # 通信进程
ldiar_process = 0           # 雷达进程






# 创建通信进程，且该进程可以响应TCP请求
def start_communication_process(com_path, com_name, test_path, test_name):

    # 防止进程被释放的变量名
    global communication_process

    # 循环创建进程，直到进程有效地被创建
    # 直到成功启动communication
    while 1:

        # 创建通信进程
        communication_process = os.popen("echo 1 | sudo -S " + com_path + com_name)     # 如果去掉等号左边的变量名，进程会直接释放
        time.sleep(0.5)

        # 判定变量
        com_process_restart_count = 0
        success = 0

        # 启动五次测试程序
        while 1:

            # 重启很多次测试程序了，要重启communication
            if(com_process_restart_count > 2):
                success = 0
                break
            
            # 启动测试进程，获得结果
            result = os.popen(test_path + test_name + " 20000").read()[:-1]

            print("try communication")

            # 根据结果判断成功与否
            if(result == "from communication_process : connection normal"):
                success = 1
                break
            else:
                com_process_restart_count = com_process_restart_count + 1

        # 成功退出循环，否则删掉本次循环创建的无效进程
        if success == 1:
            break
        else:
            yb.kill_process(com_name)


#必须按照先关闭客户端再关闭服务器的顺序
def shutdown_python(sig, frame):
    time.sleep(0.5)
    
    yb.kill_process(lidar_name)
    time.sleep(0.5)
    
    yb.kill_process(yolo_name)
    time.sleep(0.5)
    
    yb.kill_process(com_name)
    time.sleep(0.5)
    
    print("close safely")
    
    sys.exit(0)


# 主函数
if __name__ == "__main__":

    #重定义信号响应
    signal.signal(signal.SIGINT, shutdown_python)

    # 解释器改变路径
    os.chdir(basic_path)

    # 重启子模块次数
    restart_child = 0

    # 主循环，保证所有进程正常运行
    while True:

        # 检测通信进程是否存活
        # 不活着进入if
        if(not yb.judge_process_exist(com_name)):
            # 关闭其他进程
            yb.kill_process(lidar_name)
            yb.kill_process(yolo_name)
            start_communication_process(com_path, com_name, test_path, test_name)
            print("communication process : " + yb.get_process_id(com_name))


        # 活着进入else
        else:

            # 雷达进程不存在，创造一个雷达进程
            if(not yb.judge_process_exist(lidar_name)):

                print("     try lidar")

                # 重启子模块三次不成功，重启交流进程
                restart_child = restart_child + 1
                if(restart_child > 3):
                    restart_child = 0
                    yb.kill_process(com_name)

                # 尝试重启子模块
                else:
                    ldiar_process = os.popen(lidar_path + lidar_name + " " + lidar_config_path)
                    print("     lidar : " + yb.get_process_id(lidar_name))

            #正常状态下，所有进程均已开启
            else:
                pass

            # # yolo进程不存在，创造一个
            # if(not yb.judge_process_exist(yolo_name)):
            #     os.popen("cd " + yolo_path + " && " + "./" + yolo_name)
            # else:
            #     pass
            # pass
            # if(not yb.judge_process_exist(lidar_name)):
            #     ldiar_process = os.popen(lidar_path + lidar_name + " 20000")
            # else:
            #     pass

        time.sleep(0.3)



    