import threading
import os
import sys
from . import base as b
# import base as b


# 一个要运行的进程
class yzt_process:
    name = ""   #进程名
    id = -1     #进程id
    valid = 0   #进程是否有效(存活)

    # 构造函数
    def __init__(self, init_name):
        self.name = init_name
        self.id = b.get_process_id(self.name)     # 获得进程ID号
        self.valid = 1                       # 初始为有效的对象
        os.popen(self.name)

    # 看看进程死没死
    def checkValid(self):
        #活着
        if(b.judge_process_exist(self.name)):
            self.valid = 1
        else:
            self.valid = 0
            self.id = -1

    # 重启进程
    def restart(self):
        #自检存活状态
        self.checkValid()
        # 前提是进程不存活
        if(self.valid == 1):
            return
        else:
            self.valid = 1
            self.id = b.get_process_id(self.name)
            os.popen(self.name)

    # 终止进程
    def close(self):
        #自检存活状态
        self.checkValid()
        #前提是进程存活
        if(self.valid == 0):
            return
        else:
            b.kill_process(self.name)
            self.valid = 0       # 变为无效的对象
            self.id = -1


