import sys
import os


# 判断进程是否存在的函数
# 传入进程名
def judge_process_exist(process_name):
    # 执行检测指令
    result = os.popen("ps -A | grep -w " + process_name).read()
    # 返回判断值
    if(result):
        return True;
    else:
        return False;



# 获得进程id
def get_process_id(process_name):
    result = os.popen("ps -A | grep -w " + process_name + " | awk \'{print $1}\' ").read()[:-1]
    return result



# 杀死进程
def kill_process(process_name):
    if(judge_process_exist(process_name)):
        process_id = get_process_id(process_name)
        os.popen("echo 1 | sudo -S kill -15 " + process_id)
