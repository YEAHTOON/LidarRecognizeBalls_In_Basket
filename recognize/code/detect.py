import os
import sys
import numpy
import torch
import random

# 获得点云(未经处理)
def load_point_cloud(pc_path):
    return numpy.loadtxt(pc_path).astype(np.float32)

# 测试
def test():
    numpy.array([1,2])
    print("ok")
    return 1

# 加载模型
def load_pointnet_model(pth_path):
    pointnet_model = torch.load(pth_path)
    pointnet_model.eval()
    return pointnet_model

# 获得估计结果，返回一个一元列表
def get_pred(model, pointcloud, npoints):

    # 点云处理
    # 重采样，能把点云变为要求大小
    choice = numpy.random.choice(pointcloud.shape[0], npoints, replace=True)
    pointcloud = pointcloud[choice, :]
    # 对点云进行处理，包括归一化、质心归于原点
    pointcloud = pointcloud - numpy.expand_dims(numpy.mean(pointcloud, axis = 0), 0) # center
    dist = numpy.max(numpy.sqrt(numpy.sum(pointcloud ** 2, axis = 1)),0)
    pointcloud = pointcloud / dist              #scale
    theta = numpy.random.uniform(0,numpy.pi*2)
    #增强点云
    rotation_matrix = numpy.array([[numpy.cos(theta), -numpy.sin(theta)],[numpy.sin(theta), numpy.cos(theta)]])
    pointcloud[:,[0,2]] = pointcloud[:,[0,2]].dot(rotation_matrix) # random rotation
    pointcloud += numpy.random.normal(0, 0.02, size=pointcloud.shape) # random jitter

    # 估计
    pointcloud = torch.from_numpy(pointcloud)
    pointcloud = pointcloud.unsqueeze(0)        # 在头部增加一个维度，表示batch_size为1
    pointcloud = pointcloud.transpose(2, 1)     # 转换坐标，模型输入要求
    pred, _, _ = model(pointcloud)
    _, pred_choice = pred.data.max(1)

    return list(pred_choice.numpy())

   
# # 测试
# pth_path = "/home/yezhiteng/PROJECTS/PRODUCTS/ENVIRONMENT_SENSE/model_relate/202404102.pth"
# pc_path = "/home/yezhiteng/INCLUDES/pointnet.pytorch-master/20240409dataset/ball_count_0/points/2241679189_2.pts"
# if __name__ == '__main__':
#     pc = load_point_cloud(pc_path)
#     model = load_pointnet_model(pth_path)
#     result = get_pred(model, pc, 300)
#     print(result)
