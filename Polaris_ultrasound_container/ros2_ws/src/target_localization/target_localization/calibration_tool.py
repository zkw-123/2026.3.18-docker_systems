#!/usr/bin/env python3
# coding: UTF-8

"""
标定工具脚本
用于计算从marker到超声图像的变换矩阵

标定方法：
1. 点对点配准（至少需要3个非共线点）
2. 手眼标定（推荐，需要多组不同姿态的数据）
"""

import numpy as np
import json
import os
import cv2
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

class CalibrationTool:
    """标定工具类"""
    
    def __init__(self):
        self.image_points = []  # 图像中的点（像素坐标）
        self.marker_transforms = []  # 对应的marker变换矩阵
        
    def add_calibration_point(self, image_point, marker_transform):
        """
        添加标定点
        
        参数:
            image_point: (x, y) 图像像素坐标
            marker_transform: 4x4 numpy array, marker的变换矩阵
        """
        self.image_points.append(np.array(image_point))
        self.marker_transforms.append(np.array(marker_transform))
        
    def load_from_phantom_data(self, data_dir, pixel_to_mm_x=0.1, pixel_to_mm_y=0.1):
        """
        从phantom_experiment.py生成的数据中加载标定点
        
        参数:
            data_dir: phantom数据目录
            pixel_to_mm_x, pixel_to_mm_y: 像素到mm的转换（粗略估计）
        """
        files = [f for f in os.listdir(data_dir) if f.endswith('.json')]
        
        for filename in files:
            filepath = os.path.join(data_dir, filename)
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # 获取stylus位置（作为图像中的点）
            stylus_matrix = np.array(data['stylus']['matrix'])
            stylus_pos = stylus_matrix[:3, 3]
            
            # 获取phantom的变换矩阵（作为marker）
            phantom_matrix = np.array(data['phantom']['matrix'])
            
            # 将stylus位置投影到图像坐标（这里需要您手动指定图像坐标）
            # 或者使用其他方法记录图像坐标
            print(f"Point: {filename}")
            print(f"Stylus 3D position: {stylus_pos}")
            
            # 这里需要您手动输入对应的图像坐标
            print("请输入该点在图像中的像素坐标:")
            x = float(input("  x (pixel): "))
            y = float(input("  y (pixel): "))
            
            self.add_calibration_point((x, y), phantom_matrix)
    
    def compute_point_to_point_registration(self, pixel_to_mm_x=0.1, pixel_to_mm_y=0.1):
        """
        使用点对点配准计算变换矩阵
        至少需要3个非共线的点
        
        返回:
            4x4 变换矩阵（从marker坐标系到图像坐标系）
        """
        if len(self.image_points) < 3:
            raise ValueError("至少需要3个标定点")
        
        # 将图像点转换为3D点（假设z=0）
        image_points_3d = []
        for pt in self.image_points:
            x = pt[0] * pixel_to_mm_x
            y = pt[1] * pixel_to_mm_y
            z = 0.0
            image_points_3d.append([x, y, z])
        
        image_points_3d = np.array(image_points_3d)
        
        # 从marker变换矩阵中提取点的世界坐标
        # 这里假设我们追踪的是stylus，其尖端就是我们的点
        world_points = []
        for T in self.marker_transforms:
            # 假设尖端在marker坐标系的原点
            point_in_marker = np.array([0, 0, 0, 1])
            point_in_world = T @ point_in_marker
            world_points.append(point_in_world[:3])
        
        world_points = np.array(world_points)
        
        # 计算变换
        # 找到从world坐标到image坐标的变换
        # 首先找到质心
        centroid_image = np.mean(image_points_3d, axis=0)
        centroid_world = np.mean(world_points, axis=0)
        
        # 中心化
        image_centered = image_points_3d - centroid_image
        world_centered = world_points - centroid_world
        
        # 使用SVD计算旋转矩阵
        H = world_centered.T @ image_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 确保是右手坐标系
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 计算平移
        t = centroid_image - R @ centroid_world
        
        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        return T
    
    def compute_hand_eye_calibration(self):
        """
        使用手眼标定算法（推荐方法）
        需要至少3组不同姿态的数据
        
        使用AX=XB形式的手眼标定
        A: marker的运动（已知）
        B: 图像坐标系相对于世界坐标系的运动（未知）
        X: 要求解的marker到图像的固定变换
        
        返回:
            4x4 变换矩阵（从marker坐标系到图像坐标系）
        """
        if len(self.marker_transforms) < 3:
            raise ValueError("手眼标定至少需要3组数据")
        
        # OpenCV的手眼标定需要相对运动
        # 计算相对变换
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        
        # 使用第一个位置作为参考
        T_base = self.marker_transforms[0]
        T_base_inv = np.linalg.inv(T_base)
        
        for i in range(1, len(self.marker_transforms)):
            # Marker的相对运动
            T_rel = T_base_inv @ self.marker_transforms[i]
            R_gripper2base.append(T_rel[:3, :3])
            t_gripper2base.append(T_rel[:3, 3].reshape(3, 1))
            
            # 对于图像点，我们假设它们在图像坐标系中的位置不变
            # 所以相对运动是单位矩阵（这里需要更复杂的处理）
            # 实际应用中，您需要记录每个姿态下图像中点的位置变化
            R_target2cam.append(np.eye(3))
            t_target2cam.append(np.zeros((3, 1)))
        
        # 使用OpenCV的手眼标定
        # 注意：这需要您记录每个marker姿态下图像的变化
        # 如果使用固定的phantom，这个方法需要修改
        
        print("警告：手眼标定需要更多的数据和特殊的采集流程")
        print("建议使用点对点配准方法或专业的标定工具")
        
        return None
    
    def compute_pivot_calibration(self, transforms, tip_offset=None):
        """
        计算工具尖端相对于marker的偏移（pivot calibration）
        
        参数:
            transforms: list of 4x4 numpy arrays, marker在不同姿态下的变换
            tip_offset: 如果已知，可以提供初始估计
        
        返回:
            tip_position: 3D位置，工具尖端在marker坐标系中的位置
        """
        if len(transforms) < 4:
            raise ValueError("pivot标定至少需要4个不同姿态")
        
        # 构建线性方程组 Ax = b
        # 对于每个姿态i: R_i * p + t_i = pivot_point (在世界坐标系中固定)
        # 重写为: [R_i, -I] * [p; pivot_point] = -t_i
        
        n = len(transforms)
        A = np.zeros((3*n, 6))
        b = np.zeros((3*n, 1))
        
        for i, T in enumerate(transforms):
            R = T[:3, :3]
            t = T[:3, 3].reshape(3, 1)
            
            A[3*i:3*i+3, 0:3] = R
            A[3*i:3*i+3, 3:6] = -np.eye(3)
            b[3*i:3*i+3] = -t
        
        # 最小二乘求解
        x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        
        tip_position = x[0:3].flatten()
        pivot_point = x[3:6].flatten()
        
        # 计算误差
        error = np.sqrt(residuals[0] / n) if len(residuals) > 0 else 0
        
        print(f"Pivot calibration results:")
        print(f"  Tip position in marker frame: {tip_position}")
        print(f"  Pivot point in world frame: {pivot_point}")
        print(f"  RMS error: {error:.4f} mm")
        
        return tip_position, pivot_point, error
    
    def save_calibration(self, transform_matrix, filename):
        """保存标定结果"""
        calibration_data = {
            'marker_to_image_transform': transform_matrix.tolist(),
            'num_points': len(self.image_points),
            'image_points': [pt.tolist() for pt in self.image_points],
            'marker_transforms': [T.tolist() for T in self.marker_transforms]
        }
        
        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"标定结果已保存到: {filename}")
    
    def load_calibration(self, filename):
        """加载标定结果"""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        transform = np.array(data['marker_to_image_transform'])
        return transform
    
    def evaluate_calibration_error(self, transform_matrix, pixel_to_mm_x=0.1, pixel_to_mm_y=0.1):
        """
        评估标定误差
        
        返回:
            errors: 每个点的误差（mm）
            mean_error: 平均误差
            std_error: 误差标准差
        """
        errors = []
        
        for i, (img_pt, marker_T) in enumerate(zip(self.image_points, self.marker_transforms)):
            # 图像点转换为3D（假设z=0）
            img_3d = np.array([
                img_pt[0] * pixel_to_mm_x,
                img_pt[1] * pixel_to_mm_y,
                0.0,
                1.0
            ])
            
            # 使用标定的变换预测marker位置
            predicted_in_marker = np.linalg.inv(transform_matrix) @ img_3d
            
            # 实际的marker位置（假设工具尖端在原点）
            actual_in_marker = np.array([0, 0, 0, 1])
            
            # 变换到世界坐标系比较
            predicted_in_world = marker_T @ predicted_in_marker
            actual_in_world = marker_T @ actual_in_marker
            
            # 计算误差
            error = np.linalg.norm(predicted_in_world[:3] - actual_in_world[:3])
            errors.append(error)
            
            print(f"Point {i+1}: error = {error:.2f} mm")
        
        errors = np.array(errors)
        mean_error = np.mean(errors)
        std_error = np.std(errors)
        
        print(f"\n标定精度评估:")
        print(f"  平均误差: {mean_error:.2f} mm")
        print(f"  标准差: {std_error:.2f} mm")
        print(f"  最大误差: {np.max(errors):.2f} mm")
        print(f"  最小误差: {np.min(errors):.2f} mm")
        
        return errors, mean_error, std_error


def interactive_calibration():
    """交互式标定流程"""
    print("=" * 60)
    print("超声-追踪系统标定工具")
    print("=" * 60)
    
    tool = CalibrationTool()
    
    print("\n选择标定方法:")
    print("1. 手动输入标定点")
    print("2. 从phantom数据加载")
    print("3. Pivot标定（工具尖端标定）")
    
    choice = input("\n请选择 (1/2/3): ")
    
    if choice == '1':
        # 手动输入标定点
        print("\n请准备至少3个标定点")
        print("对于每个点，您需要:")
        print("  1. 记录该点在图像中的像素坐标")
        print("  2. 记录同时刻marker的变换矩阵（从NDI数据）")
        
        num_points = int(input("\n要输入多少个标定点? "))
        
        for i in range(num_points):
            print(f"\n--- 标定点 {i+1} ---")
            x = float(input("图像x坐标 (pixel): "))
            y = float(input("图像y坐标 (pixel): "))
            
            print("请输入marker的4x4变换矩阵 (16个数，按行输入):")
            matrix_values = []
            for row in range(4):
                row_str = input(f"  第{row+1}行 (4个数，空格分隔): ")
                matrix_values.extend([float(x) for x in row_str.split()])
            
            marker_T = np.array(matrix_values).reshape(4, 4)
            tool.add_calibration_point((x, y), marker_T)
        
        # 计算标定
        print("\n正在计算标定...")
        pixel_to_mm_x = float(input("像素到mm转换比例 X (默认0.1): ") or "0.1")
        pixel_to_mm_y = float(input("像素到mm转换比例 Y (默认0.1): ") or "0.1")
        
        try:
            transform = tool.compute_point_to_point_registration(pixel_to_mm_x, pixel_to_mm_y)
            print("\n标定成功!")
            print("Marker到图像的变换矩阵:")
            print(transform)
            
            # 评估误差
            tool.evaluate_calibration_error(transform, pixel_to_mm_x, pixel_to_mm_y)
            
            # 保存
            save_path = input("\n保存标定结果到 (默认: calibration_result.json): ") or "calibration_result.json"
            tool.save_calibration(transform, save_path)
            
        except Exception as e:
            print(f"标定失败: {str(e)}")
    
    elif choice == '2':
        # 从phantom数据加载
        data_dir = input("\nPhantom数据目录路径: ")
        pixel_to_mm_x = float(input("像素到mm转换比例 X (默认0.1): ") or "0.1")
        pixel_to_mm_y = float(input("像素到mm转换比例 Y (默认0.1): ") or "0.1")
        
        tool.load_from_phantom_data(data_dir, pixel_to_mm_x, pixel_to_mm_y)
        
        if len(tool.image_points) >= 3:
            transform = tool.compute_point_to_point_registration(pixel_to_mm_x, pixel_to_mm_y)
            print("\n标定成功!")
            print("变换矩阵:")
            print(transform)
            
            # 保存
            save_path = input("\n保存标定结果到: ") or "calibration_result.json"
            tool.save_calibration(transform, save_path)
    
    elif choice == '3':
        # Pivot标定
        print("\nPivot标定流程:")
        print("1. 将工具尖端固定在一个点上")
        print("2. 围绕该固定点旋转工具，记录多个不同姿态")
        print("3. 每个姿态记录marker的变换矩阵")
        
        num_poses = int(input("\n记录了多少个姿态? "))
        transforms = []
        
        for i in range(num_poses):
            print(f"\n姿态 {i+1}:")
            print("请输入marker的4x4变换矩阵 (16个数，按行输入):")
            matrix_values = []
            for row in range(4):
                row_str = input(f"  第{row+1}行 (4个数，空格分隔): ")
                matrix_values.extend([float(x) for x in row_str.split()])
            
            T = np.array(matrix_values).reshape(4, 4)
            transforms.append(T)
        
        try:
            tip_pos, pivot_point, error = tool.compute_pivot_calibration(transforms)
            
            print(f"\nPivot标定完成!")
            print(f"工具尖端在marker坐标系中的位置: {tip_pos}")
            print(f"Pivot点在世界坐标系中的位置: {pivot_point}")
            print(f"RMS误差: {error:.4f} mm")
            
            # 保存
            save_path = input("\n保存结果到: ") or "pivot_calibration.json"
            result = {
                'tip_position': tip_pos.tolist(),
                'pivot_point': pivot_point.tolist(),
                'rms_error': float(error)
            }
            with open(save_path, 'w') as f:
                json.dump(result, f, indent=2)
            
        except Exception as e:
            print(f"标定失败: {str(e)}")


if __name__ == '__main__':
    interactive_calibration()
