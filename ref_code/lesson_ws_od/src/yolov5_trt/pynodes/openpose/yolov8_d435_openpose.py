#!/usr/bin/env python3.8
import rospy
import cv2
import argparse
import time
from ultralytics import YOLO
import numpy as np
import supervision as sv
import pyrealsense2 as rs
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from object_detection_msgs.msg import RealsenseTarget
from object_detection_msgs.msg import RealsenseTargets

import runOpenpose
from runOpenpose import normalFlag
from torch import from_numpy, jit
import torch
import torch.backends.cudnn as cudnn
import tensorrt as trt
# rs = RealsenseCamera()
pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
serial_number = '151422251310'
config.enable_device(serial_number)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    #json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 live")
    parser.add_argument(
        "--webcam-resolution", 
        default=[1280, 720], 
        nargs=2, 
        type=int
    )
    args = parser.parse_args()
    return args


def main():
    # args = parse_arguments()
    # frame_width, frame_height = args.webcam_resolution
    device = torch.device("cpu")
    print("加载摔倒检测的模型开始")
    net = jit.load(r"./openpose.jit",device)
    action_net = jit.load(r"./action.jit",device)
    print("加载摔倒检测的模型结束")

    model = YOLO("./yolov8n.pt")#best.pt只能检测人体

    box_annotator = sv.BoxAnnotator(
        thickness=2,
        text_thickness=2,
        text_scale=1
    )

    while True:
        # ret, frame, depth_frame = RealsenseCamera().get_frame_stream()
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
        rs_targets_pub = rospy.Publisher('/drone0/yolov8_detector/realsense_targets', RealsenseTargets, queue_size=5)
        t_start = time.time()  # 开始计时
        # result = model(frame, agnostic_nms=True,conf=0.5)[0]
        result = model(color_image, agnostic_nms=True,conf=0.5)[0]
        detections = sv.Detections.from_yolov8(result)
        #At follows declare the class person asobject to detect 仅仅只检测人体
        detections = detections[(detections.class_id == 0)]
        labels = [
            f"{model.model.names[class_id]} {confidence:0.2f}"
            for _, confidence, class_id, _
            in detections
        ]
        object_frame = box_annotator.annotate(
            # scene=frame, 
            scene=color_image,
            detections=detections, 
            labels=labels
        )
        
        coord = detections.xyxy
        x1 = 0.0
        x2 = 0.0
        y1 = 0.0
        y2 = 0.0
        center_x = 0.0
        center_y = 0.0
        width = x2-x1
        height = y2-y1
        point_3d=(0.0,0.0,0.0)
        rs_tg = RealsenseTarget()
        class_id_list = [0]
        # 画面中存在目标对象
        # 将坐标信息恢复到原始图像的尺寸
        rs_targets = RealsenseTargets(Header(None, rospy.Time.now(), ''), []) # list
        for i in range(len(coord)):

            print('进行人体姿态检测')
            runOpenpose.run_demo(net, action_net, [color_image], 256, False, [coord[i]])  # 人体姿态检测 将图片和yolov5检测人体的框也传给openpose
            
            x1 = coord[i][0]
            x2 = coord[i][2]
            y1 = coord[i][1]
            y2 = coord[i][3]
            center_x = (x1+x2)/2
            center_y = (y1+y2)/2
            width = x2-x1
            height = y2-y1
            # print("Center coordinate is",(center_x,center_y))
            class_id_list.append(detections.class_id)
            # rs_tg.id = detections.class_id
            rs_tg.id = 0
            rs_tg.xmin = int(x1)
            rs_tg.ymin = int(y1)
            rs_tg.xmax = int(x2)
            rs_tg.ymax = int(y2)
            rs_tg.conf = 0
            rs_tg.cls = 0
            #rs_tg.is_normal = runOpenpose.normalFlag
            # realsense 3d检测
            [pixel_x, pixel_y] = [int(center_x), int(center_y)] # 可能中心点未落在目标上，待改进
            dist_to_pixel = float(0)
            [human_pixel_x,human_pixel_y] = [int((runOpenpose.human_pose_box[0]+runOpenpose.human_pose_box[2])/2),int((runOpenpose.human_pose_box[1]+runOpenpose.human_pose_box[3])/2)]
            dis = aligned_depth_frame.get_distance(pixel_x, pixel_y)

            bbox_w = int(runOpenpose.human_pose_box[2] - runOpenpose.human_pose_box[0])
            bbox_h = int(runOpenpose.human_pose_box[3] - runOpenpose.human_pose_box[1])
            #计算openpose深度
            for delta_x in range(int(-bbox_w / 4), int(bbox_w / 4)):
                for delta_y in range(int(-bbox_h / 4), int(bbox_h / 4)): # 扫描中心点周围半个边框大的区域
                    scan_dist = aligned_depth_frame.get_distance(human_pixel_x + delta_x, human_pixel_y + delta_y)
                    if scan_dist != 0:
                        if dist_to_pixel == 0 or scan_dist < dist_to_pixel:
                            dist_to_pixel = scan_dist
            
            # #计算深度
            # for delta_x in range(int(-width / 4), int(width / 4)):
            #     for delta_y in range(int(-height / 4), int(height / 4)): # 扫描中心点周围半个边框大的区域
            #         scan_dist = aligned_depth_frame.get_distance(pixel_x + delta_x, pixel_y + delta_y)
            #         if scan_dist != 0:
            #             if dist_to_pixel == 0 or scan_dist < dist_to_pixel:
            #                 dist_to_pixel = scan_dist

            if(dist_to_pixel == 0): # 如果深度估计有问题,则忽略该目标
                continue
            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [pixel_x, pixel_y], dist_to_pixel)# 这样深度估计会偏差大一些，但是更稳定
            rs_tg.position_from_realsense = Vector3(point_3d[0], point_3d[1], point_3d[2])
            rs_targets.realsense_targets.append(rs_tg) # list
        t_end = time.time()  # 结束计时
        # 添加fps显示
        fps = int(1.0 / (t_end - t_start))  
        cv2.putText(color_image, "x: " + str(round(point_3d[0], 3)), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(color_image, "y: " + str(round(point_3d[1], 3)), (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(color_image, "z: " + str(round(point_3d[2], 3)), (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(color_image, text="FPS: {}".format(fps), org=(5, 130), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=2,lineType=cv2.LINE_AA, color=(0, 255, 0))
        rs_targets_pub.publish(rs_targets) # 发送检测信息

        center_x = int(center_x)
        center_y = int(center_y)
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)
        width = int(width)
        height = int(height)
        
        cv2.imshow("color", color_image)
        

        if (cv2.waitKey(30) == 27):
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    rospy.init_node('drone0_detect_human')
    main()

pipeline.stop()
