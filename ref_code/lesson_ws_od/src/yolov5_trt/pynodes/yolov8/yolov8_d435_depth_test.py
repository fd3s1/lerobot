from realsense_camera import *
import cv2
import argparse
import time
from ultralytics import YOLO
import numpy as np
import supervision as sv

rs = RealsenseCamera()

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
    args = parse_arguments()
    frame_width, frame_height = args.webcam_resolution

    model = YOLO("yolov8n.pt")

    box_annotator = sv.BoxAnnotator(
        thickness=2,
        text_thickness=2,
        text_scale=1
    )

    while True:
        ret, frame, depth_frame = rs.get_frame_stream()
        t_start = time.time()  # 开始计时
        result = model(frame, agnostic_nms=True,conf=0.5)[0]
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
            scene=frame,
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
        for i in range(len(coord)):
            x1 = coord[i][0]
            x2 = coord[i][2]
            y1 = coord[i][1]
            y2 = coord[i][3]
            center_x = (x1+x2)/2
            center_y = (y1+y2)/2
            width = x2-x1
            height = y2-y1
            print("Center coordinate is",(center_x,center_y))

        t_end = time.time()  # 结束计时\
        # 添加fps显示
        fps = int(1.0 / (t_end - t_start))  
        # cv2.putText(frame, "x: " + str(round(point_3d[0], 3)), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        # cv2.putText(frame, "y: " + str(round(point_3d[1], 3)), (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        # cv2.putText(frame, "z: " + str(round(point_3d[2], 3)), (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(frame, text="FPS: {}".format(fps), org=(50, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=2,lineType=cv2.LINE_AA, color=(0, 255, 0))
        
        # depth_frame = np.array(depth_frame)
        # depth_frame2 = aligned_depth_frame.copy()
        
        center_x = int(center_x)
        center_y = int(center_y)
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)
        width = int(width)
        height = int(height)
        
        sum_values = 0
        
        
        for i in range(x1,x2+1):
            for j in range(y1,y2+1):
                sum_values += depth_frame[[j],[i]]
                depth_frame[[j],[i]]=1000
        print(y1,y2)
         
        depth_mm = sum_values / (width*height)
        print("Depth in the center is", depth_mm/10,"cm")
        
        
        #cv2.imshow("depth2",depth_frame2)
        #cv2.imshow("depth",depth_frame)
        cv2.imshow("color", frame)
        

        if (cv2.waitKey(30) == 27):
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    main()

rs.release()
