# MIT License
#
# Copyright (c) [2024] [Chen Zuo]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import pyrealsense2 as rs
import numpy as np
import cv2
import datetime
import time
# import xlsxwriter
import math
import serial
from LED_light_control import LightController
from math import pi
import json

XCenter_Sc = 0
YCenter_Sc = 0
row = 0

Xleftbottom = 0
Yleftbottom = 0
Xrighttop = 0
Yrighttop = 0
Xrightbottom = 0
Yrightbottom = 0

# workbook = xlsxwriter.Workbook('mxj_table.xlsx')
# worksheet = workbook.add_worksheet()

pipeline = rs.pipeline()

# Camera Init
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
time.sleep(3)
depth_sensor = profile.get_device().first_depth_sensor()  # Initializaiton
# depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Depth Scale is: ", depth_scale)

clipping_distance_in_meters = 0.7  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale  # TODO

align_to = rs.stream.color
align = rs.align(align_to)

    # 背景虚化 | Background blur
    # es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 4))
    # kernel = np.ones((5, 5), np.uint8)
# background = None
background_1 = None

light_is_on = True

t = 1
s = 1
vector2 = [0, 0, 0]

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧 | Wait to get the image frame
    aligned_frames = align.process(frames)  # 获取对齐帧 | Get the aligned frame
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧 | Get the depth frame in the aligned frame
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧 | Get the color frame in the aligned frame

    # get the intrinsics of the camera
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参 Acquiring the intrinsics of the camera
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到） | Acquiring the depth parameters (pixel coordinate system to camera coordinate system)
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }
    # 保存内参到本地 | Save the intrinsics to the local
    with open('./intr7insics.json', 'w') as fp:
        json.dump(camera_parameters, fp)
    #######################################################

    depth_image_1 = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位） | Depth image (default 16-bit)
    depth_image_8bit = cv2.convertScaleAbs(depth_image_1, alpha=0.03)  # 深度图（8位） | Depth image (8-bit)
    depth_image_3d = np.dstack((depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图 | 3-channel depth image
    # color_image = np.asanyarray(color_frame.get_data())  # RGB图 | RGB image
    color_image_1 = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧 | Return camera intrinsics, depth parameters, color image, depth image, depth frame in the aligned frame
    return intr, depth_intrin, color_image_1, depth_image_1, color_frame, aligned_depth_frame, depth_image_3d


try:
    while True:
        print("Start time:" + str(time.time() * 1000))

        ###################### 获取画面中心点的真实三维坐标 Acquiring the realworld 3D coordinates of the frame center
        # TODO 当物体进入到画面中心，中心坐标的深度信息会因为被物体遮挡发生变化需要注意这个问题
        #  When object enter the center of the frame, the depth data will change because camera will measure
        #  the object rather than environment
        intr, depth_intrin, rgb_1, depth, color_frame, aligned_depth_frame, depth_image_3d = get_aligned_images()
        # 获取对齐的图像与相机内参 | Get aligned images and camera intrinsics
        print("============")
        print(aligned_depth_frame)
        x_o = 320
        y_o = 240
        x_test = 160
        y_test = 120
        x_test_1 = 480
        y_test_1 = 360
        dis_o = aligned_depth_frame.get_distance(x_o, y_o)  # （x, y)点的真实深度

        dis_test = aligned_depth_frame.get_distance(x_test, y_test)
        dis_test_1 = aligned_depth_frame.get_distance(x_test_1, y_test_1)

        camera_coordinate_o = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_o, y_o], dis_o)

        camera_coordinate_test = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_test, y_test], dis_test)

        camera_coordinate_test_1 = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_test_1, y_test_1], dis_test)

        print("===", dis_o)

        # （x, y)点在相机坐标系下的真实值，为一个三维向量。其中camera_coordinate[2]仍为dis，camera_coordinate[0]和camera_coordinate[1]为相机坐标系下的xy真实距离。
        # The real value of the (x, y) point in the camera coordinate system is a three-dimensional vector.
        # Among them, camera_coordinate[2] is still dis, and camera_coordinate[0] and camera_coordinate[1] are the real xy distances in the camera coordinate system.

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue
        # TODO modification starts from here, notice every depth_image
        # depth_image = np.asanyarray(aligned_depth_frame.get_data())
        # color_image = np.asanyarray(color_frame.get_data())
        depth_image_1 = np.asanyarray(aligned_depth_frame.get_data())
        color_image_1 = np.asanyarray(color_frame.get_data())
        # a = 110  # 0 <= a <= 480
        # b = 350  # 0 <= b <= 480, a < b
        # c = 160  # 0 <= c <= 640
        # d = 480  # 0 <= d <= 640, c < d, change the size of the area
        # crop_img = color_image_1[a:b, c:d]  # 抠出来彩色画面的部分图像 | Crop out part of the color image

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 0
        depth_image_3d = np.dstack((depth_image_1, depth_image_1, depth_image_1))  # depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color,
        #                       color_image)  # 设置黑色背景 | Set black background
        bg_removed_1 = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color,
                                color_image_1)  # 设置黑色背景 | Set black background
        # bg_removed_img = bg_removed[a:b, c:d]
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.03), cv2.COLORMAP_JET)

        # gray_lwpCV = cv2.cvtColor(bg_removed_img, cv2.COLOR_BGR2GRAY)
        # gray_lwpCV = cv2.GaussianBlur(gray_lwpCV, (21, 21), 3, 3)
            # gray_lwpCV_norm = cv2.convertScaleAbs(grey_color, alpha=0.5, beta=0)

        gray_lwpCV_1 = cv2.cvtColor(bg_removed_1, cv2.COLOR_BGR2GRAY)
        gray_lwpCV_1 = cv2.GaussianBlur(gray_lwpCV_1, (21, 21), 3, 3)

            # gray_lwpCV_1_norm = cv2.convertScaleAbs(gray_lwpCV_1, alpha=0.5, beta=0)

        # crop_depth_img = depth_colormap[a:b, c:d]

        text_2 = "Distance of the bg:" + str(clipping_distance_in_meters)

        # cv2.putText(bg_removed_img, text_2, (0, b - a), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        cv2.putText(bg_removed_1, text_2, (0, 450), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)

        # 点的位置 | The position of the point
        cv2.circle(color_image_1, (320, 240), 8, [255, 0, 0], thickness=-1)
        # 深度从img_depth[x, y]中获得 | The depth is obtained from img_depth[x, y]
        cv2.putText(color_image_1, "Dis:" + str(dis_o) + " m", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 255])
        cv2.putText(color_image_1, "X:" + str(camera_coordinate_o[0]) + " m", (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])
        cv2.putText(color_image_1, "Y:" + str(camera_coordinate_o[1]) + " m", (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])
        cv2.putText(color_image_1, "Z:" + str(camera_coordinate_o[2]) + " m", (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])

        # 点的位置 测试点 | Test point
        cv2.circle(color_image_1, (160, 120), 8, [255, 0, 0], thickness=-1)
        # 深度从img_depth[x, y]中获得 |   The depth is obtained from img_depth[x, y]
        cv2.putText(color_image_1, "Dis:" + str(dis_test) + " m", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 255])
        cv2.putText(color_image_1, "X:" + str(camera_coordinate_test[0]) + " m", (0, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])
        cv2.putText(color_image_1, "Y:" + str(camera_coordinate_test[1]) + " m", (0, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])
        cv2.putText(color_image_1, "Z:" + str(camera_coordinate_test[2]) + " m", (0, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])

        # 点的位置 测试点2 | Test point 2
        cv2.circle(color_image_1, (480, 360), 8, [255, 0, 0], thickness=-1)
        # 深度从img_depth[x, y]中获得 | The depth is obtained from img_depth[x, y]
        cv2.putText(color_image_1, "Dis:" + str(dis_test_1) + " m", (0, 270), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 255])
        cv2.putText(color_image_1, "X:" + str(camera_coordinate_test_1[0]) + " m", (0, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])
        cv2.putText(color_image_1, "Y:" + str(camera_coordinate_test_1[1]) + " m", (0, 330), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])
        cv2.putText(color_image_1, "Z:" + str(camera_coordinate_test_1[2]) + " m", (0, 360), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 0, 0])

        # if background is None:  # TODO
        #     background = gray_lwpCV
        #     continue
        #
        # diff = cv2.absdiff(background, gray_lwpCV)  # TODO
        # diff = cv2.convertScaleAbs(diff, alpha=0.5, beta=0)
        # diff_norm = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY)[1]
        # resized_diff = cv2.resize(diff, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
        # contours, hierarchy = cv2.findContours(resized_diff.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if background_1 is None:
            background_1 = gray_lwpCV_1
            continue
        diff_1 = cv2.absdiff(background_1, gray_lwpCV_1)
        diff_1 = cv2.convertScaleAbs(diff_1, alpha=0.3, beta=0)
        diff_1_norm = cv2.threshold(diff_1, 0, 255, cv2.THRESH_BINARY)[1]
        resized_diff_1 = cv2.resize(diff_1, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
        contours_1, hierarchy_1 = cv2.findContours(resized_diff_1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("Stage 1 time:" + str(time.time() * 1000))

        light_is_on = True

        # 对于矩形区域，只显示大于给定阈值的轮廓，所以一些微小的变化不会显示。对于光照不变和噪声低的摄像头可不设定轮廓最小尺寸的阈值
        # For rectangular areas, only contours larger than the given threshold are displayed, so some small changes are not displayed.
        # For cameras with constant illumination and low noise, the threshold for the minimum size of the outline may not be set
        # for c in contours:
        #     # light_is_on = False
        #     if cv2.contourArea(c) < 100:
        #         continue
        #     (x, y, w, h) = cv2.boundingRect(c)  # This function calculates the bounding box of a rectangle
        #     cv2.rectangle(bg_removed_img, (x, y), (x + w, y + h), (0, 255, 0), 1)
        #     XCenter_Sc = x + w / 2  # TODO XCenter_Sc and YCenter_Sc. These two parameters can be optimized for taking average point
        #     YCenter_Sc = y + h / 2


        for j in contours_1:
            if cv2.contourArea(j) < 100:  # For rectangular areas, only contours larger than the given threshold are
                # displayed, so some small changes are not displayed. For cameras with constant
                # illumination and low noise, the threshold for the minimum size of the outline may not be set
                continue
            (x_1, y_1, w_1, h_1) = cv2.boundingRect(j)
            cv2.rectangle(bg_removed_1, (x_1, y_1), (x_1 + w_1, y_1 + h_1), (0, 255, 0), 1)
            # 第一个参数：bg_removed_1是原图； | The first parameter: bg_removed_1 is the original image;
            # 第二个参数：（x，y）是矩阵的左上点坐标； | The first parameter: bg_removed_1 is the original image;
            # 第三个参数：（x+w，y+h）是矩阵的右下点坐标；| The first parameter: bg_removed_1 is the original image;
            # 第四个参数：（0,255,0）是画线对应的rgb颜色； | The first parameter: bg_removed_1 is the original image;
            # 第五个参数：所画的线的宽度 | The first parameter: bg_removed_1 is the original image;
            XCenter_Sc_1 = x_1 + w_1 / 2
            YCenter_Sc_1 = y_1 + h_1 / 2  # 物体识别框的中心点像素坐标 |  The pixel coordinates of the center point of the object recognition box
            Xdetleft = x_1 + w_1 # 从左侧进入画面的物体的右边界的横坐标 | The horizontal coordinate of the right boundary of the object entering the screen from the left side
            # Xdetright = x_1 - w_1 # 从右侧进入画面的物体的左边界

            Xleftbottom = x_1
            Yleftbottom = y_1 + h_1
            Xrighttop = x_1 + w_1 * 0.75
            Yrighttop = y_1
            Xrightbottom = x_1 + w_1
            Yrightbottom = y_1 + h_1


            # TODO 延时测试代码 latency test
            # print("end:" + str(time.time() * 1000))

            # 求出物体中心点的现实世界三维坐标 | Calculate the real-world three-dimensional coordinates of the object center
            dis_obj = aligned_depth_frame.get_distance(round(XCenter_Sc_1), round(YCenter_Sc_1))  # 物体识别框中心点的坐标
            camera_coordinate_obj = rs.rs2_deproject_pixel_to_point(depth_intrin, [XCenter_Sc_1, YCenter_Sc_1], dis_obj)

            # 将物体中心深度距离输出到检测画面中 | Output the depth distance of the object center to the detection screen
            text_dis = "Object's center distance:" + str(dis_obj)
            cv2.putText(bg_removed_1, text_dis, (0, 415), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)
            cv2.putText(bg_removed_1, str(dis_obj)[:4] + 'm', (x_1, y_1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # # 求出物体中心的三维坐标与画面中心的坐标的欧几里得距离  | Calculate the Euclidean distance between the three-dimensional
            # d_obj_to_center = math.sqrt((camera_coordinate_obj[0]-0)*(camera_coordinate_obj[0]-0)
            #                        + (camera_coordinate_obj[1]-0)*(camera_coordinate_obj[1]-0))
            # print('Obj Center to O: ', d_obj_to_center)
            # text_d_obj_to_center = "Obj Center to O:" + str(d_obj_to_center)
            # cv2.putText(bg_removed_1, text_d_obj_to_center, (0, 430), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)

            #  目前只能单个物体。 | Currently only one object is supported.
            if 640 >= x_1 >= 320:
                # 求物体框左侧中心点位的现实世界三维坐标 | Calculate the real-world three-dimensional coordinates of the center point on the left side of the object box
                dis_obj_left = aligned_depth_frame.get_distance(round(x_1 + w_1 * 0.25), round(YCenter_Sc_1))  # 物体识别框左侧边中心点的坐标
                camera_coordinate_obj_left = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_1 + w_1 * 0.25, YCenter_Sc_1],
                                                                             dis_obj_left)
                camera_coordinate_cal = camera_coordinate_obj_left
                x_cal = round(x_1 + w_1 * 0.25)

            if 0 <= Xrighttop < 320:
                # 求物体框右侧侧中心点位的现实世界三维坐标 | Calculate the real-world three-dimensional coordinates of the center point on the right side of the object box
                dis_obj_right = aligned_depth_frame.get_distance(round(Xrighttop - w_1 * 0.25),
                                                                 round(YCenter_Sc_1))  # 物体识别框右侧边中心点的坐标
                camera_coordinate_obj_right = rs.rs2_deproject_pixel_to_point(depth_intrin, [Xrighttop, YCenter_Sc_1],
                                                                              dis_obj_right)
                camera_coordinate_cal = camera_coordinate_obj_right
                x_cal = round(Xrighttop - w_1 * 0.25)


            # 求出物体中心的三维坐标与画面中心的坐标的欧几里得距离 | Calculate the Euclidean distance between the three-dimensional
            d_box_to_center = math.sqrt((camera_coordinate_cal[0]-0)*(camera_coordinate_cal[0]-0)
                                   + (camera_coordinate_cal[1]-0)*(camera_coordinate_cal[1]-0))
            print('Obj to center: ', d_box_to_center)
            text_d_box_to_center = "Obj to center:" + str(d_box_to_center)
            cv2.putText(bg_removed_1, text_d_box_to_center, (0, 430), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)

            cv2.line(bg_removed_1, (x_cal, 0), (x_cal, 480), (0, 0, 255), 1)

            # 安全距离判定，开关灯 | Safety Distance judgment, turn on/off the light
            if d_box_to_center <= 0.20 or w_1 >= 320:
                light_is_on = False
            else:
                light_is_on = True

        if light_is_on:
            LightController.setLightOn()
            cv2.putText(bg_removed_1, "Laser ON", (0, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)
            # print("d:" + str(time.time() * 1000))
        else:
            LightController.setLightOff()
            cv2.putText(bg_removed_1, "Laser OFF", (0, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 2)
            # print("e:" + str(time.time() * 1000))
            '''

            if s == 2:
                end = time.time() * 1000
                print(end - start)
                start_1 = time.time() * 1000
                s = s + 1
            '''

        # images = np.hstack((bg_removed_img, crop_img))
        images_1 = np.hstack((bg_removed_1, color_image_1))
        images_2 = np.hstack((background_1, gray_lwpCV_1))
            # cv2.namedWindow('Test', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Test_1', cv2.WINDOW_AUTOSIZE)
            # cv2.namedWindow('background $ current image', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('background $ current image', images_2)
        # cv2.imshow('Test', images)
        cv2.imshow('Test_1', images_1)
            # cv2.imshow('absdiff',diff_1)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            # workbook.close()
            # serial.close()
            break

finally:
    # serial.close()
    pipeline.stop()