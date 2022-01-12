#-*- coding:UTF-8 -*-
import cv2
import os
# 调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
cap = cv2.VideoCapture(0)
face_id = input('\n enter user id:')
print(face_id)
face_detector = cv2.CascadeClassifier('/usr/local/lib/python3.7/dist-packages/cv2/data/haarcascade_frontalface_default.xml')
count = 0
while(cap.isOpened()):#USB摄像头工作时,读取一帧图像
    ret, img = cap.read()#显示图像窗口在树莓派的屏幕上
    # 转为灰度图片
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 检测人脸
    faces = face_detector.detectMultiScale(gray, 1.3, 5)
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+w), (255, 0, 0))
        count += 1
        print(count)
        # 保存图像
        cv2.imwrite("Facedata/User." + str(face_id) + '.' + str(count) + '.jpg', gray[y: y + h, x: x + w])
        cv2.imshow('image', img)
    #按下esc键退出
    k = cv2.waitKey(1)
    if k == 27:   # 通过esc键退出摄像
        break
    elif count >= 800:  # 得到1000个样本后退出摄像
        break
# 关闭摄像头
cap.release()
cv2.destroyAllWindows()