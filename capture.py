#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import time
import boto3
from botocore.client import Config
from dotenv import dotenv_values


# CLASS = ['fruits']
fruits = 'images/fruits'

try:
    os.makedirs(fruits)

except FileExistsError:
    print('Directories not created becasue they already exist')

image_count = len(os.listdir(fruits))

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 너비 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 높이 설정

config = dotenv_values('.env')
s3 = boto3.client(
    's3',
    aws_access_key_id=config['AWS_ACCESS_KEY'],
    aws_secret_access_key=config['AWS_SECRET_KEY'],
    config=Config(signature_version='s3v4')
)

while True:
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError('Could not read image from camera.')
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):

        filename = f"{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        image_path = os.path.join(fruits, filename)
        cv2.imwrite(image_path, frame)
        print(f"Image Saved: {image_path}")
        image_count += 1

        with open(image_path, 'rb') as data:#내가 올리고 싶은 파일 이름
            s3.upload_fileobj(data, config['AWS_BUCKET_NAME'], filename)#올리고 나서 웹에서 이름

    elif key == ord('q'):
        print("Exiting")
        break
cap.release()
cv2.destroyAllWindows()