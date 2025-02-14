# import torch
import cv2
import numpy as np
from urllib.request import urlopen
from ultralytics import YOLO

ip = "192.168.137.131"
stream = urlopen("http://" + ip + ":8080/stream?topic=/csi_cam_0/image_raw")
buffer = b''

model = YOLO('model/best.pt')
print("모델 불러오기 끝. 프로그램 시작!")

while True :
    buffer += stream.read(4096)
    head = buffer.find(b'\xff\xd8')
    end = buffer.find(b'\xff\xd9')

    try :
        if head > -1 and end > -1 :
            jpg = buffer[head:end+2]
            buffer = buffer[end+2:]
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            results = model(img)
            result = results[0]
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  
                confidence = box.conf[0].item()  
                class_id = int(box.cls[0].item())  
                class_name = result.names[class_id]

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                label = f"{class_name} {confidence:.2f}"
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Detection Result", img)
            key = cv2.waitKey(1)
            if key == ord('q') :
                break
    
    except :
        pass

cv2.destroyAllWindows()