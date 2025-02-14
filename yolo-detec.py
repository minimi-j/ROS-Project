import cv2
from ultralytics import YOLO

cap = cv2.VideoCapture(0)

model = YOLO('model/fruit_six.pt')

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break
    
    results = model(frame)
    result = results[0]
    for box in result.boxes:  
        x1, y1, x2, y2 = map(int, box.xyxy[0])  
        confidence = box.conf[0].item()  
        class_id = int(box.cls[0].item())  
        class_name = result.names[class_id]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 클래스 이름과 confidence 표시
        label = f"{class_name} {confidence:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Detection Result", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
