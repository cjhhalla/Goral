from ultralytics import YOLO
import cv2

# 1) 모델 로드 (원하면 yolov8s.pt, yolov8m.pt 등으로 변경)
model = YOLO("../config/license_plate_detector.pt")

# 2) 이미지 로드
img_path = "test1.png"
img = cv2.imread(img_path)

# 3) 추론
results = model(img)

# 4) 결과 오버레이
for r in results:
    boxes = r.boxes  # Boxes 객체
    for box in boxes:
        # bbox 좌표 (xyxy)
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        # confidence
        conf = float(box.conf[0])

        # class id 및 이름
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]

        label = f"{cls_name} {conf:.2f}"

        # bbox 그리기
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 텍스트 배경
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(
            img, (x1, y1 - th - 4), (x1 + tw, y1), (0, 255, 0), -1
        )

        # 텍스트
        cv2.putText(
            img,
            label,
            (x1, y1 - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

# 5) 결과 출력
cv2.imshow("YOLOv8 Detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
