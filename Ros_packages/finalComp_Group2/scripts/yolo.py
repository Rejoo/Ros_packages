import torch
from ultralytics import YOLO

model = YOLO("/home/areej/catkin_ws/src/mia_competition/models/best (4).pt")
img = "/home/areej/catkin_ws/src/mia_competition/models/20240917_154906.jpg"

results = model(img)

for result in results:
    # Print bounding boxes, class IDs, and class names
    boxes = result.boxes.xyxy.numpy()  # Convert to numpy array
    class_ids = result.boxes.cls.numpy()  # Convert to numpy array
    names = result.names

    print("Boxes:", boxes)
    print("Class IDs:", class_ids) 
    print("Names:", [names[int(cls_id)] for cls_id in class_ids])