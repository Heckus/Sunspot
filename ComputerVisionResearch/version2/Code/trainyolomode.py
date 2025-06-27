from ultralytics import YOLO

# Load a model
model = YOLO("yolov8m.pt")  # load a pretrained model. change this to the model you want to use, n, s, l etc. (only ending)

# Train the model
results = model.train(data="./yaml/mytrainingset.yaml", imgsz=800, batch=8, epochs=50, plots=False)
