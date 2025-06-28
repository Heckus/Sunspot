from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.pt")  # load a pretrained model. change this to the model you want to use, n, s, l etc. (only ending)

# Train the model
#results = model.train(data="/home/hecke/Documents/Datasets/Volleyball All-in-one.v1i.yolov8/data.yaml", imgsz=640, batch=8, epochs=50, plots=False)
results = model.train(data="/home/hecke/GitHubRepos/Sunspot/ComputerVisionResearch/version2/Code/final_augmented_dataset/data.yaml", imgsz=640, batch=8, epochs=50, plots=False)
