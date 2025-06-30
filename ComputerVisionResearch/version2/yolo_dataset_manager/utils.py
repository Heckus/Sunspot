import os
import yaml
import cv2
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import torch
from ultralytics import YOLO
import pandas as pd
import json

# --- Config and File Path Functions ---

def load_config(yaml_path):
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

def get_image_files(base_path, relative_path):
    if not isinstance(relative_path, str): return []
    full_path = os.path.join(base_path, relative_path)
    if not os.path.isdir(full_path): return []
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif']
    return sorted([os.path.join(full_path, f) for f in os.listdir(full_path) if os.path.splitext(f)[1].lower() in image_extensions])

def get_label_path(image_path):
    base_dir = os.path.dirname(image_path)
    img_name = os.path.basename(image_path)
    label_name = f"{os.path.splitext(img_name)[0]}.txt"
    
    # Standard YOLO structure: .../images/train -> .../labels/train
    label_path = os.path.join(base_dir.replace('images', 'labels', 1), label_name)
    if os.path.exists(os.path.dirname(label_path)):
        return label_path
    
    # Alternative structure: .../dataset/train/images -> .../dataset/train/labels
    label_path = os.path.join(os.path.dirname(base_dir), 'labels', os.path.basename(base_dir), label_name)
    if os.path.exists(os.path.dirname(label_path)):
        return label_path
        
    # Fallback: labels in same folder as images
    return os.path.join(base_dir, label_name)

# --- Annotation and Drawing Functions ---

def parse_yolo_labels(label_path):
    if not os.path.exists(label_path): return []
    labels = []
    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 5:
                labels.append(tuple(map(float, parts)))
    return labels

def yolo_to_abs(yolo_coords, img_width, img_height):
    class_id, x_center, y_center, width, height = yolo_coords
    abs_width = width * img_width
    abs_height = height * img_height
    x_min = (x_center * img_width) - (abs_width / 2)
    y_min = (y_center * img_height) - (abs_height / 2)
    x_max = x_min + abs_width
    y_max = y_min + abs_height
    return int(class_id), x_min, y_min, x_max, y_max

def abs_to_yolo(abs_coords, img_width, img_height):
    class_id, x_min, y_min, x_max, y_max = abs_coords
    x_center = ((x_min + x_max) / 2) / img_width
    y_center = ((y_min + y_max) / 2) / img_height
    width = (x_max - x_min) / img_width
    height = (y_max - y_min) / img_height
    return int(class_id), x_center, y_center, width, height

def draw_bounding_boxes_on_image(image, labels, class_names, colors):
    draw = ImageDraw.Draw(image)
    img_width, img_height = image.size
    try:
        font = ImageFont.truetype("arial.ttf", 15)
    except IOError:
        font = ImageFont.load_default()

    for label in labels:
        class_id, x_min, y_min, x_max, y_max = yolo_to_abs(label, img_width, img_height)
        class_name = class_names.get(class_id, "Unknown")
        color = colors[class_id % len(colors)]
        
        draw.rectangle([x_min, y_min, x_max, y_max], outline=color, width=2)
        text_bbox = draw.textbbox((x_min, y_min), class_name, font=font)
        draw.rectangle(text_bbox, fill=color)
        draw.text((x_min, y_min), class_name, fill="white", font=font)
    return image

# --- Dataset Management Functions ---

def flag_image(image_path, flag_file="flagged_images.txt"):
    with open(flag_file, "a") as f:
        f.write(f"{image_path}\n")

def delete_image_and_label(image_path):
    label_path = get_label_path(image_path)
    try:
        if os.path.exists(image_path): os.remove(image_path)
        if os.path.exists(label_path): os.remove(label_path)
        return True, f"Deleted {os.path.basename(image_path)} and its label."
    except Exception as e:
        return False, f"Error deleting files: {e}"

def save_annotations_to_file(label_path, annotations, img_width, img_height, class_names_map):
    with open(label_path, "w") as f:
        for ann in annotations:
            class_id_str = ann['label'].split(" ")[0]
            if class_id_str.isdigit():
                class_id = int(class_id_str)
            else: # If label is class name, convert to ID
                class_id = next((k for k, v in class_names_map.items() if v == class_id_str), -1)

            if class_id == -1: continue

            x_min, y_min = ann['left'], ann['top']
            x_max, y_max = ann['left'] + ann['width'], ann['top'] + ann['height']
            
            _, x_center, y_center, width, height = abs_to_yolo((class_id, x_min, y_min, x_max, y_max), img_width, img_height)
            f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

# --- Model Inference Functions ---

def find_pt_files(directory):
    if not os.path.isdir(directory): return []
    return [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.pt')]

@torch.no_grad()
def run_inference(model_path, image_path, conf_threshold=0.25):
    # This function would need to be updated to use the new drawing function if needed
    model = YOLO(model_path)
    results = model(image_path, conf=conf_threshold, verbose=False)
    result = results[0]
    img = Image.fromarray(result.plot()) # .plot() is a handy way to get the annotated image
    return img