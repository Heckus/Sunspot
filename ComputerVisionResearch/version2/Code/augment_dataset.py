# augment_dataset.py
"""
python augment_dataset.py \
    --dataset ../ultimate_dataset/data.yaml \
    --output ./final_augmented_dataset \
    --multiplier 2 \
    --rotate \
    --brightness

python augment_dataset.py \
    --dataset ../ultimate_dataset/data.yaml \
    --output ./final_augmented_dataset_v2 \
    --multiplier 1 \
    --hflip \
    --noise \
    --blur
"""
# augment_dataset.py

import os
import cv2
import yaml
import shutil
import argparse
import random
import numpy as np
import albumentations as A
from tqdm import tqdm

def load_yolo_annotations(label_path):
    """Loads YOLO annotations from a text file."""
    bboxes = []
    class_labels = []
    if os.path.exists(label_path):
        with open(label_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    class_id = int(parts[0])
                    x_center, y_center, width, height = map(float, parts[1:])
                    bboxes.append([x_center, y_center, width, height])
                    class_labels.append(class_id)
    return bboxes, class_labels

def save_yolo_annotations(output_path, bboxes, class_labels):
    """Saves YOLO annotations to a text file."""
    with open(output_path, 'w') as f:
        for bbox, class_id in zip(bboxes, class_labels):
            x_center, y_center, width, height = bbox
            f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

def main(args):
    # --- 1. Load the dataset config ---
    try:
        with open(args.dataset, 'r') as f:
            data_config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error: Could not read or parse the YAML file at {args.dataset}. {e}")
        return

    # Get absolute path from the YAML file's location
    base_dir = os.path.dirname(os.path.abspath(args.dataset))
    train_path = os.path.join(base_dir, data_config['train'])
    
    val_path = os.path.join(base_dir, data_config.get('val', ''))
    test_path = os.path.join(base_dir, data_config.get('test', ''))

    # --- 2. Create Output Directories ---
    output_base_dir = args.output
    os.makedirs(os.path.join(output_base_dir, 'train', 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_base_dir, 'train', 'labels'), exist_ok=True)
    
    if os.path.exists(val_path):
        os.makedirs(os.path.join(output_base_dir, 'valid', 'images'), exist_ok=True)
        os.makedirs(os.path.join(output_base_dir, 'valid', 'labels'), exist_ok=True)
    if os.path.exists(test_path):
        os.makedirs(os.path.join(output_base_dir, 'test', 'images'), exist_ok=True)
        os.makedirs(os.path.join(output_base_dir, 'test', 'labels'), exist_ok=True)

    print(f"Output directory created at: {output_base_dir}")

    # --- 3. Define Augmentation Pipeline ---
    transformations = []
    if args.hflip:
        transformations.append(A.HorizontalFlip(p=0.5))
    if args.rotate:
        transformations.append(A.Rotate(limit=args.rotate_limit, p=0.5, border_mode=cv2.BORDER_CONSTANT))
    if args.brightness:
        transformations.append(A.RandomBrightnessContrast(brightness_limit=args.brightness_limit, contrast_limit=0.2, p=0.5))
    if args.noise:
        # FIX 1: Changed `var_limit` to `variance_limit`
        transformations.append(A.GaussNoise(variance_limit=(10.0, 50.0), p=0.5))
    if args.blur:
        transformations.append(A.MotionBlur(blur_limit=7, p=0.5))

    if not transformations:
        print("Warning: No augmentations selected. The script will only copy the training data.")

    # FIX 2: Added `min_visibility` to handle boxes that go partially off-screen
    transform = A.Compose(
        transformations,
        bbox_params=A.BboxParams(format='yolo', label_fields=['class_labels'], min_visibility=0.25)
    )

    # --- 4. Process Training Data ---
    print("\n--- Augmenting Training Data ---")
    train_image_dir = train_path
    train_label_dir = train_path.replace('images', 'labels')

    image_files = [f for f in os.listdir(train_image_dir) if f.endswith(('.jpg', '.jpeg', '.png'))]

    for image_name in tqdm(image_files, desc="Augmenting train images"):
        image_path = os.path.join(train_image_dir, image_name)
        label_name = os.path.splitext(image_name)[0] + '.txt'
        label_path = os.path.join(train_label_dir, label_name)
        
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        bboxes, class_labels = load_yolo_annotations(label_path)

        # First, copy the original image and label
        shutil.copy(image_path, os.path.join(output_base_dir, 'train', 'images', image_name))
        if os.path.exists(label_path):
            shutil.copy(label_path, os.path.join(output_base_dir, 'train', 'labels', label_name))

        # Apply augmentations N times
        for i in range(args.multiplier):
            if not transformations: continue

            try:
                transformed = transform(image=image, bboxes=bboxes, class_labels=class_labels)
                
                transformed_image = transformed['image']
                transformed_bboxes = transformed['bboxes']
                transformed_class_labels = transformed['class_labels']

                if transformed_bboxes:
                    new_image_name = f"{os.path.splitext(image_name)[0]}_aug_{i}.jpg"
                    new_label_name = f"{os.path.splitext(image_name)[0]}_aug_{i}.txt"
                    
                    output_image_path = os.path.join(output_base_dir, 'train', 'images', new_image_name)
                    output_label_path = os.path.join(output_base_dir, 'train', 'labels', new_label_name)

                    cv2.imwrite(output_image_path, cv2.cvtColor(transformed_image, cv2.COLOR_RGB2BGR))
                    save_yolo_annotations(output_label_path, transformed_bboxes, transformed_class_labels)
            except Exception as e:
                print(f"\nCould not transform {image_name}. Skipping. Reason: {e}")


    # --- 5. Copy Validation and Test Data (No Augmentation) ---
    def copy_split(src_path, dest_name):
        if not os.path.exists(src_path): return
        print(f"\n--- Copying {dest_name} Data (No Augmentation) ---")
        dest_dir = os.path.join(output_base_dir, dest_name)
        src_image_dir = src_path
        src_label_dir = src_path.replace('images', 'labels')
        
        # Ensure destination subdirectories exist before copying
        os.makedirs(os.path.join(dest_dir, 'images'), exist_ok=True)
        os.makedirs(os.path.join(dest_dir, 'labels'), exist_ok=True)
        
        shutil.copytree(src_image_dir, os.path.join(dest_dir, 'images'), dirs_exist_ok=True)
        shutil.copytree(src_label_dir, os.path.join(dest_dir, 'labels'), dirs_exist_ok=True)
        print(f"Successfully copied {dest_name} data.")

    copy_split(val_path, 'valid')
    copy_split(test_path, 'test')
    
    # --- 6. Create New YAML File ---
    new_yaml_path = os.path.join(output_base_dir, 'augmented_data.yaml')
    data_config['path'] = os.path.abspath(output_base_dir)
    
    data_config['train'] = os.path.join('train', 'images')
    if data_config.get('val'):
        data_config['val'] = os.path.join('valid', 'images')
    if data_config.get('test'):
        data_config['test'] = os.path.join('test', 'images')
        
    with open(new_yaml_path, 'w') as f:
        yaml.dump(data_config, f, sort_keys=False)

    print(f"\n--- Augmentation Complete! ---")
    print(f"New dataset created at: {output_base_dir}")
    print(f"New YAML file created at: {new_yaml_path}")
    print("You can now use this new YAML file to train your model.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Augment a YOLOv8 dataset with various transformations.")
    parser.add_argument('--dataset', type=str, required=True, help='Path to the input dataset YAML file (e.g., data.yaml).')
    parser.add_argument('--output', type=str, required=True, help='Path to the output directory where the augmented dataset will be saved.')
    parser.add_argument('--multiplier', type=int, default=1, help='Number of augmented versions to create for each training image.')
    
    parser.add_argument('--hflip', action='store_true', help='Apply horizontal flip.')
    parser.add_argument('--rotate', action='store_true', help='Apply rotation.')
    parser.add_argument('--brightness', action='store_true', help='Apply brightness and contrast changes.')
    parser.add_argument('--noise', action='store_true', help='Apply Gaussian noise.')
    parser.add_argument('--blur', action='store_true', help='Apply motion blur.')
    
    parser.add_argument('--rotate-limit', type=int, default=15, help='Rotation limit in degrees.')
    parser.add_argument('--brightness-limit', type=float, default=0.2, help='Brightness change limit.')

    args = parser.parse_args()
    main(args)
