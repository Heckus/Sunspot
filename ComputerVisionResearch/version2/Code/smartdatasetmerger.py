# merge_datasets.py


"""
python merge_datasets.py \
    --dataset1 ../ball_dataset/data.yaml \
    --dataset2 ../actions_dataset/data.yaml \
    --output ./final_merged_dataset
"""

import os
import yaml
import shutil
from tqdm import tqdm
import argparse

def get_full_path(yaml_path, relative_path):
    """Constructs an absolute path from a YAML file's location and a relative path within it."""
    base_dir = os.path.dirname(os.path.abspath(yaml_path))
    return os.path.join(base_dir, relative_path)

def process_and_copy_files(split_path, dest_dir, class_map, dataset_prefix):
    """
    Copies images and re-indexes label files for a given data split (train/valid).

    Args:
        split_path (str): The source path for the images in the split.
        dest_dir (str): The destination directory for the split (e.g., './merged_dataset/train').
        class_map (dict): A mapping from old class indices to new ones.
        dataset_prefix (str): A unique prefix to add to filenames to avoid overwrites.
    """
    if not os.path.exists(split_path):
        print(f"Warning: Source path not found, skipping: {split_path}")
        return 0

    image_dir = split_path
    label_dir = split_path.replace('images', 'labels')

    dest_image_dir = os.path.join(dest_dir, 'images')
    dest_label_dir = os.path.join(dest_dir, 'labels')
    os.makedirs(dest_image_dir, exist_ok=True)
    os.makedirs(dest_label_dir, exist_ok=True)

    image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.jpeg', '.png'))]
    copied_count = 0

    for image_name in tqdm(image_files, desc=f"Processing {os.path.basename(dest_dir)} from {dataset_prefix}"):
        source_image_path = os.path.join(image_dir, image_name)
        label_name = os.path.splitext(image_name)[0] + '.txt'
        source_label_path = os.path.join(label_dir, label_name)

        if not os.path.exists(source_label_path):
            continue

        # Create a unique name to prevent overwrites
        new_image_name = f"{dataset_prefix}_{image_name}"
        new_label_name = f"{dataset_prefix}_{label_name}"

        # Copy the image with the new unique name
        shutil.copy(source_image_path, os.path.join(dest_image_dir, new_image_name))

        # Re-index and write the new label file
        new_lines = []
        with open(source_label_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    old_index = int(parts[0])
                    new_index = class_map.get(old_index)
                    if new_index is not None:
                        new_line = f"{new_index} {' '.join(parts[1:])}"
                        new_lines.append(new_line)

        if new_lines:
            dest_label_path = os.path.join(dest_label_dir, new_label_name)
            with open(dest_label_path, 'w') as f:
                f.write('\n'.join(new_lines))
            copied_count += 1
            
    return copied_count

def get_class_names_from_config(names_field):
    """Safely extracts class name strings from either a list or a dictionary."""
    if isinstance(names_field, dict):
        # Handles {0: 'name1', 1: 'name2'}
        return list(names_field.values())
    elif isinstance(names_field, list):
        # Handles ['name1', 'name2']
        return names_field
    return [] # Return empty list for unknown formats

def main(args):
    # --- 1. Load YAML files ---
    try:
        with open(args.dataset1, 'r') as f:
            data_config1 = yaml.safe_load(f)
        with open(args.dataset2, 'r') as f:
            data_config2 = yaml.safe_load(f)
    except Exception as e:
        print(f"Error reading YAML files: {e}")
        return

    # --- 2. Create the master class list (FIXED) ---
    # Safely get class names from both configs, regardless of format
    names1 = get_class_names_from_config(data_config1['names'])
    names2 = get_class_names_from_config(data_config2['names'])

    # Use a set to handle shared classes automatically, now with only strings
    master_class_set = set(names1) | set(names2)
    master_class_list = sorted(list(master_class_set)) # This will now work correctly
    
    print("--- Master Class List ---")
    for i, name in enumerate(master_class_list):
        print(f"{i}: {name}")
    print("-------------------------")

    # --- 3. Create class mappings from old index to new master index (FIXED) ---
    map1 = {old_idx: master_class_list.index(name) for old_idx, name in enumerate(names1)}
    map2 = {old_idx: master_class_list.index(name) for old_idx, name in enumerate(names2)}

    # --- 4. Create Output Directories ---
    output_dir = args.output
    os.makedirs(os.path.join(output_dir, 'train'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'valid'), exist_ok=True)

    # --- 5. Process and copy files for both datasets ---
    print("\n--- Processing Dataset 1 ---")
    d1_prefix = os.path.splitext(os.path.basename(args.dataset1))[0]
    d1_train_path = get_full_path(args.dataset1, data_config1['train'])
    d1_valid_path = get_full_path(args.dataset1, data_config1.get('val', data_config1['train']))
    process_and_copy_files(d1_train_path, os.path.join(output_dir, 'train'), map1, d1_prefix)
    process_and_copy_files(d1_valid_path, os.path.join(output_dir, 'valid'), map1, d1_prefix)

    print("\n--- Processing Dataset 2 ---")
    d2_prefix = os.path.splitext(os.path.basename(args.dataset2))[0]
    d2_train_path = get_full_path(args.dataset2, data_config2['train'])
    d2_valid_path = get_full_path(args.dataset2, data_config2.get('val', data_config2['train']))
    process_and_copy_files(d2_train_path, os.path.join(output_dir, 'train'), map2, d2_prefix)
    process_and_copy_files(d2_valid_path, os.path.join(output_dir, 'valid'), map2, d2_prefix)

    # --- 6. Create the final merged YAML file ---
    final_yaml_path = os.path.join(output_dir, 'merged_data.yaml')
    # Create the names dictionary with index-based format
    names_dict = {i: name for i, name in enumerate(master_class_list)}
    
    final_config = {
        'path': os.path.abspath(output_dir),
        'train': 'train/images',
        'val': 'valid/images',
        'nc': len(master_class_list),
        'names': names_dict
    }

    with open(final_yaml_path, 'w') as f:
        yaml.dump(final_config, f, sort_keys=False, default_flow_style=False)
        
    print("\n--- Merging Complete! ---")
    print(f"Merged dataset created at: {output_dir}")
    print(f"New master YAML file at: {final_yaml_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Merge two YOLOv8 datasets with potentially shared classes.")
    parser.add_argument('--dataset1', type=str, required=True, help='Path to the first dataset YAML file.')
    parser.add_argument('--dataset2', type=str, required=True, help='Path to the second dataset YAML file.')
    parser.add_argument('--output', type=str, required=True, help='Path to the output directory for the merged dataset.')
    
    args = parser.parse_args()
    main(args)
