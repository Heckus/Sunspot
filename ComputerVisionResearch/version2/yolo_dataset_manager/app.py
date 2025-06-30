import streamlit as st
from streamlit_drawable_canvas import st_canvas
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter
from PIL import Image
from utils import *

st.set_page_config(layout="wide", page_title="YOLOv8 Dataset Manager")

# --- Session State Initialization ---
def init_session_state():
    defaults = {
        'image_sets': {}, 'all_images': [], 'class_names': {}, 'colors': [], 
        'models': [], 'current_view': 'grid', 'selected_image': None
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

init_session_state()

# --- Data Loading ---
def load_data(yaml_path):
    try:
        config = load_config(yaml_path)
        st.session_state.class_names = config.get('names', {})
        st.session_state.class_list = list(st.session_state.class_names.values())

        np.random.seed(42)
        st.session_state.colors = [tuple(np.random.randint(100, 256, 3)) for _ in st.session_state.class_names]
        
        dataset_root = config.get('path', '')
        if not os.path.isabs(dataset_root):
            dataset_root = os.path.join(os.path.dirname(yaml_path), dataset_root)
        dataset_root = os.path.abspath(dataset_root)

        st.session_state.image_sets = {
            'train': get_image_files(dataset_root, config.get('train', '')),
            'valid': get_image_files(dataset_root, config.get('val', '')),
            'test': get_image_files(dataset_root, config.get('test', ''))
        }
        st.session_state.all_images = sorted([img for split in st.session_state.image_sets.values() for img in split])
    except Exception as e:
        st.error(f"Error loading dataset: {e}")

# --- Sidebar ---
st.sidebar.title("YOLOv8 Manager")
yaml_path_input = st.sidebar.text_input("Absolute path to data.yaml", placeholder="/path/to/data.yaml")
if yaml_path_input and os.path.exists(yaml_path_input):
    if st.session_state.get('loaded_yaml') != yaml_path_input:
        load_data(yaml_path_input)
        st.session_state['loaded_yaml'] = yaml_path_input
        st.rerun()

if not st.session_state.image_sets:
    st.info("Please provide a valid `data.yaml` path in the sidebar to get started.")
    st.stop()

# --- Enhanced Metrics ---
st.sidebar.header("📊 Dataset Analytics")
all_labels_data = []
all_image_dims = []
for split in st.session_state.image_sets.values():
    for img_path in split:
        if os.path.exists(img_path):
            with Image.open(img_path) as img:
                all_image_dims.append(img.size)
            labels = parse_yolo_labels(get_label_path(img_path))
            for l in labels:
                all_labels_data.append({'class_id': int(l[0]), 'w': l[3], 'h': l[4]})

if all_labels_data:
    df = pd.DataFrame(all_labels_data)
    df_dims = pd.DataFrame(all_image_dims, columns=['width', 'height'])

    st.sidebar.metric("Total Images", len(st.session_state.all_images))
    st.sidebar.metric("Total Bounding Boxes", len(df))
    st.sidebar.metric("Avg Boxes / Image", f"{len(df) / len(st.session_state.all_images):.2f}")
    
    st.sidebar.subheader("Class Distribution")
    class_counts = df['class_id'].map(st.session_state.class_names).value_counts()
    st.sidebar.bar_chart(class_counts)

    st.sidebar.subheader("Image Dimensions")
    fig, ax = plt.subplots(figsize=(5,3))
    ax.hist(df_dims['width'], bins=20, alpha=0.7, label='Width')
    ax.hist(df_dims['height'], bins=20, alpha=0.7, label='Height')
    ax.legend()
    st.sidebar.pyplot(fig)

    st.sidebar.subheader("Annotation Size (W vs H)")
    fig, ax = plt.subplots(figsize=(5,3))
    ax.scatter(df['w'], df['h'], alpha=0.2)
    ax.set_xlabel("Normalized Width")
    ax.set_ylabel("Normalized Height")
    st.sidebar.pyplot(fig)

# --- Main App ---
main_tabs = st.tabs(["Dataset Manager", "Model Tester"])

## =============================================
##          DATASET MANAGER TAB
## =============================================
with main_tabs[0]:
    if st.session_state.selected_image:
        # --- DETAIL / EDIT VIEW ---
        st.header("📝 Annotation Editor")
        img_path = st.session_state.selected_image
        
        if not os.path.exists(img_path):
            st.warning("This image no longer exists. Returning to grid view.")
            st.session_state.selected_image = None
            st.rerun()

        bg_image = Image.open(img_path)
        img_w, img_h = bg_image.size
        
        # Prepare initial rectangles for the canvas from the label file
        initial_rects = []
        labels = parse_yolo_labels(get_label_path(img_path))
        for l in labels:
            class_id, x_min, y_min, x_max, y_max = yolo_to_abs(l, img_w, img_h)
            initial_rects.append({
                "type": "rect", "left": x_min, "top": y_min, "width": x_max - x_min, "height": y_max - y_min,
                "fill": "rgba(0,0,0,0)", "stroke": f"rgb{st.session_state.colors[class_id % len(st.session_state.colors)]}", "strokeWidth": 2,
                "label": f"{class_id} {st.session_state.class_names.get(class_id, 'Unknown')}"
            })

        col1, col2 = st.columns([3, 1])
        with col1:
            canvas_result = st_canvas(
                fill_color="rgba(255, 165, 0, 0.3)",
                stroke_width=2,
                stroke_color="#FF0000",
                background_image=bg_image,
                height=img_h,
                width=img_w,
                initial_drawing={"objects": initial_rects},
                drawing_mode="transform", # rect, circle, line, transform
                key="canvas",
            )
        
        with col2:
            st.subheader("Tools")
            if st.button("⬅️ Back to Grid View", use_container_width=True):
                st.session_state.selected_image = None
                st.rerun()
            
            # Annotation tools
            selected_class_name = st.selectbox("Select Class for New Box", options=st.session_state.class_list)
            
            if st.button("💾 Save Annotations", use_container_width=True):
                if canvas_result.json_data is not None:
                    new_annotations = [obj for obj in canvas_result.json_data["objects"] if 'label' in obj]
                    # Update labels of new boxes with the selected class
                    for ann in new_annotations:
                         if not ann['label']: # Newly drawn box
                            class_id = st.session_state.class_list.index(selected_class_name)
                            ann['label'] = f"{class_id} {selected_class_name}"
                    
                    save_annotations_to_file(get_label_path(img_path), new_annotations, img_w, img_h, st.session_state.class_names)
                    st.success(f"Annotations saved for {os.path.basename(img_path)}")
                else:
                    st.warning("No annotations to save.")

            st.subheader("File Management")
            if st.button("🚩 Flag Image", use_container_width=True):
                flag_image(img_path)
                st.info(f"Flagged {os.path.basename(img_path)}")

            if st.button("❌ Delete Image", type="primary", use_container_width=True):
                st.session_state.delete_confirm = True

            if st.session_state.get('delete_confirm'):
                st.warning(f"Really delete {os.path.basename(img_path)} and its label permanently?")
                if st.button("YES, DELETE IT", use_container_width=True):
                    success, message = delete_image_and_label(img_path)
                    if success:
                        st.success(message)
                        st.session_state.selected_image = None
                        st.session_state.delete_confirm = False
                        st.rerun()
                    else:
                        st.error(message)
                if st.button("CANCEL", use_container_width=True):
                    st.session_state.delete_confirm = False
                    st.rerun()


    else:
        # --- GRID VIEW ---
        st.header("🖼️ Dataset Grid View")
        split_to_show = st.selectbox("Select Dataset Split", options=st.session_state.image_sets.keys())
        image_list = st.session_state.image_sets[split_to_show]

        if not image_list:
            st.info(f"No images found in the '{split_to_show}' split.")
        else:
            num_cols = st.slider("Number of columns", 2, 10, 4)
            cols = st.columns(num_cols)
            for i, img_path in enumerate(image_list):
                with cols[i % num_cols]:
                    try:
                        image = Image.open(img_path)
                        labels = parse_yolo_labels(get_label_path(img_path))
                        annotated_image = draw_bounding_boxes_on_image(image.copy(), labels, st.session_state.class_names, st.session_state.colors)
                        
                        st.image(annotated_image, caption=f"{os.path.basename(img_path)} ({len(labels)} boxes)", use_container_width=True)
                        if st.button("Select / Edit", key=f"select_{img_path}", use_container_width=True):
                            st.session_state.selected_image = img_path
                            st.session_state.delete_confirm = False
                            st.rerun()
                    except Exception as e:
                        st.error(f"Failed to load {os.path.basename(img_path)}: {e}")

## =============================================
##             MODEL TESTER TAB
## =============================================
with main_tabs[1]:
    st.header("Model Tester")
    st.write("Test your trained `.pt` models on single frames from your dataset.")

    col_lhs, col_mid, col_rhs = st.columns([1, 2, 1])

    with col_rhs:
        st.subheader("⚙️ Model Selection")
        model_dir = st.text_input("Enter path to your models directory", "models_ball3")
        
        if os.path.isdir(model_dir):
            st.session_state.models = find_pt_files(model_dir)
            if st.session_state.models:
                selected_model_name = st.selectbox("Select a Model", [os.path.basename(m) for m in st.session_state.models])
                model_path = os.path.join(model_dir, selected_model_name)
            else:
                st.warning(f"No `.pt` files found in `{model_dir}`.")
                model_path = None
        else:
            st.error(f"Directory not found: `{model_dir}`")
            model_path = None

    with col_lhs:
        st.subheader("🖼️ Image Selection")
        if st.session_state.all_images:
            selected_image_path = st.selectbox("Select an Image", st.session_state.all_images, format_func=os.path.basename)
        else:
            st.warning("Load a dataset in the 'Dataset Viewer' tab to select images.")
            selected_image_path = None

    with col_mid:
        st.subheader("👁️‍🗨️ Inference View")
        if selected_image_path:
            st.text("Selected Frame:")
            st.image(selected_image_path, use_container_width=True, caption=f"Input: {os.path.basename(selected_image_path)}")
            
            st.divider()

            st.text("Model Output:")
            if model_path and st.button("Run Test on Frame", use_container_width=True):
                with st.spinner("Running inference..."):
                    try:
                        annotated_image = run_inference(model_path, selected_image_path)
                        st.image(annotated_image, use_container_width=True, caption=f"Prediction by: {os.path.basename(model_path)}")
                    except Exception as e:
                        st.error(f"Failed to run inference: {e}")
            else:
                 st.info("Click the button to see the model's prediction.")
        else:
            st.info("Select an image from the left panel to begin.")