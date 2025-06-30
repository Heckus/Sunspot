# YOLOv8 Dataset Manager & Annotation Tool

An interactive Streamlit application to visualize, manage, analyze, and edit your YOLOv8 datasets.

This tool provides a comprehensive suite of features to streamline your computer vision workflow, moving beyond simple viewing to active dataset curation.

## Features

-   **File Manager Grid View:**
    -   View your entire dataset split (train, valid, test) in a configurable grid.
    -   See annotations overlaid on all images at once for rapid scanning.
    -   Select any image to enter the powerful "Edit Mode".

-   **Advanced Image Management:**
    -   **Flag Images:** Mark problematic or interesting images for later review. Flagged image paths are saved to `flagged_images.txt`.
    -   **Delete Images:** Permanently remove an image and its corresponding label file from the dataset (with a confirmation step).

-   **Visual Annotation Editor:**
    -   **Edit Boxes:** Adjust the position and size of existing bounding boxes by dragging their corners.
    -   **Add Boxes:** Draw new bounding boxes directly onto the image.
    -   **Delete Boxes:** Remove incorrect annotations.
    -   **Change Classes:** Assign the correct class to new or existing boxes.
    -   **Save Changes:** Overwrite the label file with your corrected annotations.

-   **In-Depth Dataset Analytics:**
    -   **Class Distribution:** Visualize the number of instances per class.
    -   **Annotation Stats:** See total bounding box counts and average boxes per image.
    -   **Image Dimensions:** Analyze the distribution of image resolutions in your dataset.
    -   **Bounding Box Size Analysis:** View a scatter plot of annotation width vs. height to understand object size distribution.

-   **Model Testing:**
    -   Load and test your own `.pt` model files on any image from your dataset.

## Setup and Installation

1.  **Clone the repository or save the files** into a directory.

2.  **Install the required libraries.** It's recommended to do this in a virtual environment.

    ```bash
    pip install -r requirements.txt
    ```

## How to Run

1.  Navigate to the project directory in your terminal.

2.  Run the Streamlit app:

    ```bash
    streamlit run app.py
    ```