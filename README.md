# RPL - Computer Vision tools

A repository for computer vision tools at the RPL.

## rpl_cv

The rpl_cv/ directory contains a Python package with robot-specific methods and accompanying vision utilities.

### Installation

For a typical install:

    git clone https://github.com/AD-SDL/rpl-camera-vision.git
    cd rpl-camera-vision
    pip install .

For development:

    git clone https://github.com/AD-SDL/rpl-camera-vision.git
    cd rpl-camera-vision
    pip install -e .

To uninstall:

    pip uninstall rpl_cv

Troubleshooting:

Sometimes one package will require opencv-python while another requires opencv-contrib-python. Both cannot be installed at the same time. The easiest solution is to tell the package manager to ignore the conflict, then fix it manually in post. The following commands should handle nearly all situations.

    pip uninstall opencv-python
    pip uninstall opencv-contrib-python
    pip install opencv-contrib-python

Because opencv-contrib-python is a superset of opencv-python, it can satisfy all opencv-python requirements (even if the package manager cannot figure this out).

### Features

    Todo

### Examples

This example shows how to read well colors from plates on the OT-2. This example can be modified for extra debug-style output by changing `get_colors(img, dbg=0)` to `get_colors(img, dbg=2)`.

    python examples/plate_read_colors.py

This example requires two programs to be run simultaneously, and shows how to quickly set up a camera stream between two machines on the same network, where one machine performs a custom operation on the received images.

    python examples/video_receiver.py
    python examples/video_sender.py

## yolo

The yolo/ directory contains data directories and YOLO implementations for training and evaluating YOLO based models.

### Usage

- Provide training, validation, and test data in rpl-camera-vision/rpl_yolo/UnityPerceptionData/ as shown in rpl-camera-vision/rpl_yolo/UnityPerceptionData/README.md

- Create a YAML file similar to the example in rpl-camera-vision/rpl_yolo/data/OT2_test.yaml

- Train the model by running:

    `python train.py --epochs 50 --imgsz 720 --weights yolov5s.pt --data OT2_test.yaml`

- Once the model is trained, test on an image by running:

    `python detect.py --weights runs/train/exp/weights/best.pt --source ../UnityPerceptionData/OT2_1/images/rgb_2.png`

## dashboard

The dashboard/ directory contains a dashboard utility written with ROS2 and flask.

### Usage

    Todo

## node_editor

The node_editor/ directory contains a node editor GUI application written with PyQt5 to modularly design workflows.

### Usage

    Todo
