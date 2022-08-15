# RPL - Computer Vision tools

A repository for computer vision tools at the RPL.

## rpl_cv

The rpl_cv/ directory contains a Python package with robot-specific methods and accompanying vision utilities.

### Installation

    git clone https://github.com/AD-SDL/rpl-camera-vision.git
    cd rpl-camera-vision
    pip install -e .

### Features

    Todo

### Examples

    Todo

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
