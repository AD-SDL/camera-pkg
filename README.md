# RPL - Camera Vision protocols

A repository for CV related tasks at RPL

## Camera Clients

Python interface that allows remote commands to be executed

- Cameras are used for different operations / control / checks at RPL.

### Current features
* Camera initialization..
* Detect Wells
* Detect Pipetes
* read QR

## Install

    conda create -n rpl-cv-test python=3.8
    conda activate rpl-cv-test

    git clone https://github.com/AD-SDL/rpl-camera-driver.git
    cd rpl-camera-driver
    pip install -r requirements.txt
    pip install -e .

Better to install in develop-mode while the config is still changing


## Ros Install

## YOLOv5

- Provide training, validation, and test data in rpl-camera-vision/rpl_yolo/UnityPerceptionData/ as shown in rpl-camera-vision/rpl_yolo/UnityPerceptionData/README.md

- Create a YAML file similar to the example in rpl-camera-vision/rpl_yolo/data/OT2_test.yaml

- Train the model using the following command:

  - `python train.py --epochs 50 --imgsz 720 --weights yolov5s.pt --data OT2_test.yaml`

- Once the model is trained, test on an image using the following command:

  - `python detect.py --weights runs/train/exp/weights/best.pt --source ../UnityPerceptionData/OT2_1/images/rgb_2.png`
