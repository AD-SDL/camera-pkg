# To train using yolo:
python train.py  --epochs 100 --imgsz 720 --weights yolov5m.pt --data OT_3_attempt2.yaml
python train.py --hyp runs/evolve/exp5/hyp_evolve.yaml --epochs 100 --imgsz 720 --weights runs/train/exp5/weights/best.pt --data OT_3_attempt2.yaml
# To detect
python detect.py --weights runs/train/exp61/weights/best.pt --source /home/hdantes/yolov5/UnityPerceptionData/IMG_5258.MOV
