import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random

from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, plot_one_box, set_logging
from utils.datasets import letterbox
from utils.torch_utils import select_device

opt = {}
opt["weights"] = "yolov5s.pt"
opt["img_size"] = 640
opt["conf_thres"] = 0.25
opt["iou_thres"] = 0.45
opt["device"] = "0"

def detect():
    image = cv2.imread("data/images/bus.jpg")
    # Padded resize
    image = letterbox(image, new_shape=opt["img_size"])[0]
    print(image.shape)

    # Convert
    img = image[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    print(img.shape)

    # Initialize
    set_logging()
    device = select_device(opt["device"])
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(opt["weights"], map_location=device)  # load FP32 model
    imgsz = check_img_size(opt["img_size"], s=model.stride.max())  # check img_size

    if half:
        model.half()  # to FP16

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

    # Run inference  
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0

    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    pred = model(img)[0]

    # Apply NMS
    pred = non_max_suppression(pred, opt["conf_thres"], opt["iou_thres"])

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        if det is not None and len(det):
            # Rescale boxes from img_size to image size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], image.shape).round()

            for *xyxy, conf, cls in reversed(det):
                label = '%s %.2f' % (names[int(cls)], conf)
                plot_one_box(xyxy, image, label=label, color=colors[int(cls)], line_thickness=3)

    cv2.imshow("result", image)
    cv2.waitKey(0)



if __name__ == '__main__':
    with torch.no_grad():
        detect()
