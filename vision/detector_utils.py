# vision/detector_utils.py
import torch
import numpy as np
import cv2

class ObjectDetector:
    def __init__(self, model_path="yolov5s.pt", device="cuda"):
        self.device = device
        self.model = None

        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.backend = "ultralytics"
        except Exception:
            self.model = None

        if self.model is None:
            self.model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path)
            self.backend = "torch.hub"

        if hasattr(self.model, "to"):
            self.model.to(device)

    def detect_objects(self, image, conf_thres=0.5):
        detections = []

        if self.backend == "ultralytics":
            results = self.model(image[..., ::-1])  # RGB
            for r in results:
                boxes = r.boxes
                masks = getattr(r, "masks", None)  # segmentation 结果（可能为 None）

                for i, b in enumerate(boxes):
                    x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())
                    conf = float(b.conf[0].cpu().numpy())
                    cls  = int(b.cls[0].cpu().numpy())
                    if conf < conf_thres:
                        continue

                    det = {
                        "bbox": (x1, y1, x2, y2),
                        "conf": conf,
                        "class": cls
                    }

                    # 如果有 mask
                    if masks is not None and i < len(masks.data):
                        mask = masks.data[i].cpu().numpy()
                        mask = (mask * 255).astype(np.uint8)
                        mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
                        _, mask_bin = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
                        det["mask"] = mask_bin
                        det["area"] = float(mask_bin.sum())  # 用面积做选择策略
                    else:
                        det["mask"] = None
                        det["area"] = float((x2 - x1) * (y2 - y1))

                    detections.append(det)

        else:
            # torch.hub yolov5（只有 bbox）
            results = self.model(image[..., ::-1])
            for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
                if conf < conf_thres:
                    continue
                x1, y1, x2, y2 = map(int, xyxy)
                detections.append({
                    "bbox": (x1, y1, x2, y2),
                    "conf": float(conf),
                    "class": int(cls),
                    "mask": None,
                    "area": float((x2-x1)*(y2-y1))
                })

        return detections
