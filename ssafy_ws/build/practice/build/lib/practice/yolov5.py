import torch

class YOLOv5:
    def __init__(self, model_path, device="cpu"):
        self.device = device
        self.model = torch.hub.load(
            'ultralytics/yolov5', 'custom', path=model_path
        ).to(device)

    def detect(self, img):
        results = self.model(img)
        return results