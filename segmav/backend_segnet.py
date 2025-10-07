# backend_segnet.py
import numpy as np
import cv2
import jetson.inference
import jetson.utils

class SegNetBackend:
    def __init__(self, network='fcn-resnet18-cityscapes', class_id=8, threshold=0.5):
        self.net = jetson.inference.segNet(network)
        self.class_id = int(class_id)
        self.threshold = float(threshold)

    def infer_mask(self, bgr_img: np.ndarray) -> np.ndarray:
        rgba = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGBA)
        cuda_img = jetson.utils.cudaFromNumpy(rgba)
        class_mask_cuda = self.net.Segment(cuda_img)
        class_mask = jetson.utils.cudaToNumpy(class_mask_cuda).astype(np.uint8)
        return ((class_mask == self.class_id).astype(np.uint8) * 255)
