# backend_segnet.py
import numpy as np
import cv2
import jetson.inference
import jetson.utils

class SegNetBackend:
    """
    Jetson-Inference segNet backend.
    Returns a binary mask (uint8 0/255) for the selected class_id (e.g., Cityscapes sidewalk=8 or road=7).
    """
    def __init__(self, network="fcn-resnet18-cityscapes", class_id=8, threshold=0.5):
        """
        network: any segNet model name jetson-inference supports (e.g., fcn-resnet18-cityscapes)
        class_id: semantic class id you want to extract (Cityscapes: road=7, sidewalk=8)
        threshold: kept for API compatibility (class ID mask is already hard)
        """
        self.net = jetson.inference.segNet(network)
        self.class_id = int(class_id)
        self.threshold = float(threshold)

    def infer_mask(self, bgr_img: np.ndarray) -> np.ndarray:
        """
        bgr_img: numpy HxWx3, uint8 BGR
        returns: mask HxW uint8 (255 for desired class, else 0)
        """
        rgba = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGBA)
        cuda_img = jetson.utils.cudaFromNumpy(rgba)
        class_mask_cuda = self.net.Segment(cuda_img)
        class_mask = jetson.utils.cudaToNumpy(class_mask_cuda).astype(np.uint8)
        mask = (class_mask == self.class_id).astype(np.uint8) * 255
        return mask
