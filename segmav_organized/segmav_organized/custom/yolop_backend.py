import cv2, torch, numpy as np

class YOLOPBackend:
    """YOLOP drivable-area backend. Returns uint8 mask (255=navigable)."""
    def __init__(self, device=None, half=True):
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.model  = torch.hub.load("hustvl/YOLOP", "yolop", pretrained=True).to(self.device).eval()
        self.half   = (half and self.device.type=="cuda")
        if self.half: self.model.half()
        self.inp_w, self.inp_h = 640, 384

    @torch.inference_mode()
    def infer_mask(self, bgr):
        h0,w0 = bgr.shape[:2]
        img = cv2.resize(bgr, (self.inp_w,self.inp_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)/255.0
        t   = torch.from_numpy(img).permute(2,0,1).unsqueeze(0).to(self.device)
        if self.half: t = t.half()
        _, da, _ = self.model(t)              # det, drivable, lane
        da = torch.sigmoid(da).squeeze().float().cpu().numpy()
        mask = (da>0.5).astype(np.uint8)*255
        return cv2.resize(mask, (w0,h0), interpolation=cv2.INTER_NEAREST)
