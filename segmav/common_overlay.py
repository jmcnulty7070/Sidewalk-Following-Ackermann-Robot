# common_overlay.py
import cv2, math, collections
import numpy as np

PURPLE = (180, 0, 180)
RED    = (0, 0, 255)
BLUE   = (255, 0, 0)
WHITE  = (255, 255, 255)
YELLOW = (0, 255, 255)

def flip_img(img, mode):
    if mode == 'rotate-180':
        return cv2.rotate(img, cv2.ROTATE_180)
    if mode == 'flip-horizontal':
        return cv2.flip(img, 1)
    if mode == 'flip-vertical':
        return cv2.flip(img, 0)
    return img

def largest_contour(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(cnts, key=cv2.contourArea) if cnts else None

def centroid(submask):
    M = cv2.moments(submask, binaryImage=True)
    if M['m00'] <= 0:
        return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy)

class AngleSmoother3:
    def __init__(self): self.buf = collections.deque(maxlen=3)
    def add(self, deg):
        self.buf.append(float(deg))
        return sum(self.buf)/len(self.buf) if self.buf else 0.0

def draw_steven_overlay(frame_bgr, mask):
    h, w = mask.shape[:2]
    out = frame_bgr.copy()

    purple = np.zeros_like(out, dtype=np.uint8); purple[:] = PURPLE
    alpha = 0.35
    mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) // 255
    overlay = (purple * mask_3).astype(np.uint8)
    out = cv2.addWeighted(out, 1.0, overlay, alpha, 0)

    cnt = largest_contour(mask)
    if cnt is None or cv2.contourArea(cnt) < 400:
        cv2.putText(out, 'No contour', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)
        return 0.0, out

    cv2.drawContours(out, [cnt], -1, RED, 2)

    midy = h // 2
    cv2.rectangle(out, (0, 0), (w, midy), BLUE, 2)
    cv2.rectangle(out, (0, midy), (w, h), BLUE, 2)

    top = mask[:midy, :]
    bot = mask[midy:, :]

    c1 = centroid(top)
    c2 = centroid(bot)

    if c1 is None or c2 is None:
        cv2.putText(out, 'Centroids missing', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)
        return 0.0, out

    cv2.circle(out, (c1[0], c1[1]), 5, WHITE, -1)
    cv2.circle(out, (c2[0], c2[1] + midy), 5, WHITE, -1)

    p1 = (c1[0], c1[1])
    p2 = (c2[0], c2[1] + midy)
    cv2.line(out, p1, p2, WHITE, 2)

    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]
    angle_deg = math.degrees(math.atan2(dx, dy))
    return angle_deg, out
