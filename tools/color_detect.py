"""Nhan dien mau tren khung hinh BGR cua OpenCV.

Module nay chi tra loi "thay gi, o dau", KHONG quyet dinh lam gi. Khong cham
mang, khong cham GUI -> test duoc bang anh tu sinh
(python/test_color_detect_offline.py).

HAI CAI BAY:
  1. Do NAM VAT QUA MOC 0 cua Hue (OpenCV H = 0..179): do thuan o CA HAI dau
     ~0..10 va ~170..179. Mot khoang don bat duoc dung mot nua sac do -> moi
     ColorProfile giu mot DANH SACH khoang.
  2. Nguong dien tich tinh theo TY LE khung, khong theo diem anh -> doi do
     phan giai khong phai chinh lai gi.

DUNG KET QUA (cho phan dieu khien sau nay):
    det = ColorDetector()
    for d in det.detect(frame_bgr):
        d.label              # "red"|"orange"|"yellow"|"green"|"blue"|"purple"
        d.nx, d.ny           # vi tri chuan hoa -1..+1, GOC O TAM anh
                             #   nx < 0 = ben TRAI tam, ny < 0 = PHIA TREN tam
        d.area_frac          # dien tich / dien tich khung, 0..1 -- proxy do XA
        d.cx, d.cy, d.bbox   # don vi diem anh, de ve overlay
Toa do chuan hoa la thu duy nhat nen dua vao dieu khien: no khong doi khi doi
do phan giai, va dau cua no da khop voi truc anh (phai duong, xuong duong).
"""
from __future__ import annotations

# H trong OpenCV: 0..179 (KHONG phai 0..359 -- chia doi de vua mot byte).
# S, V: 0..255.
HUE_MAX = 179


class ColorProfile:
    """Ten + DANH SACH khoang HSV + nguong dien tich.

    La danh sach vi mau do vat qua moc 0; cac mau khac chi can mot khoang nhung
    giu chung kieu de detect() khong phai co hai duong xu ly.
    """

    def __init__(self, name, hsv_ranges, min_area_frac=0.002, max_blobs=3):
        self.name = name
        self.hsv_ranges = list(hsv_ranges)
        self.min_area_frac = min_area_frac
        self.max_blobs = max_blobs

    def __repr__(self):
        return "ColorProfile(%r, %d khoang)" % (self.name, len(self.hsv_ranges))


# Bang nguong HSV do nguoi dung cung cap (HSV_RANGES_UAV), da tinh chinh cho
# UAV: san S/V dat RIENG tung mau thay vi dung chung 43/46.
#   vang  V>=90  -- vang von sang, san cao de khong bat vung nhat mau
#   cam   S>=100 -- cam nam sat do/vang nen can do bao hoa cao moi tach duoc
#   tim   S>=70  -- vat mau tim thuc te thuong kem bao hoa hon
#
# Cac khoang H tiep giap nhau va phu gan het vong mau -> mot diem anh chi thuoc
# DUNG MOT mau. Khoang 78..99 (xanh ngoc) co y de trong.
# Do van phai HAI khoang (0..10 va 156..179) vi no vat qua moc 0 cua Hue.
#
# ⚠ San V 60..90 la danh doi CO THAT: xem do trong docstring dau file -- vat
# trong bong sau (V tut ve ~55) se bi loai. Neu bay ngoai troi co bong do manh
# thi ha V min truoc, dung dong H.
DEFAULT_PROFILES = [
    ColorProfile('red',    [((0, 90, 70), (10, 255, 255)),
                            ((156, 90, 70), (179, 255, 255))]),
    ColorProfile('orange', [((11, 100, 80), (25, 255, 255))]),
    ColorProfile('yellow', [((26, 90, 90), (34, 255, 255))]),
    ColorProfile('green',  [((35, 80, 60), (77, 255, 255))]),
    ColorProfile('blue',   [((100, 80, 60), (124, 255, 255))]),
    ColorProfile('purple', [((125, 70, 60), (155, 255, 255))]),
]


class Detection:
    """Mot vet mau tim duoc. Xem docstring dau file."""

    __slots__ = ("label", "cx", "cy", "nx", "ny", "area_px", "area_frac", "bbox")

    def __init__(self, label, cx, cy, nx, ny, area_px, area_frac, bbox):
        self.label = label
        self.cx = cx
        self.cy = cy
        self.nx = nx
        self.ny = ny
        self.area_px = area_px
        self.area_frac = area_frac
        self.bbox = bbox          # (x, y, w, h)

    def __repr__(self):
        return ("Detection(%s, n=(%+.2f,%+.2f), area=%.3f%%)"
                % (self.label, self.nx, self.ny, self.area_frac * 100.0))


class ColorDetector:
    """Tim cac vet mau theo profile. Mot the hien dung lai duoc moi khung."""

    def __init__(self, profiles=None, blur_ksize=5, morph_ksize=5):
        self.profiles = list(profiles) if profiles else list(DEFAULT_PROFILES)
        self.blur_ksize = blur_ksize      # 0 = tat
        self.morph_ksize = morph_ksize    # 0 = tat
        self.enabled = {p.name: True for p in self.profiles}
        self._kernel = None

    # ---------------------------------------------------------------- mask
    def build_mask(self, frame_bgr, profile):
        """Mat na nhi phan cho MOT profile; tach rieng de GUI xem duoc."""
        import cv2
        import numpy as np

        img = frame_bgr
        if self.blur_ksize and self.blur_ksize >= 3:
            k = self.blur_ksize | 1          # ksize phai LE
            img = cv2.GaussianBlur(img, (k, k), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = None
        for lo, hi in profile.hsv_ranges:
            m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if mask is None:
            return np.zeros(frame_bgr.shape[:2], np.uint8)

        if self.morph_ksize and self.morph_ksize >= 3:
            if self._kernel is None or self._kernel.shape[0] != self.morph_ksize:
                self._kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (self.morph_ksize, self.morph_ksize))
            # OPEN xoa hat nhieu, CLOSE va lo thung. Thu tu quan trong:
            # CLOSE truoc se phong to nhieu roi xoa khong het.
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        return mask

    # ------------------------------------------------------------- detect
    def detect(self, frame_bgr):
        """Tra list[Detection], sap theo dien tich giam dan (to nhat truoc)."""
        import cv2

        if frame_bgr is None or frame_bgr.size == 0:
            return []
        h, w = frame_bgr.shape[:2]
        total = float(w * h)
        half_w, half_h = w / 2.0, h / 2.0
        out = []

        for prof in self.profiles:
            if not self.enabled.get(prof.name, True):
                continue
            mask = self.build_mask(frame_bgr, prof)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            found = []
            for c in cnts:
                area = float(cv2.contourArea(c))
                if area / total < prof.min_area_frac:
                    continue
                m = cv2.moments(c)
                if m["m00"] <= 0.0:
                    continue
                cx = m["m10"] / m["m00"]
                cy = m["m01"] / m["m00"]
                found.append(Detection(
                    label=prof.name, cx=cx, cy=cy,
                    # Goc o TAM anh: -1 mep trai, +1 mep phai.
                    nx=(cx - half_w) / half_w,
                    ny=(cy - half_h) / half_h,
                    area_px=area, area_frac=area / total,
                    bbox=cv2.boundingRect(c)))
            found.sort(key=lambda d: d.area_px, reverse=True)
            out.extend(found[:prof.max_blobs])

        out.sort(key=lambda d: d.area_px, reverse=True)
        return out

    # ------------------------------------------------------------ overlay
    def draw(self, frame_bgr, detections):
        """Ve khung + tam len BAN SAO — ve de len anh goc se doi mau cua no,
        va phan dieu khien co the con muon chay lai detect() tren khung do.
        """
        import cv2

        img = frame_bgr.copy()
        h, w = img.shape[:2]
        # Vach tam de doc dau nx/ny bang mat.
        cv2.line(img, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)
        cv2.line(img, (0, h // 2), (w, h // 2), (60, 60, 60), 1)

        for d in detections:
            x, y, bw, bh = d.bbox
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
            cv2.circle(img, (int(d.cx), int(d.cy)), 4, (0, 0, 255), -1)
            cv2.putText(img, "%s %+.2f,%+.2f" % (d.label, d.nx, d.ny),
                        (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 255), 1, cv2.LINE_AA)
        return img
