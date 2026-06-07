"""Vision-Language-Model interface for semantic room classification.

This module defines a *pluggable* adapter so the rest of the system never has
to know whether labels come from a real VLM or a simulation mock:

    VLMInterface (abstract)
        ├── MockVLM   - HSV colour analysis of the live camera frame (used in sim)
        └── ClipVLM   - real CLIP zero-shot classification (reference implementation)

Both return the exact same `VLMResult`, so `semantic_tagger_node` is identical
regardless of which backend is active. Swap the backend in one line.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np


# Canonical room labels the system understands.
ROOM_LABELS = ['kitchen', 'toilet', 'office', 'hallway']


@dataclass
class VLMResult:
    label: str           # one of ROOM_LABELS, or 'unknown'
    confidence: float    # 0.0 - 1.0
    description: str      # human-readable, e.g. "this image contains a toilet"


class VLMInterface:
    """Abstract base. A backend turns an RGB image into a room label."""

    def classify(self, rgb_image: np.ndarray) -> VLMResult:
        raise NotImplementedError



# ---------------------------------------------------------------------------- #
#     Mock backend image processing                                            #
# ---------------------------------------------------------------------------- #
class MockVLM(VLMInterface):
    """Recognise a room from the sign visible in the camera frame.

    Two complementary cues, both computed from the live `/camera/image_raw`
    pixels (never from the robot's position):

      1. COLOUR segmentation  - each sign has a distinct background colour
         (red=kitchen, blue=toilet, green=office). Robust to angle/distance.
      2. TEMPLATE matching    - the cropped colour region is matched against
         the actual sign PNG (the 'TOILET'/'KITCHEN'/'OFFICE' artwork). This
         confirms we are reading the sign content, not merely a colour blob.

    A real VLM (see ClipVLM) replaces both steps with one image embedding,
    but the interface and output are identical.
    """

    # (label, lower HSV, upper HSV). Hue is OpenCV-scaled 0-179.
    _COLOR_RULES = {
        'kitchen': [(np.array([0, 110, 70]),   np.array([12, 255, 255])),
                    (np.array([168, 110, 70]), np.array([180, 255, 255]))],
        'toilet':  [(np.array([100, 110, 70]), np.array([132, 255, 255]))],
        'office':  [(np.array([40, 70, 50]),   np.array([88, 255, 255]))],
    }

    def __init__(self, min_pixel_fraction: float = 0.03,
                 template_threshold: float = 0.35, signs_dir: str | None = None):
        self._min_fraction = min_pixel_fraction
        self._tmpl_thresh = template_threshold
        self._templates = self._load_templates(signs_dir)

    # ----------------------------- template loading ----------------------------- #
    def _load_templates(self, signs_dir):
        import cv2
        if signs_dir is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                signs_dir = os.path.join(
                    get_package_share_directory('tb3_semantic_mapping'), 'signs')
            except Exception:
                signs_dir = ''
        templates = {}
        for label in ('kitchen', 'toilet', 'office'):
            path = os.path.join(signs_dir, f'{label}_sign.png')
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE) if signs_dir else None
            if img is not None:
                templates[label] = img
        return templates

    # ---------- multi-scale template match within a region of interest ---------- #
    def _match(self, gray_roi, template) -> float:
        import cv2
        if gray_roi.size == 0 or template is None:
            return 0.0
        rh, rw = gray_roi.shape
        best = 0.0
        # ------ Try templates slightly smaller than the ROI at several scales. ------ #
        for frac in (0.9, 0.7, 0.5, 0.35):
            tw = int(rw * frac)
            th = max(6, int(template.shape[0] * tw / template.shape[1]))
            if tw < 20 or tw >= rw or th >= rh:
                continue
            resized = cv2.resize(template, (tw, th))
            res = cv2.matchTemplate(gray_roi, resized, cv2.TM_CCOEFF_NORMED)
            best = max(best, float(res.max()))
        return best

    def classify(self, rgb_image: np.ndarray) -> VLMResult:
        import cv2

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        total = hsv.shape[0] * hsv.shape[1]

        # --------- colour cue: best room + bounding box of its largest blob --------- #
        best_label, best_frac, best_box = None, 0.0, None
        for label, ranges in self._COLOR_RULES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, lo, hi)
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            frac = float(np.count_nonzero(mask)) / float(total)
            if frac > best_frac:
                xs = np.where(mask.any(axis=0))[0]
                ys = np.where(mask.any(axis=1))[0]
                box = (xs[0], ys[0], xs[-1], ys[-1]) if xs.size and ys.size else None
                best_label, best_frac, best_box = label, frac, box

        if best_label is None or best_frac < self._min_fraction:
            return VLMResult('hallway', 0.5,
                             'this image shows an open corridor (no room sign)')

        # ----- template cue: confirm by reading the sign artwork in that region ----- #
        tmpl_score = 0.0
        if best_box is not None and best_label in self._templates:
            x0, y0, x1, y1 = best_box
            pad = 10
            roi = gray[max(0, y0 - pad):y1 + pad, max(0, x0 - pad):x1 + pad]
            tmpl_score = self._match(roi, self._templates[best_label])

        if tmpl_score >= self._tmpl_thresh:
            confidence = float(min(0.99, 0.70 + tmpl_score * 0.29))
            desc = (f'detected and read the {best_label.upper()} sign in the image '
                    f'(match={tmpl_score:.2f})')
        else:
            confidence = float(min(0.90, 0.60 + best_frac * 2.0))
            desc = f'this image contains a {best_label} (sign colour visible)'

        return VLMResult(best_label, confidence, desc)



# ---------------------------------------------------------------------------- #
#                 Documents how a true VLM plugs in unchanged.                 #
# ---------------------------------------------------------------------------- #
class ClipVLM(VLMInterface):
    """Zero-shot room classification with OpenAI CLIP.

    Not used in the simulation demo (kept dependency-free), but shows that a
    real VLM is a drop-in replacement: it consumes the same RGB frame and
    returns the same `VLMResult`. To enable:  pip install torch ftfy clip
    then pass `backend='clip'` to the tagger node.
    """

    _PROMPTS = {
        'kitchen': 'a photo of an office kitchen or pantry',
        'toilet':  'a photo of a toilet or restroom',
        'office':  'a photo of an office with desks and chairs',
        'hallway': 'a photo of an empty hallway or corridor',
    }

    def __init__(self, model_name: str = 'ViT-B/32'):
        import clip      
        import torch

        self._torch = torch
        self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self._model, self._preprocess = clip.load(model_name, device=self._device)
        self._labels = list(self._PROMPTS.keys())
        text = clip.tokenize(list(self._PROMPTS.values())).to(self._device)
        with torch.no_grad():
            self._text_features = self._model.encode_text(text)
            self._text_features /= self._text_features.norm(dim=-1, keepdim=True)

    def classify(self, rgb_image: np.ndarray) -> VLMResult:
        from PIL import Image

        img = self._preprocess(Image.fromarray(rgb_image)).unsqueeze(0).to(self._device)
        with self._torch.no_grad():
            feat = self._model.encode_image(img)
            feat /= feat.norm(dim=-1, keepdim=True)
            probs = (100.0 * feat @ self._text_features.T).softmax(dim=-1)[0]

        idx = int(probs.argmax())
        label = self._labels[idx]
        return VLMResult(label, float(probs[idx]), f'this image contains a {label}')


def make_vlm(backend: str = 'mock', **kwargs) -> VLMInterface:
    """Factory: return the requested VLM backend."""
    if backend == 'mock':
        return MockVLM(**kwargs)
    if backend == 'clip':
        return ClipVLM(**kwargs)
    raise ValueError(f'Unknown VLM backend: {backend!r}')
