"""Pure helpers for rendering semantic-segmentation results."""
from __future__ import annotations

import colorsys

import cv2
import numpy as np


DEFAULT_CLASSES = ("wall", "floor", "ceiling", "table", "chair")


def class_color(class_id: int) -> tuple[int, int, int]:
    """Return a stable, visually separated BGR color for a class id."""
    hue = (class_id * .618033988749895) % 1.0
    red, green, blue = colorsys.hsv_to_rgb(hue, .72, 1.0)
    return round(blue * 255), round(green * 255), round(red * 255)


def filtered_label_map(probabilities: np.ndarray, class_ids: set[int], threshold: float) -> np.ndarray:
    """Convert CxHxW probabilities to a filtered label map; -1 means hidden."""
    if probabilities.ndim != 3 or probabilities.shape[0] < 1:
        raise ValueError("probabilities_must_be_c_h_w")
    if not 0 <= threshold <= 1:
        raise ValueError("threshold_out_of_range")
    labels = probabilities.argmax(axis=0)
    confidence = probabilities.max(axis=0)
    visible = np.isin(labels, list(class_ids)) & (confidence >= threshold)
    return np.where(visible, labels, -1).astype(np.int32)


def render_overlay(frame: np.ndarray, label_map: np.ndarray, id2label: dict[int, str], alpha: float = .5):
    """Blend class masks on a BGR frame and return per-class pixel coverage."""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame_must_be_bgr")
    if label_map.shape != frame.shape[:2]:
        raise ValueError("label_map_size_mismatch")
    if not 0 <= alpha <= 1:
        raise ValueError("alpha_out_of_range")
    color_layer = frame.copy()
    coverage = []
    total = label_map.size
    for class_id in sorted(int(value) for value in np.unique(label_map) if value >= 0):
        mask = label_map == class_id
        color_layer[mask] = class_color(class_id)
        coverage.append({"class_id": class_id, "label": id2label[class_id],
                         "pixels": int(mask.sum()), "coverage": round(float(mask.sum()) / total, 4)})
    visible = label_map >= 0
    blended = cv2.addWeighted(frame, 1 - alpha, color_layer, alpha, 0)
    output = frame.copy()
    output[visible] = blended[visible]
    return output, coverage


def resolve_class_ids(id2label: dict[int, str], requested: list[str]) -> set[int]:
    normalized = {name.strip().casefold() for name in requested if name.strip()}
    available = {label.casefold(): int(class_id) for class_id, label in id2label.items()}
    missing = normalized - set(available)
    if missing:
        raise ValueError("unknown_classes:" + ",".join(sorted(missing)))
    return {available[name] for name in normalized}
