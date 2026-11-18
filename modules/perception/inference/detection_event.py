"""Versioned person-detection event shared with downstream consumers."""
from __future__ import annotations

from datetime import datetime, timezone
import math
import uuid


SCHEMA_VERSION = "person-detection-event/1.0"


def utc_from_timestamp(timestamp):
    """Return an RFC 3339 UTC timestamp from a Unix timestamp."""
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat().replace("+00:00", "Z")


def build_person_detection_event(*, input_id, frame_id, detected_at_utc, image_width,
                                 image_height, model, detections, event_id=None):
    event = {
        "schema_version": SCHEMA_VERSION,
        "event_id": event_id or str(uuid.uuid4()),
        "input_id": input_id,
        "frame_id": frame_id,
        "detected_at_utc": detected_at_utc,
        "timestamp_source": "server_receive",
        "image": {"width_px": image_width, "height_px": image_height},
        "model": dict(model),
        "detections": [],
    }
    for index, detection in enumerate(detections):
        event["detections"].append({
            "detection_id": f"{frame_id}-person-{index + 1}",
            "class_name": "person",
            "confidence": detection["confidence"],
            "bbox_xyxy_px": list(detection["xyxy"]),
            "polygons_px": [[list(point) for point in polygon]
                            for polygon in detection["polygons"]],
        })
    validate_person_detection_event(event)
    return event


def validate_person_detection_event(event):
    required = {"schema_version", "event_id", "input_id", "frame_id", "detected_at_utc",
                "timestamp_source", "image", "model", "detections"}
    missing = required - set(event)
    if missing:
        raise ValueError(f"missing_event_fields:{','.join(sorted(missing))}")
    if event["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unsupported_schema_version")
    uuid.UUID(event["event_id"])
    for field in ("input_id", "frame_id", "detected_at_utc"):
        if not isinstance(event[field], str) or not event[field]:
            raise ValueError(f"invalid_{field}")
    if event["timestamp_source"] != "server_receive":
        raise ValueError("invalid_timestamp_source")
    width = event["image"].get("width_px")
    height = event["image"].get("height_px")
    if not isinstance(width, int) or not isinstance(height, int) or width < 1 or height < 1:
        raise ValueError("invalid_image_size")
    for field in ("name", "version", "weights_sha256"):
        if field not in event["model"]:
            raise ValueError(f"missing_model_{field}")
    for detection in event["detections"]:
        if detection.get("class_name") != "person":
            raise ValueError("invalid_class_name")
        confidence = detection.get("confidence")
        if not _finite_number(confidence) or not 0 <= confidence <= 1:
            raise ValueError("invalid_confidence")
        bbox = detection.get("bbox_xyxy_px")
        if not isinstance(bbox, list) or len(bbox) != 4 or not all(_finite_number(v) for v in bbox):
            raise ValueError("invalid_bbox")
        x1, y1, x2, y2 = bbox
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError("bbox_out_of_bounds")
        polygons = detection.get("polygons_px")
        if not isinstance(polygons, list) or not polygons:
            raise ValueError("invalid_polygons")
        for polygon in polygons:
            if not isinstance(polygon, list) or len(polygon) < 3:
                raise ValueError("invalid_polygon")
            for point in polygon:
                if (not isinstance(point, list) or len(point) != 2
                        or not all(_finite_number(v) for v in point)):
                    raise ValueError("invalid_polygon_point")
                if not (0 <= point[0] <= width and 0 <= point[1] <= height):
                    raise ValueError("polygon_out_of_bounds")
    return event


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
