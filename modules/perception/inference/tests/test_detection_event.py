import json
from pathlib import Path
import unittest

from modules.perception.inference.detection_event import validate_person_detection_event


SAMPLES = Path(__file__).parents[4] / "contracts" / "perception" / "samples" / "person_detection_event_v1"


class PersonDetectionEventTests(unittest.TestCase):
    def test_public_samples_are_valid(self):
        expected_counts = {"no_person.json": 0, "one_person.json": 1, "multiple_people.json": 2}
        for name, expected_count in expected_counts.items():
            with self.subTest(name=name):
                event = json.loads((SAMPLES / name).read_text(encoding="utf-8"))
                self.assertIs(validate_person_detection_event(event), event)
                self.assertEqual(len(event["detections"]), expected_count)

    def test_rejects_out_of_bounds_bbox(self):
        event = json.loads((SAMPLES / "one_person.json").read_text(encoding="utf-8"))
        event["detections"][0]["bbox_xyxy_px"][2] = 641
        with self.assertRaisesRegex(ValueError, "bbox_out_of_bounds"):
            validate_person_detection_event(event)


if __name__ == "__main__":
    unittest.main()
