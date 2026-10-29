from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from modules.mapping.localization.fixed_pose_association import FixedPoseAssociation
from modules.mapping.localization.landmarks import confirm_landmarks
from modules.mapping.localization.tests.test_landmarks import snapshot, CAMERA
from modules.mapping.localization.orbslam3 import convert


def split_views(xyz=None):
    native = snapshot(centers=(0., .3, .6, .9, 1.2, 1.5),
                      xyz=xyz or [[.1, .2, 5.], [.1, .2, 5.]])
    for p in native["points"]:
        p["descriptor"] = [0] * 32
    for i, frame in enumerate(native["keyframes"]):
        frame["observations"] = [o for o in frame["observations"] if o["point_id"] == (50 if i < 3 else 51)]
    return native


class FixedPoseTests(unittest.TestCase):
    def test_same_landmark_in_separate_views_merges_without_changing_any_pose(self):
        native = split_views()
        original = deepcopy(native)
        fixed, _, _ = confirm_landmarks(native, CAMERA)
        association = FixedPoseAssociation()
        merged, quality = association.apply(fixed, CAMERA)
        confirmed, _, _ = confirm_landmarks(merged, CAMERA)
        self.assertEqual([p["id"] for p in confirmed["points"]], [50])
        self.assertEqual(confirmed["points"][0]["observations"], 6)
        self.assertEqual(quality["merged_pairs"], 1)
        self.assertFalse(quality["camera_poses_modified"])
        self.assertEqual([f["T_cw"] for f in merged["keyframes"]], [f["T_cw"] for f in native["keyframes"]])
        self.assertEqual(native, original)

    def test_same_pixel_or_descriptor_cannot_merge_incompatible_depth(self):
        fixed, _, _ = confirm_landmarks(split_views([[.1, .2, 5.], [.1, .2, 10.]]), CAMERA)
        merged, quality = FixedPoseAssociation().apply(fixed, CAMERA)
        self.assertEqual(len(merged["points"]), 2)
        self.assertEqual(quality["merged_pairs"], 0)

    def test_same_color_and_position_require_descriptor_agreement(self):
        fixed, _, _ = confirm_landmarks(split_views(), CAMERA)
        fixed["points"][1]["descriptor"] = [255] * 32
        self.assertEqual(len(FixedPoseAssociation().apply(fixed, CAMERA)[0]["points"]), 2)

    def test_two_observed_features_in_same_frame_remain_distinct(self):
        native = snapshot(xyz=[[.1, .2, 5.], [.101, .2, 5.]])
        for p in native["points"]: p["descriptor"] = [0] * 32
        fixed, _, _ = confirm_landmarks(native, CAMERA)
        self.assertEqual(len(FixedPoseAssociation().apply(fixed, CAMERA)[0]["points"]), 2)

    def test_ambiguous_repeating_patterns_remain_separate(self):
        native = snapshot(centers=tuple(i * .3 for i in range(9)), xyz=[[.1, .2, 5.]] * 3)
        for p in native["points"]: p["descriptor"] = [0] * 32
        for i, frame in enumerate(native["keyframes"]):
            frame["observations"] = [o for o in frame["observations"] if o["point_id"] == 50 + i // 3]
        fixed, _, _ = confirm_landmarks(native, CAMERA)
        self.assertEqual(len(FixedPoseAssociation().apply(fixed, CAMERA)[0]["points"]), 3)

    def test_alias_is_rechecked_and_other_maps_do_not_inherit_it(self):
        association = FixedPoseAssociation()
        fixed, _, _ = confirm_landmarks(split_views(), CAMERA)
        association.apply(fixed, CAMERA)
        self.assertEqual(association.aliases[51], 50)
        simultaneous = snapshot(xyz=[[.1, .2, 5.], [.1, .2, 5.]])
        for p in simultaneous["points"]: p["descriptor"] = [0] * 32
        fixed, _, _ = confirm_landmarks(simultaneous, CAMERA)
        merged, _ = association.apply(fixed, CAMERA)
        self.assertEqual({p["id"] for p in merged["points"]}, {50, 51})
        association.apply(confirm_landmarks(split_views(), CAMERA)[0], CAMERA)
        other = deepcopy(fixed)
        other["map_id"] += 1
        association.apply(other, CAMERA)
        self.assertEqual(association.aliases[51], 51)

    def test_publication_records_aliases_with_unmodified_camera_trajectory(self):
        native = split_views()
        with tempfile.TemporaryDirectory() as directory:
            # Missing RGB changes only fallback color, never the association.
            value = convert(native, CAMERA, {}, Path(directory), FixedPoseAssociation())
        self.assertEqual(value["point_aliases"], {"51": 50})
        self.assertEqual(value["point_observations"]["frame_index_by_point"], {"50": [0, 1, 2, 3, 4, 5]})
        for c, original in zip(value["cameras"], native["keyframes"]):
            self.assertEqual(c["cam_from_world_translation_raw"], [r[3] for r in original["T_cw"][:3]])
            self.assertEqual(c["cam_from_world_rotation_raw"], [r[:3] for r in original["T_cw"][:3]])


if __name__ == "__main__":
    unittest.main()
