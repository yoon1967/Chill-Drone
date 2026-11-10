"""Exact-timestamp camera poses and image-space depth anchors from CUDA trackers."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import platform
import sys

import cv2
import numpy as np
import torch


def reply(**value):
    print('POSE_RESULT ' + json.dumps(value, allow_nan=False), file=sys.__stdout__, flush=True)


def frames(config):
    width, height = config['size']
    fx, fy, cx, cy = config['intrinsics']
    distortion = config['distortion']
    rectification = None
    if any(distortion):
        camera = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
        rectification = cv2.initUndistortRectifyMap(camera, np.asarray(distortion), None, camera,
                                                   (width, height), cv2.CV_16SC2)
    while header := sys.stdin.buffer.readline(128):
        length, stamp = header.split()
        jpeg = sys.stdin.buffer.read(int(length))
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise ValueError('pose_frame_geometry_mismatch')
        if rectification is not None:
            image = cv2.remap(image, *rectification, cv2.INTER_LINEAR)
        yield float(stamp), image


def anchors(pixels, depth):
    values = np.column_stack((pixels, depth))
    return values[np.isfinite(values).all(axis=1) & (values[:, 2] > 0)].tolist()


def pose_frame(stamp, matrix=None, depth_anchors=()):
    if matrix is None or not np.isfinite(matrix).all():
        reply(timestamp_s=stamp, tracking_state=1, map_id=0)
    else:
        reply(timestamp_s=stamp, tracking_state=2, map_id=0, T_cw=matrix.tolist(),
              depth_anchors=depth_anchors)


def dpvo_frame_anchors(tracker, index):
    from dpvo.lietorch import SE3
    from dpvo.projective_ops import iproj

    pg = tracker.pg
    # The newest patches have little baseline. Use well-tracked older patches
    # that the graph associates with the current frame.
    supported = (pg.jj == index) & (pg.ii != index) & (pg.weight[0] > .25).all(dim=-1)
    ii, kk = pg.ii[supported], pg.kk[supported]
    if not len(kk):
        return []
    poses = SE3(tracker.poses)
    relative = poses[:, index:index + 1] * poses[:, ii].inv()
    lifted = iproj(tracker.patches[:, kk], tracker.intrinsics[:, ii])
    points = (relative[:, :, None, None] * lifted)[0, :, tracker.P // 2, tracker.P // 2]
    points = points[(points[:, 2] > .2) & (points[:, 3] > 0)]
    intrinsics = pg.intrinsics_[index]
    pixels = (points[:, :2] / points[:, 2:3] * intrinsics[:2] + intrinsics[2:]) * tracker.RES
    return anchors(pixels.cpu().numpy(), (points[:, 2] / points[:, 3]).cpu().numpy())


def run_dpvo(config):
    from dpvo.config import cfg
    from dpvo.dpvo import DPVO
    from dpvo.lietorch import SE3
    from dpvo.net import VONet

    if 'microsoft' in platform.release().lower():
        # The CUDA LieTorch pose transforms may hang on WSL2.
        from dpvo.lietorch.group_ops import GroupOp

        @classmethod
        def forward_on_cpu(cls, _ctx, group_id, *inputs):
            return cls.forward_op(group_id, *(value.cpu() for value in inputs)).to(inputs[0].device)

        GroupOp.forward = forward_on_cpu

    mode = config['model']
    hybrid = mode == 'orb-slam3-dpvo'
    cfg = cfg.clone()
    cfg.LOOP_CLOSURE = mode in {'dpv-slam', 'dpv-slam-plus-plus'}
    cfg.CLASSIC_LOOP_CLOSURE = mode == 'dpv-slam-plus-plus'
    width, height = config['size']
    width, height = width - width % 16, height - height % 16
    intrinsics = torch.tensor(config['intrinsics'], dtype=torch.float32, device='cuda')
    network = VONet()
    weights = torch.load('/opt/dpvo/dpvo.pth', map_location='cpu', weights_only=True)
    network.load_state_dict({key.replace('module.', '', 1): value for key, value in weights.items()
                             if 'update.lmbda' not in key})
    tracker = DPVO(cfg, network, ht=height, wd=width, viz=False)
    if mode == 'dpv-slam-plus-plus' and not tracker.cfg.CLASSIC_LOOP_CLOSURE:
        raise RuntimeError('classic_loop_closure_unavailable')
    accepted = []
    reply(ready=True)
    for stamp, image in frames(config):
        image = torch.from_numpy(np.ascontiguousarray(image[:height, :width])).permute(2, 0, 1).cuda()
        diagnostics = io.StringIO()
        with redirect_stdout(diagnostics):
            tracker(stamp, image, intrinsics)
        index = tracker.n - 1
        reliable = False
        if (tracker.is_initialized and tracker.pg.tstamps_[index] == tracker.counter - 1
                and 'Warning BA failed' not in diagnostics.getvalue()):
            pg = tracker.pg
            linked = (pg.ii == index) | (pg.jj == index)
            reliable = int((pg.weight[:, linked] > .25).all(dim=-1).sum()) >= 12
        if not hybrid:
            accepted.append(reliable)
        if not reliable:
            pose_frame(stamp)
            continue
        pose = SE3(tracker.pg.poses_[index:index + 1]).matrix()[0].cpu().numpy()
        pose_frame(stamp, pose, () if hybrid else dpvo_frame_anchors(tracker, index))

    if hybrid:
        return
    if not tracker.is_initialized:
        reply(finished=True)
        return

    # DPVO's final trajectory uses camera-to-world poses. The retained graph's
    # patch depths have the same optimized scale, including loop corrections.
    trajectory, stamps = tracker.terminate()
    pg = tracker.pg
    patches = pg.patches_[:tracker.n, :, :, tracker.P // 2, tracker.P // 2].cpu().numpy()
    supported = np.zeros((tracker.n, tracker.M), dtype=bool)
    for ii, jj, kk, weight in ((pg.ii, pg.jj, pg.kk, pg.weight),
                               (pg.ii_inac, pg.jj_inac, pg.kk_inac, pg.weight_inac)):
        good = (ii != jj) & (weight[0] > .25).all(dim=-1)
        supported[ii[good].cpu().numpy(), (kk[good] % tracker.M).cpu().numpy()] = True
    retained = {}
    for index, frame_id in enumerate(pg.tstamps_[:tracker.n]):
        values = patches[index, supported[index] & (patches[index, :, 2] > 0)]
        retained[frame_id] = anchors(values[:, :2] * tracker.RES, 1. / values[:, 2])
    transforms = SE3(torch.from_numpy(trajectory)).inv().matrix().numpy()
    for i, (stamp, matrix) in enumerate(zip(stamps, transforms)):
        pose_frame(float(stamp), matrix if accepted[i] else None, retained.get(i, ()))
    reply(finished=True)


def vista_input(image):
    """Use the upstream 224px center crop and keep its inverse pixel transform."""
    from PIL import Image

    width, height = image.shape[1], image.shape[0]
    cropped = np.array([width - 20, height - 20])
    resized = np.floor(cropped * (max(224 / cropped) + 1e-8)).astype(int)
    x, y = np.rint((resized - 224) / 2).astype(int)
    image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)).crop((10, 10, width - 10, height - 10))
    image = image.resize(tuple(map(int, resized)), resample=Image.Resampling.LANCZOS).crop((x, y, x + 224, y + 224))
    rgb = np.asarray(image).copy()
    scale = cropped / resized
    offset = (np.array([x, y]) + .5) * scale + 9.5
    return rgb, scale, offset


def vista_pose(slam, index, stamp, scale, offset):
    view = slam.get_view(index, return_intri=False)
    depth = view.depth.numpy()[4::12, 4::12]
    v, u = np.mgrid[4:224:12, 4:224:12]
    pixels = np.column_stack((u.ravel(), v.ravel())) * scale + offset
    pose_frame(stamp, np.linalg.inv(view.pose.numpy()), anchors(pixels, depth.ravel()))


def run_vista(config):
    from vista_slam.slam import OnlineSLAM

    assets = Path('/pose-model')
    slam = OnlineSLAM(ckpt_path=str(assets / 'frontend_sta_weights.pth'), vocab_path=str(assets / 'ORBvoc.txt'),
        max_view_num=4096, neighbor_edge_num=3, loop_edge_num=2, pgo_every=35, verbose=True)
    stamps = []
    reply(ready=True)
    for stamp, image in frames(config):
        if len(stamps) >= slam.max_view_num:
            raise ValueError('pose_frame_limit')
        rgb, scale, offset = vista_input(image)
        rgb = torch.from_numpy(rgb).permute(2, 0, 1)
        gray = (rgb.float() * torch.tensor([.2989, .587, .114])[:, None, None]).sum(dim=0)
        view = dict(rgb=(rgb[None].to(slam.device, dtype=torch.float32) / 127.5 - 1.),
                    shape=torch.tensor([[224, 224]]), gray=gray.to(torch.uint8).numpy(), view_name=str(len(stamps)))
        slam.step(view)
        stamps.append(stamp)
        if len(stamps) >= 2:
            vista_pose(slam, len(stamps) - 1, stamp, scale, offset)
        else:
            pose_frame(stamp)
    if len(stamps) >= 2:
        if len(stamps) % slam.pgo_every:
            slam.pose_graph_optimize()
        for i, stamp in enumerate(stamps):
            vista_pose(slam, i, stamp, scale, offset)
    reply(finished=True)


def main():
    config = json.loads(sys.argv[1])
    cv2.setNumThreads(1)
    torch.set_num_threads(2)
    torch.manual_seed(43)
    np.random.seed(43)
    try:
        with redirect_stdout(sys.stderr):
            if config['model'] == 'vista-slam':
                run_vista(config)
            else:
                with torch.no_grad():
                    run_dpvo(config)
    except Exception as error:
        reply(error=f'{type(error).__name__}: {error}'[:300])
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
