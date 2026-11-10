"""Stream exact-frame DPVO camera poses from the pinned upstream GPU runtime."""

from contextlib import redirect_stdout
import io
import json
import platform
import sys


def run(source, reply, calibration):
    import cv2
    import numpy as np
    import torch
    from dpvo.config import cfg
    from dpvo.dpvo import DPVO
    from dpvo.lietorch import SE3
    from dpvo.net import VONet

    if 'microsoft' in platform.release().lower():
        # LieTorch's CUDA pose kernels can stall under WSL2. In this inference
        # process the same group operations can run on CPU without gradients.
        from dpvo.lietorch.group_ops import GroupOp

        @classmethod
        def forward_on_cpu(cls, _ctx, group_id, *inputs):
            return cls.forward_op(group_id, *(value.cpu() for value in inputs)).to(inputs[0].device)

        GroupOp.forward = forward_on_cpu

    width, height = calibration['size']
    fx, fy, cx, cy = calibration['intrinsics']
    distortion = calibration['distortion']
    rectification = None
    if any(distortion):
        camera = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
        rectification = cv2.initUndistortRectifyMap(camera, np.asarray(distortion), None, camera,
                                                   (width, height), cv2.CV_16SC2)
    cropped_width, cropped_height = width - width % 16, height - height % 16
    intrinsics = torch.tensor([fx, fy, cx, cy], dtype=torch.float32, device='cuda')
    cfg = cfg.clone()
    cfg.LOOP_CLOSURE = cfg.CLASSIC_LOOP_CLOSURE = False
    weights = torch.load('/opt/dpvo/dpvo.pth', map_location='cpu', weights_only=True)
    network = VONet()
    network.load_state_dict({key.replace('module.', '', 1): value for key, value in weights.items()
                             if 'update.lmbda' not in key})
    tracker = DPVO(cfg, network, ht=cropped_height, wd=cropped_width, viz=False)
    reply({'ready': True})

    with torch.no_grad():
        while header := source.readline(128):
            length, stamp = header.split()
            jpeg = source.read(int(length))
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (height, width):
                raise ValueError('dpvo_frame_geometry_mismatch')
            if rectification is not None:
                image = cv2.remap(image, *rectification, cv2.INTER_LINEAR)
            image = np.ascontiguousarray(image[:cropped_height, :cropped_width])
            rgb_input = torch.from_numpy(image).permute(2, 0, 1).cuda()  # Upstream uses OpenCV BGR.
            stamp = float(stamp)
            diagnostics = io.StringIO()
            with redirect_stdout(diagnostics):
                tracker(stamp, rgb_input, intrinsics)

            result = {'timestamp_s': stamp, 'tracking_state': 1}
            index = tracker.n - 1
            if (tracker.is_initialized and tracker.pg.tstamps_[index] == tracker.counter - 1 and
                    'Warning BA failed' not in diagnostics.getvalue()):
                current = (tracker.pg.ii == index) | (tracker.pg.jj == index)
                weights = tracker.pg.weight[:, current]
                if int((weights > .25).all(dim=-1).sum()) >= 12:
                    pose = SE3(tracker.pg.poses_[index:index + 1]).matrix()[0].cpu().numpy()
                    if np.isfinite(pose).all():
                        result.update(tracking_state=2, T_cw=pose.tolist())
            reply(result)


def main():
    output = sys.stdout

    def reply(value):
        output.write(json.dumps(value, allow_nan=False) + '\n')
        output.flush()

    try:
        with redirect_stdout(sys.stderr):
            run(sys.stdin.buffer, reply, json.loads(sys.argv[1]))
    except Exception as error:
        reply({'error': f'{type(error).__name__}: {error}'[:160]})
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
