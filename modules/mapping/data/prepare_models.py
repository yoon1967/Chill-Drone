"""Explicit model setup; ordinary mapping only reads prepared local weights."""
import argparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download', action='store_true', help='Allow first-use model downloads while internet is available')
    args = parser.parse_args()
    from .model_assets import require_local_models
    if args.download:
        from .stream_mapping import DepthEstimator
        from .rescue_mapping import SemanticEstimator
        print('Preparing models. Keep internet connected until this command finishes.', flush=True)
        depth = DepthEstimator(allow_download=True)
        depth.prepare()
        semantic = SemanticEstimator(allow_download=True)
    for name, path in require_local_models().items():
        print(f'{name}: {path}', flush=True)
    print('Model files are ready on disk. Drone mapping does not download models.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError) as error:
        raise SystemExit(str(error))
