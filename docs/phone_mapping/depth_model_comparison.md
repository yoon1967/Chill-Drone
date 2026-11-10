# 깊이 모델 비교 벤치마크

같은 영상의 동일한 5FPS 프레임에 세 깊이 모델을 적용하고, ORB-SLAM3 자세는 영상마다 한 번 계산해 재사용한다. 표의 가중치 크기는 로컬에서 읽은 파일 크기이며, GPU 메모리는 PyTorch가 할당한 최대값이다.

| 모델 | 파라미터 | 가중치 파일 | 프레임별 실행의 최대 CUDA 할당 |
| --- | ---: | ---: | ---: |
| Depth Anything V2 Metric Indoor Small | 24.8M | 99.2MB | 95MiB |
| Metric Video Depth Anything Small | 28.4M | 116.4MB | 382MiB |
| MoGe-2 Small | 35M | 140.6MB | 484MiB |

## 두 비교 방식

- **저장 영상 비교**는 Metric Video Depth Anything의 32프레임 창과 10프레임 겹침을 사용한다. V2와 MoGe-2는 한 장씩 깊이를 추정한다. ORB가 확인한 자세와 희소 깊이로 배율을 연결하고, 좌표계가 다른 구간은 PLY를 분리한다.
- **프레임별 비교**는 깊이 한 장이 나오면 같은 프레임의 카메라 자세와 배율을 적용해 복셀에 바로 합친다. Metric Video Depth Anything은 공식 스트리밍 API를 호출해 과거 특징을 유지한다. V2가 ORB 경로의 좌표계를 연결하며, 다른 두 모델은 저장된 공통 경로를 읽는다. 영상 특징으로 연결하기 어려운 프레임은 예측하거나 직전 자세를 유지하며, 각 연결 방법을 별도로 기록한다.
- 두 방식 모두 긴 변 384픽셀의 공통 RGB, 0.2–8.0m 복셀 입력 범위와 점 32,000개 예산을 적용한다. 원본 깊이 NPZ에는 표시 범위 밖의 양수 추정값도 저장한다. 미터 깊이는 모델 추정치이고 PLY의 좌표는 공통 단안 지도 단위다.

## 실행

프로젝트 Python 환경과 CUDA를 사용한다. ORB-SLAM3의 기존 Docker 이미지가 준비된 상태에서 실행한다. V2는 프로젝트가 쓰는 Hugging Face 체크포인트를 사용한다. 다른 두 모델은 아래 공식 저장소를 준비하고, 가중치도 Hugging Face 캐시에 받는다. 모델과 코드 리비전은 [측정 기록](../../data/mapping/benchmarks/2026-11-10-depth-model-comparison.json)에 적었다. MoGe-2의 경량 실행에는 해당 리비전의 [공식 의존성](https://github.com/microsoft/MoGe/blob/0744441/requirements.txt)을 사용한다.

- [Depth Anything V2 가중치](https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf)
- [Video Depth Anything 코드](https://github.com/DepthAnything/Video-Depth-Anything) · [Metric Small 가중치](https://huggingface.co/depth-anything/Metric-Video-Depth-Anything-Small)
- [MoGe 코드](https://github.com/microsoft/MoGe) · [MoGe-2 Small 가중치](https://huggingface.co/Ruicheng/moge-2-vits-normal)

프로젝트 루트에서 아래 명령을 실행한다. 영상에 대응하는 카메라 보정 JSON이 있으면 prepare에 --camera를 추가한다. 앞서 생성한 source.json과 orbslam3_poses.npz가 있다면 그 폴더를 재사용할 수 있다.

~~~powershell
$video = 'data/real_data/03_movie_source/source_video.mp4'
$poses = 'data/outputs/depth-bench/clip-3/poses'
$vda = '<Video-Depth-Anything 저장소 경로>'
$moge = '<MoGe 저장소 경로>'

python -B -m modules.mapping.benchmarks.compare_depth_models prepare --video $video --output $poses

foreach ($mode in @('batch', 'framewise')) {
  $output = "data/outputs/depth-bench/clip-3/$mode"
  foreach ($model in @('depth-anything-v2', 'metric-video-depth-anything', 'moge-2')) {
    python -B -m modules.mapping.benchmarks.compare_depth_models run --video $video --poses $poses --output $output --model $model --mode $mode --vda-source $vda --moge-source $moge
    if ($LASTEXITCODE -ne 0) { throw "Benchmark failed for $mode / $model" }
  }
}
~~~

프레임별 실행에서는 V2를 먼저 실행해 common_trajectory.npz를 생성한다. 같은 영상과 모드에서 세 모델의 --output은 공통으로 지정한다. 첫 N초를 실제 영상 시각대로 입력하려면 프레임별 run에 --paced-seconds N을 붙인다.

각 모델 폴더에 depth_m.npz, preview.mp4, result.json과 PLY가 생성된다. 프레임별 실행은 unified-cloud.ply와 frame_progress.npz를 생성한다. 프레임별 기록에 깊이, 자세·배율 적용, 복셀 갱신 시간과 점 수가 들어 있다. 팀 공유용 기록에는 수치만 담고 입력 영상과 렌더링 산출물은 로컬에서 관리한다.

## 기존 실기체 영상 결과

RTX 4050 Laptop 6GiB에서 세 영상의 배치·프레임별 조합 18개를 비교했다. 프레임별 실행은 371개, 956개, 139개의 입력을 모델마다 모두 누적해 조합당 PLY 하나를 생성했다. 검증에서 세 모델의 프레임 번호, 시각과 공통 카메라 경로가 일치했다.

두 번째 영상의 첫 8초를 5FPS 시각대로 공급한 40프레임에서 입력 예정 시각부터 복셀 갱신까지 측정했다.

| 모델 | 복셀 갱신 지연 P95 | 200ms 초과 |
| --- | ---: | ---: |
| Depth Anything V2 | 90.9ms | 0/40 |
| Metric Video Depth Anything | 129.1ms | 0/40 |
| MoGe-2 Small | 179.8ms | 2/40 |

측정은 저장된 ORB-SLAM3 기록을 사용했다. 실시간 카메라 수신과 새 ORB 추적, 화면 표시를 포함한 전체 지연은 별도 검증이 필요하다. ORB 초기 정합과 좌표계 변경을 연결한 여섯 번 중 네 번은 직전 자세를 기준으로 삼았으므로 연결부에 위치 오차가 남을 수 있다. 정확한 거리 오차는 실측 기준으로 확인한다.
