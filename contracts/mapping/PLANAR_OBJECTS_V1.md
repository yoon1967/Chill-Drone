# 관측 영상 기반 평면 객체 계약 1.0

생산자: `modules/mapping/localization/planar.py`, `online.py`. 소비자: `apps/console/`.
오프라인은 `objects.json` + `textures/`, 온라인은 `scene.planar_objects` + revision의 `object_textures/`다.
원본 사진·텍스처·실제 객체 좌표는 로컬 ignore 산출물이며 이 계약 문서에 포함하지 않는다.

- `schema_version`: `planar-objects/1.0`.
- `scene_binding_sha256`: points/cameras/scale을 정렬 키·compact JSON·allow_nan=false로 직렬화한 SHA-256.
  지도 좌표·원본 관측 또는 스케일이 다른 묶음은 소비자가 거부한다.
- `unit`, `segment_id`: 해당 지도의 단위와 좌표 segment. 임의 단위를 미터로 표시하지 않는다.
- `method`, `classification`: 처리 방법과 기하 후보임을 명시한다. 의미 분류 확률이 아니다.
- `statistics`: 객체/평면/분석 프레임/영상 사각형/지지 후보 수와 처리 시간. 관측 부족이면 objects=0과 reason.
- 선택 필드 `detection`: profile과 실제 임계값. `statistics.shapes`, `local_planes`, `rejected`는 모양별 수,
  국소 평면 시도 수와 단계별 탈락 수다. 탈락 수는 동일 영상 경계의 복수 평면 가설을 포함하며 고유 사물 개수가 아니다.
  `consensus_proposals`는 여러 영상을 평면에 정렬해 추가한 경계 제안 수, `plane_groups`는 채택 면의 공면 그룹 수,
  `object_limit_reached`는 처리 객체 수 상한에 도달했는지 표시한다.
- `id_counter`: 같은 segment에서 사용한 객체 ID의 상한. `object_id`는 경계 겹침·평면·크기 대응으로 유지한다.
  온라인 증분 지도는 같은 segment 안에서 미관측 면도 유지한다. segment 간 재식별·루프 폐쇄는 보장하지 않는다.
  `plane_id`는 현재 묶음의 평면 그룹이며 영구 공면 식별자로 조인하면 안 된다.
- 온라인 선택 통계 `update_mode=incremental_keyframe`, `new_objects`, `retained_objects`, `textures_written`는
  새 면/유지한 면/새 JPEG 수다. `fresh_frames`, `retried_frames`, `pending_frames`는 새 등록/재시도/대기 시점 수다.
  `reused_boundaries`는 이미 확인한 면의 경계를 건너뛴 횟수, `new_landmarks`/`reused_landmarks`는
  현재 native 점 중 확인된 유한 면 밖/안의 점 수다. 영구 landmark 생성·삭제 수가 아니다.

## 객체 필드

| 필드 | 의미 |
| --- | --- |
| `object_id`, `plane_id`, `label` | 객체 ID, 현재 평면 그룹, 문 형태/사각 평면이라는 기하 표시 |
| `shape` (선택) | vertical / horizontal / compact. 평면 가로·세로 비율로 분류하며 실제 중력 방향이나 사물 종류가 아님 |
| `plane_orientation` (선택) | reference / nonparallel. 가장 많은 지지가 있는 대표 평면과 법선 차이가 30° 이상인지 표시. 윗면 판정이 아님 |
| `boundary_source` (선택) | image_lines_or_contours / multiview_plane_consensus. 원본 영상 또는 다중 시점 평면 경계 합의에서 제안됐는지 표시 |
| `corners` | 표시 좌표의 경계 꼭짓점 4개. 같은 좌표계의 평면에서 순환 순서 |
| `normal`, `offset` | 정규화된 n과 d. 평면식 n·X+d=0, 법선은 선택한 관측 카메라 방향 |
| `width`, `height`, `area` | 대응 변 길이의 평균과 곱. unit과 일관되며 실측 정확도 미검증 |
| `support_count`, `support_points` | 현재 원본 관측의 지지점 수와 표시 좌표. 저장 목록은 최대 160점 |
| `support_point_ids` | 같은 native SfM 모델의 landmark ID. 온라인 voxel 대표점 ID와 조인하면 안 됨 |
| `plane_rms`, `plane_threshold` | 지지점의 평면 거리 RMS, 관측 조건에서 산정한 허용 거리. unit과 같음 |
| `coverage` | 분석 프레임에서 지지점 hull / 사각 영역 면적 |
| `geometric_score` | 0~1 지수. 지지 수(0.4), 다중 시점 수(0.3), 분포 면적(0.3)을 각각 포화·합산. 분류 확률 아님 |
| `source_frame_id` | 텍스처를 가져온 등록 프레임. 절대 경로는 노출하지 않음 |
| `source_quad`, `source_analysis_size` | 왜곡 보정·축소된 분석 영상의 사각 픽셀 경계와 [폭,높이] |
| `evidence` | 프레임별 경계 일치율과 경계 거리 중앙값. 선택 min_edge_agreement는 네 변 중 최소 일치율. border_appearance_correlation은 정사 보정 경계 띠 밝기의 상관도이며 무늬가 부족하면 null. 거리 단위는 최대 폭 960px 분석 영상의 픽셀 |
| `texture_file` | `object-숫자.jpg`. 단일 사각형의 실제 관측 RGB를 정사 보정한 텍스처 |
| `mapping_status` (선택) | new / retained. 이번 갱신에서 추가했는지 이전 면을 유지했는지 표시 |
| `observation_revision`, `created_timestamp_s` (선택) | 지지 native ID를 얻은 revision과 원본 시점 시각. 유지한 면의 support_point_ids를 현재 native 모델과 조인하면 안 됨 |
| `inference` | 경계 내부는 평면 가정으로 완성한 영역임을 명시 |

원본 미리보기의 경계는 표시 좌표 corners를 해당 프레임의 원본 카메라 모델로 재투영한다.
source_quad를 원본 이미지 픽셀로 직접 해석하면 안 된다. 텍스처는 공간 좌표에 고정된 앞면의 외관이며,
유리 뒤 선반·반사상·문 두께·사물함 전체 부피는 측정하지 않는다. 미관측 영역을 자유 공간으로 판정하지 않는다.
