# 지도 계약

병렬 구현의 초기 기본값은 [PoC 연결 초안 0.1.0](../POC_V0_1.md)의 영상·2D 관측·복원 결과를 사용한다.

실제 입력·2D 관측·좌표/시각·스케일·지도/카메라 경로 결과의 합의 문서와 공개 가능한 작은 예제를 둔다. 계약 버전·생산자/소비자·오류 예제를 포함한다.

기존 휴대폰 실험의 확정 수치 입력은 [phone snapshot 1.0](PHONE_INPUT_SNAPSHOT_V1.md)으로 보존한다.
이는 공통 PoC 형식과 다르며 변환 전 서로 같은 입력으로 처리하지 않는다.

## 단일 영상 파일 처리 계약

아래 계약은 기존 snapshot 및 공통 PoC와 구분한다. 각 파일 내부의 버전과 필수 필드를 따른다.

| 계약 | 생산자 → 소비자 | 핵심 내용 |
| --- | --- | --- |
| [PHONE_VIDEO_INPUT_V1](PHONE_VIDEO_INPUT_V1.md) | video_input → full_features/reconstruct | 프레임·원본 PTS·회전/축소·해시 |
| [PHONE_FEATURE_RESULT_V1](PHONE_FEATURE_RESULT_V1.md) | full_features → reconstruct | DB·2D CSV·매칭·검사 |
| [PHONE_SFM_RESULT_V1](PHONE_SFM_RESULT_V1.md) | reconstruct → result_io/console | 희소 점·카메라·complete/partial·단위/변환 |
| [PHONE_PREVIEW_V1](PHONE_PREVIEW_V1.md) | render_preview → 사용자 | 저장 결과 대조·PNG·출처 해시 |
| [PHONE_VIDEO_PIPELINE_V1](PHONE_VIDEO_PIPELINE_V1.md) | run_video_pipeline → 호출자 | 전체 상태·종료 코드·시간·산출물 상대 경로 |

설치·단일 명령·설정·검사는 [실행 안내](../../docs/phone_mapping/video_pipeline.md),
실제 검사 결과와 미검증 범위는 [검증 요약](../../docs/phone_mapping/validation_summary.md)을 따른다.
팀 작업 절차는 저장소 루트의 docs/team/START_HERE.md를 따른다.
