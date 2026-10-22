# 2층 1: 확정 2D 입력 snapshot

`run_manifest.json`이 활성 파일·버전·해시의 기준이다. `features_v1/`은 자동 좌표/매칭,
`measurements_v2_floor/`는 수동 기준점이다. 기존 파일은 전달 ZIP과 바이트가 같다.

Git 전용 검사: `python -B scripts/phone_mapping/shared_inputs.py`

외부 자료 연결: `python -B scripts/phone_mapping/shared_inputs.py --package-root "압축을 푼 패키지 루트"`

패키지 루트는 `handoff_manifest.json`이 있는 위치다. 저장소나 자료 폴더를 다른 위치로 옮겨도
`--run-dir`, `--package-root`를 지정하면 된다. Python API는 `shared_inputs.load_run()`을 사용한다.
생산자 JSON 안의 옛 상대 경로는 실행 당시 출처다. 현재 폴더에 직접 이어 붙이지 않는다.

31장, 자동 좌표 178,584개, 매칭 59,175개(기하 검증 52,159개), 수동 관측 15개다.
가로 A–B=0.44m는 스케일 시험용이다. 세로 A–C=1.36m의 C는 바닥 접점이며 동일점 대응 미확인으로
사용이 비활성화되어 있다. 실제 3D 복원·치수 정확도·수신자 환경은 검증하지 않았다.

외부 ZIP은 이미 사용자에게서 김수호에게 전달되었다. Drive 폴더 ID는 제공되지 않아 null이다.
`external_artifacts.json`의 이름·SHA-256으로 해당 ZIP을 확인한다. 자동 다운로드는 하지 않는다.
