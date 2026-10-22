# 지도 입력·데이터·전달

촬영/프레임·2D 특징점/매칭·실측 입력, 점군 처리·회차별 저장·재생·미리보기·업로드 코드를 둔다. 문의: 박수경. 원본 영상·생성물은 data의 비공개 경로에 둔다.

## 휴대폰 영상의 2D 좌표 인계 시험

- 코드: `scripts/phone_mapping/` (팀 폴더 개편 전에 작성한 기존 실행 경로 유지).
- 설치·실행·로컬 데이터 준비: [실행 안내](../../../docs/phone_mapping/README.md).
- 입출력·좌표 규칙·품질 한계: [좌표 인계 계획](../../../docs/phone_mapping/coordinate_handoff_plan.md).
- 생성 환경: Windows, Python 3.12.14, PyCOLMAP 4.2.1. 세부 패키지는 `scripts/phone_mapping/requirements-features.txt`로 고정한다.
- 자동 SIFT 특징점·여러 프레임의 대응·수동 실측 기준점을 전달한다. 실제 3D 복원과 수신자 환경 확인은 후속 단계다.
- 확정 JSON/CSV·설정·검증 자료는 `data/mapping/phone_video/2f1_20261021/`에 공유한다. 원본 프레임·DB·ZIP과 중간 결과는 로컬/Drive에 보관한다.
- `python -B scripts/phone_mapping/shared_inputs.py`로 Git 자료를 검사한다. `--package-root`로 어느 위치의 전달 패키지든 연결하고 `--workspace`로 새 작업 사본을 만들 수 있다.
- 파트 검사: `python -B -m unittest discover -s modules/mapping/data/tests -v`.

팀 작업 절차는 저장소 루트의 docs/team/START_HERE.md를 따른다.
