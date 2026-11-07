// 실시간 감독 화면 프리셋: 어떤 칸(패널)을 격자 어디에 얼마나 크게 둘지.
// - grid: [가로 칸 수, 세로 칸 수]. 화면 높이를 이 격자가 채운다(스크롤 없음). 상태 줄·알림·조종 줄은 격자 밖에 고정.
// - panels: 중요한 칸부터 적는다. 좁은 화면에서는 이 순서대로 위에서부터 쌓는다.
//   id는 app.js의 PANELS에 있는 칸(video·findings·map·timeline), x·y는 0부터 시작하는 왼쪽 위 칸, w·h는 차지하는 칸 수.
//   지도 칸은 mode("2d" 자율 지도 / "3d" 점군)로 프리셋을 고를 때 처음 보기를 정할 수 있다(없으면 쓰던 보기 그대로).
// - 첫 번째가 기본 프리셋. 주소에 ?preset=<id>를 붙이면 그 프리셋으로 연다.
"use strict";

window.PRESETS = [
  {
    id: "rescue", name: "재난 정찰", grid: [12, 6],
    note: "영상을 크게 보며 사람을 찾는다. 발견 목록과 지도를 옆에",
    panels: [
      { id: "video", x: 0, y: 0, w: 7, h: 6 },
      { id: "findings", x: 7, y: 0, w: 5, h: 3 },
      { id: "map", x: 7, y: 3, w: 5, h: 3 },
    ],
  },
  {
    id: "autonomy", name: "자율 탐색", grid: [12, 6],
    note: "자율 비행이 그리는 위치·경로·찾은 사람을 크게. 모드 바뀜·자율 연결은 비행 기록에",
    panels: [
      { id: "map", x: 0, y: 0, w: 7, h: 4, mode: "2d" },
      { id: "video", x: 7, y: 0, w: 5, h: 3 },
      { id: "findings", x: 7, y: 3, w: 5, h: 3 },
      { id: "timeline", x: 0, y: 4, w: 7, h: 2 },
    ],
  },
  {
    id: "mapping", name: "지도 만들기", grid: [12, 6],
    note: "3D 점군을 크게. 지도 콘솔 진행 상황은 지도 칸의 '지도 콘솔 ↗'",
    panels: [
      { id: "map", x: 0, y: 0, w: 8, h: 6, mode: "3d" },
      { id: "video", x: 8, y: 0, w: 4, h: 3 },
      { id: "findings", x: 8, y: 3, w: 4, h: 3 },
    ],
  },
  {
    id: "review", name: "기록 검토", grid: [12, 6],
    note: "기록 재생(--replay)으로 지난 비행을 다시 볼 때. 영상과 비행 기록을 나란히",
    panels: [
      { id: "video", x: 0, y: 0, w: 7, h: 4 },
      { id: "findings", x: 7, y: 0, w: 5, h: 3 },
      { id: "timeline", x: 0, y: 4, w: 7, h: 2 },
      { id: "map", x: 7, y: 3, w: 5, h: 3 },
    ],
  },
];
