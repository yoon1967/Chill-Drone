// 서버 연결 전 화면 구성용 샘플. 실제 측정값이 아니다.
// 형식: drone_state·mission은 contracts/api/DRONE_CONTROL_V0_1.md(초안),
// 탐지 이벤트는 contracts/POC_V0_1.md 4장, pose·path는 AUTONOMY_BRIDGE_V0_1.md 4장을 따른다.
"use strict";

window.SAMPLE = (() => {
  const RUN = "sample-run";
  const base = {
    schema_version: "0.1.0", type: "drone_state", time_source: "gateway_receive", source: "fake",
    connected: true, state_age_s: 0.08, battery_pct: 66, height_m: 0.95, height_source: "down_tof",
    temp_high_c: 71, attitude: { roll_rad: 0.01, pitch_rad: -0.02, yaw_rad: 1.2 },
    front_tof: { m: 0.84, valid: true, reason: null, age_s: 0.12 },
    down_tof: { m: 0.95, valid: true, reason: null },
    video: { available: false, fps: null, frame_age_s: null },
    link: { level: "good", state_hz: 9.9, rtt_ms: 21, snr: null },
    flight_time_s: 312, operator_interventions: 0, autonomy_used: true,
  };
  const mission = (state, phase, progress, reason = null) => ({ state, phase, goal: null, progress, reason });

  const scenarios = {
    auto: {
      label: "자율 탐색 중",
      state: { ...base, mode: "auto", authority: "autonomy", allowed: ["emergency", "land", "pause", "takeover", "auto_stop"],
        warnings: [], mission: mission("FOLLOW", "running", 0.52) },
      detections: 2,
    },
    person: {
      label: "사람 발견 직후",
      state: { ...base, battery_pct: 62, mode: "auto", authority: "autonomy",
        allowed: ["emergency", "land", "pause", "takeover", "auto_stop"], warnings: [], mission: mission("LOOK", "running", 0.58) },
      detections: 3, alert: "person",
    },
    link: {
      label: "통신 약해짐",
      state: { ...base, battery_pct: 55, mode: "paused", authority: "none",
        allowed: ["emergency", "land", "takeover", "auto_stop", "resume"], warnings: ["link_weak"],
        link: { level: "weak", state_hz: 5.1, rtt_ms: 340, snr: null }, mission: mission("FOLLOW", "paused", 0.61, "link_weak") },
      detections: 3, alert: "link",
    },
    stale: {
      label: "자율 응답 지연",
      state: { ...base, mode: "auto", authority: "autonomy", allowed: ["emergency", "land", "pause", "takeover", "auto_stop"],
        warnings: ["autonomy_stale"], mission: mission("REPLAN", "running", 0.52) },
      detections: 2, alert: "stale",
    },
    manual: {
      label: "사람이 조종 중",
      state: { ...base, mode: "manual", authority: "operator", allowed: ["emergency", "land", "auto_start"],
        warnings: [], operator_interventions: 1, mission: null },
      detections: 2,
    },
    ground: {
      label: "이륙 전",
      state: { ...base, battery_pct: 87, height_m: null, flight_time_s: 0, autonomy_used: false, mode: "grounded", authority: "none",
        allowed: ["emergency", "confirm_area", "takeoff"], warnings: [],
        down_tof: { m: null, valid: false, reason: "below_min_or_on_ground" }, mission: null },
      detections: 0,
    },
  };

  // 자율 위치 추정 경로(map 좌표, m). 발견 표시는 탐지 시각에 가장 가까운 드론 위치다(사람 위치 아님).
  const track = [
    [0, 0.0, 0.0], [20, 0.0, 2.6], [40, 1.5, 3.4], [60, 3.6, 3.3], [80, 4.6, 2.2],
    [100, 4.8, 0.8], [120, 6.4, 0.5], [140, 8.1, 0.9], [160, 8.6, 2.4], [180, 8.5, 3.6],
  ].map(([t, x, y]) => ({ t_s: t, x_m: x, y_m: y }));
  const plan = [[8.5, 3.6], [8.0, 4.4], [6.6, 4.6]].map(([x, y]) => ({ x_m: x, y_m: y }));
  const rooms = [
    { name: "방 A", x: 1.0, y: 2.6, w: 2.4, h: 1.6 },
    { name: "방 B", x: 6.0, y: 1.6, w: 1.8, h: 2.2 },
  ];

  const ev = (n, t, conf, bbox) => ({
    schema_version: "0.1.0", run_id: RUN, event_id: `sample-event-${n}`, frame_id: `frame-${String(t * 24).padStart(6, "0")}`,
    pts_us: t * 1e6, class_name: "person", confidence: conf, bbox_xyxy_px: bbox, image_width: 960, image_height: 720,
    object_position: null,
  });
  const events = [
    ev(1, 45, 0.61, [520, 250, 610, 470]),
    ev(2, 118, 0.44, [180, 300, 230, 420]),
    ev(3, 176, 0.87, [500, 210, 600, 480]),
  ];

  const timeline = [
    { t_s: 0, kind: "mode", text: "이륙 (운용자)" },
    { t_s: 8, kind: "mode", text: "수동 → 자율 (운용자)" },
    { t_s: 45, kind: "detection", text: "사람 발견 0.61" },
    { t_s: 92, kind: "intervention", text: "일시정지 (운용자) → 다시 시작" },
    { t_s: 118, kind: "detection", text: "사람 발견 0.44" },
    { t_s: 176, kind: "detection", text: "사람 발견 0.87" },
  ];

  return { run_id: RUN, scenarios, track, plan, rooms, events, timeline };
})();
