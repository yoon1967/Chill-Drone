const KEY = 'rescue-mapping-settings-v1';
const defaults = {semantic_mapping: true, depth_resolution: 384, sample_fps: 5, point_budget: 32000, person_color: '#ff3030', person_outline: true, semantic_colors: false, appearance_changed: false};
let settings = {...defaults};
try {
  const saved = JSON.parse(globalThis.localStorage?.getItem(KEY) || '{}');
  settings = {...defaults, ...saved};
  // Upgrade the previous default once; subsequent explicit low-budget choices
  // and person appearance preferences survive reloads.
  if (saved.point_budget_version !== 2 && saved.point_budget === 8000) settings.point_budget = defaults.point_budget;
  settings.point_budget_version = 2;
  globalThis.localStorage?.setItem(KEY, JSON.stringify(settings));
} catch {}
const subscribers = new Set();
export function validPersonColor(value) { return typeof value === 'string' && /^#[\da-f]{6}$/i.test(value); }
if (!validPersonColor(settings.person_color)) settings.person_color = defaults.person_color;
const validSampleFps = value => typeof value === 'number' && Number.isFinite(value) && value >= .1 && value <= 15;
if (!validSampleFps(settings.sample_fps)) settings.sample_fps = defaults.sample_fps;
export function mappingSettings() { return {...settings}; }
export function mappingOptions() {
  const input = globalThis.document?.getElementById?.('rescue-sample-fps');
  if (input?.validity?.valid === false) throw new Error('지도에 사용할 초당 프레임은 0.1~15 사이로 입력해 주세요.');
  return {semantic_mapping: settings.semantic_mapping === true, depth_resolution: [256, 384, 518].includes(settings.depth_resolution) ? settings.depth_resolution : 384,
    point_budget: [4000,8000,10000,16000,32000].includes(settings.point_budget) ? settings.point_budget : defaults.point_budget,
    sample_fps: settings.sample_fps, person_color: settings.person_color, person_outline: settings.person_outline === true};
}
export function onMappingSettings(callback) { subscribers.add(callback); return () => subscribers.delete(callback); }
export function connectMappingSettings() {
  const panel = document.getElementById('rescue-settings'); if (!panel || panel.dataset.bound) return; panel.dataset.bound = 'true';
  const fields = {'rescue-enabled': ['semantic_mapping', 'checked'], 'rescue-resolution': ['depth_resolution', 'value'],
    'rescue-sample-fps': ['sample_fps', 'value'], 'import-sample-fps': ['sample_fps', 'value'],
    'rescue-point-budget': ['point_budget', 'value'],
    'rescue-person-color': ['person_color', 'value'], 'rescue-person-outline': ['person_outline', 'checked'], 'rescue-semantic-colors': ['semantic_colors', 'checked']};
  const savedMode = () => document.getElementById('import-mode')?.value === 'sfm';
  function samplingControls() {
    for (const id of ['rescue-sample-fps','import-sample-fps']) {
      const input = document.getElementById(id);
      if (id === 'import-sample-fps' && savedMode()) { if (input) input.max = 30; continue; }
      if (input) { input.value = settings.sample_fps; input.max = 15; input.setCustomValidity?.(''); }
    }
    const note = document.getElementById('rescue-sampling-note');
    if (note) note.textContent = `초당 ${settings.sample_fps}프레임 목표 · 약 ${Number((1/settings.sample_fps).toFixed(3))}초마다 1장`;
  }
  for (const [id, [name, property]] of Object.entries(fields)) {
    const input = document.getElementById(id); if (!input) continue;
    input[property] = settings[name];
    input.addEventListener('input', () => {
      if (id === 'import-sample-fps' && savedMode()) return;
      const value = ['depth_resolution','point_budget','sample_fps'].includes(name) ? Number(input[property]) : input[property];
      if (name === 'sample_fps' && !validSampleFps(value)) {
        input.setCustomValidity?.('초당 프레임은 0.1~15 사이로 입력해 주세요.'); return;
      }
      settings[name] = value;
      if (name === 'sample_fps') samplingControls();
      if (name === 'person_color' || name === 'person_outline') settings.appearance_changed = true;
      if (!validPersonColor(settings.person_color)) settings.person_color = defaults.person_color;
      try { localStorage.setItem(KEY, JSON.stringify(settings)); } catch {}
      for (const callback of subscribers) callback(mappingSettings());
    });
  }
  document.getElementById('import-mode')?.addEventListener('change', () => {
    const input = document.getElementById('import-sample-fps');
    if (savedMode()) { if (input) { input.max = 30; input.setCustomValidity?.(''); } }
    else samplingControls();
  });
  samplingControls();
}

const palette = [0x899b9c,0xff3030,0xa8bac4,0x7f9a80,0xe2dcc2,0xc29462,0x7aacc1,0xb58aab,0x80c9d2];
export function rescuePointColor(rgb, label, value, prefs) {
  let color;
  if (label === 1) color = parseInt((prefs.appearance_changed ? prefs.person_color : value.person_color || prefs.person_color).slice(1),16);
  else if (prefs.semantic_colors && Number.isInteger(label)) color = palette[label];
  if (color === undefined) return rgb;
  return [color>>16&255,color>>8&255,color&255];
}
