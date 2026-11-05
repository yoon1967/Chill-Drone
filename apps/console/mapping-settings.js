const KEY = 'rescue-mapping-settings-v1';
const defaults = {semantic_mapping: true, depth_resolution: 384, point_budget: 8000, person_color: '#ff3030', person_outline: true, semantic_colors: false, appearance_changed: false};
let settings = {...defaults};
try { settings = {...defaults, ...JSON.parse(globalThis.localStorage?.getItem(KEY) || '{}')}; } catch {}
const subscribers = new Set();
export function validPersonColor(value) { return typeof value === 'string' && /^#[\da-f]{6}$/i.test(value); }
if (!validPersonColor(settings.person_color)) settings.person_color = defaults.person_color;
export function mappingSettings() { return {...settings}; }
export function mappingOptions() {
  return {semantic_mapping: settings.semantic_mapping === true, depth_resolution: [256, 384, 518].includes(settings.depth_resolution) ? settings.depth_resolution : 384,
    point_budget: [4000,8000,10000].includes(settings.point_budget) ? settings.point_budget : 8000,
    person_color: settings.person_color, person_outline: settings.person_outline === true};
}
export function onMappingSettings(callback) { subscribers.add(callback); return () => subscribers.delete(callback); }
export function connectMappingSettings() {
  const panel = document.getElementById('rescue-settings'); if (!panel || panel.dataset.bound) return; panel.dataset.bound = 'true';
  const fields = {'rescue-enabled': ['semantic_mapping', 'checked'], 'rescue-resolution': ['depth_resolution', 'value'],
    'rescue-point-budget': ['point_budget', 'value'],
    'rescue-person-color': ['person_color', 'value'], 'rescue-person-outline': ['person_outline', 'checked'], 'rescue-semantic-colors': ['semantic_colors', 'checked']};
  for (const [id, [name, property]] of Object.entries(fields)) {
    const input = document.getElementById(id); input[property] = settings[name];
    input.addEventListener('input', () => {
      settings[name] = ['depth_resolution','point_budget'].includes(name) ? Number(input[property]) : input[property];
      if (name === 'person_color' || name === 'person_outline') settings.appearance_changed = true;
      if (!validPersonColor(settings.person_color)) settings.person_color = defaults.person_color;
      try { localStorage.setItem(KEY, JSON.stringify(settings)); } catch {}
      for (const callback of subscribers) callback(mappingSettings());
    });
  }
}

const palette = [0x899b9c,0xff3030,0xa8bac4,0x7f9a80,0xe2dcc2,0xc29462,0x7aacc1,0xb58aab,0x80c9d2];
export function rescuePointColor(rgb, label, value, prefs) {
  let color;
  if (label === 1) color = parseInt((prefs.appearance_changed ? prefs.person_color : value.person_color || prefs.person_color).slice(1),16);
  else if (prefs.semantic_colors && Number.isInteger(label)) color = palette[label];
  if (color === undefined) return rgb;
  return [color>>16&255,color>>8&255,color&255];
}
