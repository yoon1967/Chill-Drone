// 관제 화면 테마: 자동(이 기기의 밝게/어둡게 설정을 따름)·밝게·어둡게.
// <html data-theme="light|dark">를 정하면 style.css 맨 위 색 토큰이 바뀐다. <head>에서 먼저 읽어 처음부터 맞는 색으로 그린다.
// 고른 값은 이 브라우저에만 저장한다(저장이 막혀 있어도 이번 화면에는 적용).
// 테마가 바뀌면 window에 "themechange"를 보낸다(3D 점군처럼 캔버스에 그리는 칸이 다시 그린다).
(function () {
  const KEY = "controller.theme";
  const root = document.documentElement;
  const media = window.matchMedia ? window.matchMedia("(prefers-color-scheme: dark)") : null;
  let pref = "auto";
  try {
    const saved = localStorage.getItem(KEY);
    if (saved === "light" || saved === "dark") pref = saved;
  } catch (e) { /* 저장소를 못 쓰면 자동 */ }

  function apply() {
    const theme = pref === "auto" ? (media && media.matches ? "dark" : "light") : pref;
    if (root.dataset.theme === theme) return;
    root.dataset.theme = theme;
    window.dispatchEvent(new CustomEvent("themechange", { detail: { theme: theme, pref: pref } }));
  }

  function choose(value) {
    pref = value === "light" || value === "dark" ? value : "auto";
    try {
      if (pref === "auto") localStorage.removeItem(KEY);
      else localStorage.setItem(KEY, pref);
    } catch (e) { /* 이번 화면에만 적용 */ }
    apply();
  }

  apply();
  if (media) {
    if (media.addEventListener) media.addEventListener("change", apply);
    else if (media.addListener) media.addListener(apply);   // 옛 Safari
  }
  document.addEventListener("DOMContentLoaded", function () {
    const select = document.getElementById("theme");
    if (!select) return;
    select.value = pref;
    select.addEventListener("change", function () { choose(select.value); });
  });
})();
