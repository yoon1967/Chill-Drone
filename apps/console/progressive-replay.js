export class ProgressiveReplay {
  constructor() { this.entries = []; this.following = true; this.playing = false; this.time = 0; this.rate = 1; this.version = 0; this.wall = null; }
  get first() { return this.entries[0]?.timestamp_s ?? 0; }
  get last() { return this.entries.at(-1)?.timestamp_s ?? 0; }
  setEntries(entries) { this.entries = entries; this.time = this.following ? this.last : Math.max(this.first, Math.min(this.last, this.time)); }
  seek(seconds) { if (!Number.isFinite(seconds) || !this.entries.length) return; this.following = false; this.playing = false; this.wall = null; this.time = Math.max(this.first, Math.min(this.last, seconds)); this.version++; }
  follow() { this.following = true; this.playing = false; this.time = this.last; this.wall = null; this.version++; }
  play(now) { if (!this.entries.length) return; if (this.time >= this.last) this.time = this.first; this.following = false; this.playing = true; this.wall = now; this.version++; }
  pause() { this.playing = false; this.wall = null; this.version++; }
  tick(now) { if (!this.playing) return; this.time = Math.min(this.last, this.time + Math.max(0, now - this.wall) / 1000 * this.rate); this.wall = now; if (this.time >= this.last) this.pause(); }
  selected() { let lo = 0, hi = this.entries.length - 1; while (lo < hi) { const mid = Math.ceil((lo + hi) / 2); if (this.entries[mid].timestamp_s <= this.time) lo = mid; else hi = mid - 1; } return this.entries[lo]; }
}
