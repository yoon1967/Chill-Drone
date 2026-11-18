import test from 'node:test';
import assert from 'node:assert/strict';
import { recordingDescription, renderRecording } from '../recording-view.js';

const state = (recording = {}, extra = {}) => ({session_id: 'live_example', session_status: 'receiving',
  recording: {status: 'recording', bytes_received: 200, bytes_written: 150, parts: 1, ...recording}, ...extra});

test('original drone bitstream and received JPEG input have different labels', () => {
  assert.match(recordingDescription(state({original_h264: true})).label, /드론 원본 녹화/);
  assert.match(recordingDescription(state({original_h264: true})).detail, /H.264 원본/);
  assert.match(recordingDescription(state()).label, /수신 영상/);
  assert.equal(recordingDescription({session_id: 'old'}), null);
  assert.equal(recordingDescription({recording: {status: 'complete'}}), null);
});

test('download is available only after both recording and mapping have finished', () => {
  assert.equal(recordingDescription(state({finalized: true})).downloadable, false);
  assert.equal(recordingDescription(state({finalized: false}, {session_status: 'complete'})).downloadable, false);
  const result = recordingDescription(state({finalized: true}, {session_status: 'complete'}));
  assert.equal(result.downloadable, true);
  assert.equal(result.url, '/api/live/session-download?session=live_example');
  assert.match(recordingDescription(state({status: 'finalizing'})).label, /마무리/);
});

test('storage errors keep map processing distinct and preserve completed downloads', () => {
  assert.match(recordingDescription(state({error: 'disk_full'})).label, /지도 처리는 계속/);
  assert.match(recordingDescription(state({error: 'disk_full'}, {session_status: 'failed'})).label, /기록 정리/);
  const result = recordingDescription(state({error: 'disk_full', finalized: true, playback_errors: 1}, {session_status: 'failed'}));
  assert.equal(result.downloadable, true);
  assert.match(result.label, /일부 실패/);
  assert.match(result.detail, /재생 파일을 만들지 못/);
  assert.match(recordingDescription(state({bytes_written: 0, finalized: true}, {session_status: 'complete'})).label, /영상 없이/);
});

test('rendering clears old download links when another session starts', () => {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {removeAttribute(name) { delete this[name]; }});
    return elements.get(id);
  };
  renderRecording(state({finalized: true, session_path: 'local session'}, {session_status: 'complete'}), element);
  assert.equal(element('session-download').hidden, false);
  assert.match(element('recording-path').textContent, /local session/);
  renderRecording(state(), element);
  assert.equal(element('session-download').hidden, true);
  assert.equal(element('session-download').href, undefined);
  renderRecording({}, element);
  assert.equal(element('session-recording').hidden, true);
});
