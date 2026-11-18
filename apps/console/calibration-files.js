const PHOTO = /\.(jpe?g|png|webp)$/i;
const VIDEO = /\.(mp4|webm|mov)$/i;

export function validateCalibrationFiles(files, {additional = false} = {}) {
  const list = Array.from(files || []);
  if (!list.length) throw new Error('동영상 1개 또는 사진 여러 장을 선택하세요.');
  const videos = list.filter(file => ['video/mp4', 'video/webm', 'video/quicktime'].includes(file.type) ||
    (!file.type && VIDEO.test(file.name)));
  const photos = list.filter(file => ['image/jpeg', 'image/png', 'image/webp'].includes(file.type) ||
    (!file.type && PHOTO.test(file.name)));
  if (videos.length === 1 && list.length === 1) {
    if (videos[0].size > 300 * 1024 * 1024) throw new Error('동영상은 300MB 이하를 선택하세요.');
    return {kind: 'video', files: list};
  }
  if (photos.length !== list.length || list.length > 120 || (!additional && list.length < 2) ||
      list.some(file => file.size > 25 * 1024 * 1024) ||
      list.reduce((size, file) => size + file.size, 0) > 300 * 1024 * 1024)
    throw new Error('같은 카메라의 사진 2~120장(각 25MB 이하, 합계 300MB 이하) 또는 동영상 1개를 선택하세요.');
  return {kind: 'photos', files: list};
}

function frameSize(width, height, expected) {
  if (expected) {
    if (Math.abs(width / height - expected[0] / expected[1]) > .01)
      throw new Error('영상 방향이나 가로세로 비율이 다릅니다. 같은 촬영 설정의 자료를 사용하세요.');
    return expected;
  }
  const scale = Math.min(1, 1280 / width, 960 / height);
  const size = [Math.round(width * scale), Math.round(height * scale)];
  if (size[0] < 320 || size[1] < 180)
    throw new Error('사진·동영상 해상도가 너무 낮습니다. 320×180 이상을 사용하세요.');
  return size;
}

async function jpeg(canvas) {
  const encode = quality => new Promise(resolve => canvas.toBlob(resolve, 'image/jpeg', quality));
  let blob = await encode(.85);
  if (blob?.size > 2097152) blob = await encode(.65);
  if (!blob || blob.size > 2097152) throw new Error('한 장면이 2MB를 넘습니다. 더 작은 해상도의 자료를 선택하세요.');
  return blob;
}

function waitEvent(target, success) {
  return new Promise((resolve, reject) => {
    const done = () => { target.removeEventListener(success, ok); target.removeEventListener('error', fail); };
    const ok = () => { done(); resolve(); };
    const fail = () => { done(); reject(new Error('동영상을 읽지 못했습니다. 브라우저가 지원하는 MP4 또는 WebM을 사용하세요.')); };
    target.addEventListener(success, ok, {once: true});
    target.addEventListener('error', fail, {once: true});
  });
}

export async function feedCalibrationFiles(selection, {canvas, context, onFrame, onProgress,
  isCurrent = () => true, expectedSize, video} = {}) {
  let size = expectedSize;
  const send = async (image, width, height, index, total) => {
    if (!isCurrent()) return false;
    size = frameSize(width, height, size);
    canvas.width = size[0]; canvas.height = size[1];
    context.drawImage(image, 0, 0, ...size);
    const state = await onFrame(await jpeg(canvas));
    if (!isCurrent()) return false;
    onProgress(index, total, state);
    return state.accepted < 40 && !state.coverage?.ready;
  };
  if (selection.kind === 'photos') {
    for (let i = 0; i < selection.files.length; i++) {
      if (!isCurrent()) break;
      const image = await createImageBitmap(selection.files[i]);
      try { if (!await send(image, image.width, image.height, i + 1, selection.files.length)) break; }
      finally { image.close(); }
    }
    return size;
  }
  video ||= document.createElement('video');
  const url = URL.createObjectURL(selection.files[0]);
  try {
    video.preload = 'auto'; video.muted = true; video.playsInline = true;
    video.src = url;
    if (video.readyState < 1) await waitEvent(video, 'loadedmetadata');
    if (!Number.isFinite(video.duration) || video.duration < 1 || video.duration > 300)
      throw new Error('동영상 길이는 1초 이상, 5분 이하로 선택하세요.');
    const count = Math.min(120, Math.max(30, Math.ceil(video.duration * 2)));
    for (let i = 0; i < count && isCurrent(); i++) {
      const position = Math.min(video.duration - .01, (i + .5) * video.duration / count);
      const ready = waitEvent(video, 'seeked');
      video.currentTime = position;
      await ready;
      if (!await send(video, video.videoWidth, video.videoHeight, i + 1, count)) break;
    }
    return size;
  } finally {
    video.removeAttribute('src'); video.load(); URL.revokeObjectURL(url);
  }
}
