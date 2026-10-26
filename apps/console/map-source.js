// Every asynchronous read belongs to one tab selection, including its response body.
export class MapSource {
  constructor(source = 'saved', fetcher = (url, options) => fetch(url, options)) {
    this.fetcher = fetcher;
    this.generation = 0;
    this.select(source);
  }

  select(source) {
    if (!['saved', 'live'].includes(source)) throw new Error('알 수 없는 지도입니다.');
    this.controller?.abort();
    this.source = source;
    this.generation++;
    this.controller = new AbortController();
  }

  context() { return {source: this.source, generation: this.generation, signal: this.controller.signal}; }
  isCurrent(context) { return context.generation === this.generation; }

  async read(path, context) {
    try {
      const response = await this.fetcher(`/api/${context.source}/${path}`, {signal: AbortSignal.any([context.signal, AbortSignal.timeout(10000)])});
      if (!this.isCurrent(context) || response.status === 409) return null;
      const value = await response.json();
      if (!this.isCurrent(context)) return null;
      return {ok: response.ok, status: response.status, value};
    } catch (error) {
      if (!this.isCurrent(context) || error.name === 'AbortError') return null;
      throw error;
    }
  }
}

export function captureError(error) {
  if (error.name === 'NotAllowedError' || error.name === 'SecurityError')
    return {state: 'denied', message: '카메라 권한이 거부되었습니다. 브라우저의 이 사이트 카메라 권한을 확인한 뒤 다시 시작하세요.'};
  if (error.name === 'NotFoundError')
    return {state: 'failed', message: '연결된 카메라를 찾지 못했습니다. 카메라 연결이나 녹화 영상 입력을 확인하세요.'};
  if (error.name === 'NotReadableError')
    return {state: 'failed', message: '카메라를 열지 못했습니다. 다른 앱에서 사용 중인지 확인한 뒤 다시 시작하세요.'};
  return {state: 'failed', message: `영상 연결 실패: ${error.message}`};
}
