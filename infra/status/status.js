const el = (id) => document.getElementById(id);
const dateFormat = new Intl.DateTimeFormat('ko-KR', {timeZone:'Asia/Seoul',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
let pending = false;
let lastData = null;
let containerFilter = 'all';
const icons = {web:'◫',edge:'◇',docker:'▦','gitlab-runner':'↗'};
const descriptions = {web:'웹 페이지 · 상태 확인 API',edge:'HTTPS 연결 · 인증서 자동 갱신',docker:'컨테이너 실행 환경 · 호스트 서비스','gitlab-runner':'검증 · 빌드 · 배포 · 호스트 서비스'};
const stateLabels = {created:'생성됨',running:'실행 중',paused:'일시 중지',restarting:'재시작 중',removing:'삭제 중',exited:'중지됨',dead:'실패',unknown:'미확인'};
const eventLabels = {create:'생성',start:'시작',stop:'중지',die:'종료',destroy:'삭제',restart:'재시작',pause:'일시 중지',unpause:'재개',rename:'이름 변경','health_status: healthy':'상태 정상','health_status: unhealthy':'상태 이상','health_status: starting':'상태 검사 시작'};
function text(id,value) { if(el(id).textContent!==String(value))el(id).textContent=value; }
function badge(label,kind='') {
  const item=document.createElement('span');item.className=`badge ${kind}`;
  const dot=document.createElement('span');dot.className='dot';
  item.append(dot,document.createTextNode(label));return item;
}
function renderContainers(data) {
  const containers=data.containers || [];
  const query=el('container-search').value.trim().toLowerCase();
  const running=containers.filter(item=>item.state==='running').length;
  el('filter-all').textContent=`전체 ${containers.length}`;
  el('filter-running').textContent=`실행 중 ${running}`;
  el('filter-stopped').textContent=`미실행 ${containers.length-running}`;
  document.querySelectorAll('.filter-button').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.filter===containerFilter)));
  el('inventory-count').textContent=data.fresh?`${containers.length} CONTAINERS`:'상태 확인 필요';
  const filtered=containers.filter(item=>(containerFilter==='all'||(containerFilter==='running'?item.state==='running':item.state!=='running'))&&`${item.name} ${item.image}`.toLowerCase().includes(query));
  el('services').replaceChildren();
  for(const item of filtered) {
    const row=document.createElement('div');row.className='service container-row';
    const icon=document.createElement('span');icon.className='service-icon';icon.setAttribute('aria-hidden','true');icon.textContent=icons[item.id] || '▦';
    const info=document.createElement('div');info.className='service-info';
    const name=document.createElement('p');name.className='service-name container-name';name.textContent=item.name;
    const image=document.createElement('p');image.className='service-description container-image';image.textContent=item.image?item.image.split('@sha256:')[0]:'이미지 확인 중';image.title=item.image || '';
    const identifier=document.createElement('p');identifier.className='container-meta';identifier.textContent=[item.label!==item.name?item.label:'',item.container_id].filter(Boolean).join(' · ');
    info.append(name,image,identifier);
    const healthy=item.state==='running'&&item.health==='healthy';
    const label=!data.fresh?'정보 지연':item.state==='running'&&item.health==='unhealthy'?'상태 이상':item.state==='running'&&item.health==='starting'?'준비 중':healthy?'정상':stateLabels[item.state] || '미확인';
    const kind=!data.fresh?'warning':item.state==='exited'||item.state==='created'?'stopped':item.health==='unhealthy'||item.state==='dead'?'danger':item.state==='running'&&item.health!=='starting'?'':'warning';
    row.append(icon,info,badge(label,kind));el('services').append(row);
  }
  if(!filtered.length){const empty=document.createElement('p');empty.className='placeholder';empty.textContent=!data.fresh?'컨테이너 정보를 확인하지 못했습니다.':containers.length?'조건에 맞는 컨테이너가 없습니다.':'등록된 컨테이너가 없습니다.';el('services').append(empty);}
}
function renderHostServices(data) {
  el('host-services').replaceChildren();
  data.services.forEach((item) => {
    const okay = data.fresh && item.state === 'active';
    const row = document.createElement('div'); row.className='service';
    const icon = document.createElement('span');icon.className='service-icon';icon.setAttribute('aria-hidden','true');icon.textContent=icons[item.id] || '◇';
    const info=document.createElement('div');info.className='service-info';
    const name=document.createElement('p');name.className='service-name';name.textContent=item.label;
    const description=document.createElement('p');description.className='service-description';description.textContent=descriptions[item.id];
    info.append(name,description);
    row.append(icon,info,badge(okay?'정상':data.fresh?'확인 필요':'정보 지연',okay?'':'warning'));el('host-services').append(row);
  });
}
function renderEvents(data) {
  el('events').replaceChildren();
  for(const event of (data.events || []).slice(0,8)) {
    const row=document.createElement('div');row.className='event-row';
    const action=document.createElement('span');action.className=`event-action ${event.action==='destroy' || event.action==='stop'?'muted':''}`;action.textContent=eventLabels[event.action] || '변경';
    const name=document.createElement('span');name.className='event-name';name.textContent=event.name;
    const time=document.createElement('time');time.dateTime=event.at;time.textContent=dateFormat.format(new Date(event.at));
    row.append(action,name,time);el('events').append(row);
  }
  if(!(data.events || []).length){const empty=document.createElement('p');empty.className='placeholder';empty.textContent=data.events_ok?'새 변경이 생기면 여기에 표시됩니다.':'변경 내역을 확인하고 있습니다.';el('events').append(empty);}
  el('events-note').textContent=data.events_ok?'최근 20건 중 최대 8건 표시 · 재부팅 시 초기화':'변경 내역 수집 지연 · 컨테이너 목록은 별도로 갱신됩니다.';
}
function resource(name,value,fresh) {
  const valid=fresh && Number.isFinite(value);
  el(name).textContent=valid?value.toFixed(1):'—';
  el(`${name}-bar`).style.width=valid?`${value}%`:'0%';
  const meter=el(`${name}-meter`);
  if(valid){meter.setAttribute('aria-valuenow',value);}else{meter.removeAttribute('aria-valuenow');}
}
function uptime(seconds) {
  if(!Number.isFinite(seconds))return '—';
  const days=Math.floor(seconds/86400),hours=Math.floor(seconds%86400/3600),minutes=Math.floor(seconds%3600/60);
  return days?`${days}일 ${hours}시간`:hours?`${hours}시간 ${minutes}분`:`${minutes}분`;
}
async function refresh() {
  if(pending)return;
  pending=true;el('refresh').disabled=true;
  const start=performance.now();
  try {
    const response=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(5000)});
    if(!response.ok)throw new Error('Status unavailable');
    const data=await response.json();
    lastData=data;
    const healthy=data.status==='ok';
    document.body.classList.remove('is-offline');
    el('hero').className=healthy?'hero':'hero degraded';
    el('hero-icon').querySelector('path').setAttribute('d',healthy?'m8 16 5 5 11-11':'M16 8v12M16 25v.1');
    el('overall-caption').textContent=healthy?'ALL SYSTEMS OPERATIONAL':'ATTENTION REQUIRED';
    text('overall-title',healthy?'실행 중인 컨테이너가 정상이에요':data.fresh&&data.summary.stopped?'미실행 컨테이너가 있어요':'일부 상태를 확인해 주세요');
    el('overall-detail').textContent=data.fresh?`전체 ${data.summary.total}개 · 실행 중 ${data.summary.running}개 · 미실행 ${data.summary.stopped}개. 새 컨테이너는 자동으로 표시됩니다.`:'호스트 상태 정보가 늦어지고 있습니다. 최신 정보가 도착하면 갱신됩니다.';
    el('live-text').textContent=data.fresh?'실시간 연결':'정보 지연';
    el('last-checked').textContent=`최근 확인 ${dateFormat.format(new Date())}`;
    el('latency').textContent=Math.round(performance.now()-start);
    el('container-count').textContent=data.fresh?data.summary.running:'—';
    el('container-total').textContent=data.fresh?`/ ${data.summary.total}`:'/ —';
    el('container-note').textContent=data.fresh?`전체 ${data.summary.total}개 중 ${data.summary.stopped}개 미실행`:'최신 실행 상태를 확인하지 못했습니다';
    resource('cpu',data.resources.cpu_percent,data.fresh);resource('memory',data.resources.memory_percent,data.fresh);
    el('disk').textContent=data.fresh && Number.isFinite(data.resources.disk_percent)?`${data.resources.disk_percent.toFixed(1)}%`:'—';
    el('uptime').textContent=data.fresh?uptime(data.uptime_seconds):'—';
    el('revision').textContent=data.revision.slice(0,8);el('revision').title=data.revision;
    el('snapshot-note').textContent=data.fresh?`호스트 수집 ${dateFormat.format(new Date(data.collected_at))} · 3초 간격`:'최신 호스트 정보를 확인하지 못했습니다. 수집은 3초 간격으로 진행합니다.';
    renderContainers(data);renderHostServices(data);renderEvents(data);
  } catch(error) {
    document.body.classList.add('is-offline');el('hero').className='hero offline';
    el('hero-icon').querySelector('path').setAttribute('d','m10 10 12 12M22 10 10 22');
    el('overall-caption').textContent='CONNECTION UNAVAILABLE';text('overall-title','서버 응답을 확인할 수 없어요');
    el('overall-detail').textContent='네트워크 연결을 확인해 주세요. 3초 후 다시 확인합니다.';
    el('live-text').textContent='연결 확인 필요';el('latency').textContent='—';el('container-count').textContent='—';el('container-total').textContent='/ —';
    el('container-note').textContent='최신 실행 상태를 확인하지 못했습니다';
    resource('cpu',null,false);resource('memory',null,false);el('disk').textContent='—';el('uptime').textContent='—';
    el('snapshot-note').textContent='현재 상태를 가져오지 못했습니다. 자동으로 다시 확인합니다.';
    const data={...(lastData || {containers:[],services:[],events:[]}),fresh:false};renderContainers(data);renderHostServices(data);renderEvents(data);
  } finally {pending=false;el('refresh').disabled=false;}
}
el('refresh').addEventListener('click',refresh);
el('container-search').addEventListener('input',()=>{if(lastData)renderContainers(lastData);});
document.querySelectorAll('.filter-button').forEach(button=>button.addEventListener('click',()=>{containerFilter=button.dataset.filter;if(lastData)renderContainers(lastData);}));
document.addEventListener('visibilitychange',()=>{if(!document.hidden)refresh();});
refresh();setInterval(()=>{if(!document.hidden)refresh();},3000);
