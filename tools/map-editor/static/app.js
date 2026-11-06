const initializeMapEditor = () => {
  const requiredIds = [
    'mapCanvas', 'mapSelect', 'banner', 'mapStatus', 'mapIdentity', 'coords',
    'regionList', 'chargerList', 'regionEditor', 'chargerEditor', 'emptySelection',
    'planResult', 'undoBtn', 'redoBtn', 'newRegionBtn', 'deleteRegionBtn',
    'addChargerBtn', 'deleteChargerBtn', 'relocateChargerBtn', 'chargerId',
    'chargerName', 'chargerX', 'chargerY', 'chargerYaw', 'regionId', 'regionName',
    'routeId', 'margin', 'direction', 'laps', 'planBtn', 'finishBoundaryBtn',
    'cancelDrawBtn', 'fitBtn', 'saveProjectBtn', 'loadProjectBtn', 'fileInput',
    'exportBtn',
  ];
  const missingIds = requiredIds.filter(id => !document.getElementById(id));
  if (missingIds.length) {
    const message = `地图编辑器页面结构不完整，缺少控件：${missingIds.join(', ')}`;
    const notice = document.createElement('div');
    notice.setAttribute('role', 'alert');
    notice.style.cssText = 'margin:1rem;padding:1rem;background:#fff0ef;color:#8b322a;font:16px sans-serif';
    notice.textContent = message;
    document.body?.prepend(notice);
    console.error(message);
    return;
  }
  const $ = id => document.getElementById(id);
  const canvas = $('mapCanvas'), ctx = canvas.getContext('2d');
  let info, mapImage = new Image(), project, selectedRegion = null, selectedCharger = null;
  let drawMode = 'select', draft = [], pendingCharger = null, planned = new Map();
  let fit = 1, zoom = 1, panX = 0, panY = 0, drag = null, undoStack = [], redoStack = [];
  let canvasRect, dpr = window.devicePixelRatio || 1;

  const clone = obj => JSON.parse(JSON.stringify(obj));
  const active = () => project?.regions.find(r => r.region_id === selectedRegion);
  const safeText = s => String(s ?? '').replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;').replaceAll("'",'&#39;');
  const sameIdentity = (a,b) => !!a&&!!b&&['map_id','revision','sha256','frame_id'].every(k=>a[k]===b[k]);
  const selectedPlan = () => active() && planned.get(active().region_id);

  async function api(path, data) {
    const response = await fetch(path, data ? {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)} : {});
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || response.statusText);
    return body;
  }
  function setBanner(message, error=true) {
    const el = $('banner'); el.textContent = message; el.classList.toggle('hidden', !message);
    el.style.background = error ? '#fff0ef' : '#eaf7f0'; el.style.color = error ? '#8b322a' : '#155a39';
  }
  function snapshot() { undoStack.push(clone(project)); if (undoStack.length > 40) undoStack.shift(); redoStack=[]; refreshUndo(); }
  function refreshUndo() { $('undoBtn').disabled=!undoStack.length; $('redoBtn').disabled=!redoStack.length; }
  function undo() { if(!undoStack.length)return; redoStack.push(clone(project)); project=undoStack.pop(); selectedRegion=project.regions[0]?.region_id||null; selectedCharger=null; planned.clear(); renderAll(); refreshUndo(); }
  function redo() { if(!redoStack.length)return; undoStack.push(clone(project)); project=redoStack.pop(); selectedRegion=project.regions[0]?.region_id||null; selectedCharger=null; planned.clear(); renderAll(); refreshUndo(); }
  function download(filename, data) {
    const blob=new Blob([JSON.stringify(data,null,2)+'\n'],{type:'application/json'}), link=document.createElement('a');
    link.href=URL.createObjectURL(blob);link.download=filename;link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1200);
  }
  function safeFilename(s){return String(s||'cleannav').replace(/[^A-Za-z0-9._-]+/g,'_').slice(0,60)}

  async function loadInfo() {
    info=await api('/api/info');
    mapImage=new Image(); mapImage.src='/api/map.png?'+Date.now(); await new Promise((resolve,reject)=>{mapImage.onload=resolve;mapImage.onerror=reject;});
    if (!project || !sameIdentity(project.map,info.identity)) {
      project={project_schema:'cleannav.map_editor_project.v1',map:clone(info.identity),regions:[],chargers:[]};
      selectedRegion=null;selectedCharger=null;planned.clear();undoStack=[];redoStack=[];
      drawMode='select';draft=[];pendingCharger=null;
      setModeButtons();
    }
    $('mapStatus').textContent=`${info.map_label} · ${info.width}×${info.height} · ${info.resolution.toFixed(3)} m/cell`;
    $('mapStatus').style.color=info.test_only?'#96621c':'#526b80';
    $('mapIdentity').innerHTML=`<strong>地图身份</strong><br>${safeText(info.identity.map_id)}<br>revision: ${safeText(info.identity.revision)}<br>OccupancyGrid SHA-256: ${safeText(info.identity.sha256)}<br>footprint外接圆 ${info.vehicle.disk_radius.toFixed(3)} m · Rmin ${info.vehicle.turning_radius.toFixed(2)} m`+(info.test_only?'<br><b>仅为合成测试地图，不是实测地图</b>':'');
    resize();renderAll();
  }
  async function loadMapChoices() {
    const data=await api('/api/maps'), select=$('mapSelect');
    select.innerHTML=data.maps.filter(m=>!m.unavailable).map(m=>`<option value="${safeText(m.key)}">${safeText(m.label)}${m.test_only?' (演示)':''}</option>`).join('');
    select.value=data.selected;
  }
  function pixelToMap(u,v) {
    const x=u*info.resolution,y=(info.height-v)*info.resolution,[ox,oy,yaw]=info.origin,c=Math.cos(yaw),s=Math.sin(yaw);
    return [ox+c*x-s*y,oy+s*x+c*y];
  }
  function mapToPixel(x,y) {
    const [ox,oy,yaw]=info.origin,c=Math.cos(yaw),s=Math.sin(yaw),dx=x-ox,dy=y-oy;
    return [(c*dx+s*dy)/info.resolution,info.height-(-s*dx+c*dy)/info.resolution];
  }
  function toScreen(point) { const [u,v]=mapToPixel(point[0],point[1]),scale=fit*zoom;return [panX+(canvas.clientWidth-info.width*fit*zoom)/2+u*scale,panY+(canvas.clientHeight-info.height*fit*zoom)/2+v*scale]; }
  function screenToMap(x,y) { const scale=fit*zoom,u=(x-panX-(canvas.clientWidth-info.width*scale)/2)/scale,v=(y-panY-(canvas.clientHeight-info.height*scale)/2)/scale;return pixelToMap(u,v); }
  function resize() {
    const rect=canvas.getBoundingClientRect(); if(rect.width<1||rect.height<1)return;
    dpr=window.devicePixelRatio||1;canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);
    canvasRect=rect;fit=Math.min(rect.width/info.width,rect.height/info.height)*.98;draw();
  }
  function traceWorld(points, close=false) { if(!points?.length)return;const first=toScreen(points[0]);ctx.beginPath();ctx.moveTo(...first);for(let i=1;i<points.length;i++)ctx.lineTo(...toScreen(points[i]));if(close)ctx.closePath(); }
  function arrow(p,yaw,color='#146bd1',size=13) { const [x,y]=toScreen(p);ctx.save();ctx.translate(x,y);ctx.rotate(-yaw);ctx.fillStyle=color;ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(size,0);ctx.lineTo(-size*.65,-size*.58);ctx.lineTo(-size*.35,0);ctx.lineTo(-size*.65,size*.58);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore(); }
  function draw() {
    if(!info||!mapImage.complete)return;
    const w=canvas.clientWidth,h=canvas.clientHeight;ctx.clearRect(0,0,w,h);ctx.fillStyle='#dbe2e9';ctx.fillRect(0,0,w,h);
    const scale=fit*zoom,ox=panX+(w-info.width*scale)/2,oy=panY+(h-info.height*scale)/2;
    ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,ox,oy,info.width*scale,info.height*scale);
    for(const poly of info.keepout||[]){traceWorld(poly,true);ctx.fillStyle='rgba(205,45,55,.24)';ctx.fill();ctx.strokeStyle='rgba(165,35,45,.75)';ctx.lineWidth=1.2;ctx.stroke();}
    for(const region of project?.regions||[]){
      traceWorld(region.boundary,true);ctx.fillStyle=region.region_id===selectedRegion?'rgba(32,127,214,.13)':'rgba(70,137,184,.08)';ctx.fill();ctx.strokeStyle=region.region_id===selectedRegion?'#1672ce':'#5485a7';ctx.lineWidth=2;ctx.stroke();
      const r=planned.get(region.region_id);
      if(r?.feasible){const line=r.dense.map(p=>p.slice(0,2));traceWorld(line);ctx.strokeStyle='#17a879';ctx.lineWidth=3;ctx.lineJoin='round';ctx.stroke();
        const entry=r.entry_pose;arrow(entry.slice(0,2),entry[2],'#076b4b',11);
        for(const p of r.waypoints.slice(0,-1)){const [x,y]=toScreen(p);ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);ctx.fillStyle='#075d43';ctx.fill();}
      }
      if(region.region_id===selectedRegion){for(let i=0;i<region.boundary.length;i++){const [x,y]=toScreen(region.boundary[i]);ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#146bd1';ctx.lineWidth=2;ctx.stroke();}}
      const c=region.boundary.reduce((a,p)=>[a[0]+p[0]/region.boundary.length,a[1]+p[1]/region.boundary.length],[0,0]),[cx,cy]=toScreen(c);ctx.fillStyle='#234157';ctx.font='600 13px sans-serif';ctx.fillText(region.display_name||region.region_id,cx+6,cy-6);
    }
    if(draft.length){traceWorld(draft);ctx.setLineDash([5,4]);ctx.strokeStyle='#1672ce';ctx.lineWidth=2;ctx.stroke();ctx.setLineDash([]);for(const p of draft){const [x,y]=toScreen(p);ctx.beginPath();ctx.arc(x,y,4,0,7);ctx.fillStyle='#146bd1';ctx.fill();}}
    for(const charger of project?.chargers||[]){const p=charger.staging_pose;arrow([p.x,p.y],p.yaw,charger.charger_id===selectedCharger?'#d17c10':'#c47a18',12);const [x,y]=toScreen([p.x,p.y]);ctx.fillStyle='#76501c';ctx.font='12px sans-serif';ctx.fillText(charger.display_name||charger.charger_id,x+10,y-8);}
    if(pendingCharger?.phase==='heading'){traceWorld([pendingCharger.point,pendingCharger.preview]);ctx.strokeStyle='#d17c10';ctx.lineWidth=2;ctx.stroke();}
  }
  function renderLists() {
    $('regionList').innerHTML=(project?.regions||[]).map(r=>`<div class="item ${r.region_id===selectedRegion?'selected':''}" data-region="${safeText(r.region_id)}"><span class="dot"></span>${safeText(r.display_name||r.region_id)}<small>${planned.has(r.region_id)?(planned.get(r.region_id).feasible?'环线✓':'未通过'):'未规划'}</small></div>`).join('')||'<div class="hint">尚无区域</div>';
    $('chargerList').innerHTML=(project?.chargers||[]).map(c=>`<div class="item ${c.charger_id===selectedCharger?'selected':''}" data-charger="${safeText(c.charger_id)}"><span class="dot" style="background:#c47a18"></span>${safeText(c.display_name||c.charger_id)}<small>staging</small></div>`).join('')||'<div class="hint">尚无充电位置</div>';
    document.querySelectorAll('[data-region]').forEach(el=>el.onclick=()=>{selectedRegion=el.dataset.region;selectedCharger=null;drawMode='select';draft=[];pendingCharger=null;setModeButtons();renderAll();});
    document.querySelectorAll('[data-charger]').forEach(el=>el.onclick=()=>{selectedCharger=el.dataset.charger;selectedRegion=null;drawMode='select';draft=[];pendingCharger=null;setModeButtons();renderAll();});
  }
  function showResult(result) {
    const el=$('planResult');
    if(!result){el.className='result neutral';el.textContent='尚未规划';return;}
    if(!result.feasible){el.className='result fail';el.textContent=`不可行：${result.reason}\n已检查候选数：${result.searched_candidates||0}\n区域边界可调整后重新规划。`;return;}
    el.className='result ok';const r=result.report;
    el.textContent=`几何检查通过 · ${r.checkpoint_count} 个航点\n环线 ${r.length_m.toFixed(1)} m · Rmin ${r.minimum_radius_m.toFixed(2)} m\n足迹外接圆 ${r.footprint_disk_radius_m.toFixed(2)} m · 边界裕度 ${r.safety_margin_m.toFixed(2)} m\n${result.sequence_goals_with_goto_return} 个 Mission 目标 / 3轮+进区+返充\n\n尚未验证Smac逐段路径与Gazebo运动。`;
  }
  function renderEditor() {
    const r=active(),c=project?.chargers.find(x=>x.charger_id===selectedCharger);
    $('emptySelection').classList.toggle('hidden',!!r||!!c);$('regionEditor').classList.toggle('hidden',!r);$('chargerEditor').classList.toggle('hidden',!c);
    if(c){$('chargerId').value=c.charger_id;$('chargerName').value=c.display_name||c.charger_id;$('chargerX').value=c.staging_pose.x;$('chargerY').value=c.staging_pose.y;$('chargerYaw').value=c.staging_pose.yaw;return;}
    if(!r){showResult(null);return;}
    $('regionId').value=r.region_id;$('regionName').value=r.display_name;$('routeId').value=r.route_id||'perimeter';
    $('margin').value=r.options?.margin??.15;$('direction').value=r.options?.direction??'ccw';$('laps').value=r.options?.laps??3;
    showResult(planned.get(r.region_id));
  }
  function renderAll(){renderLists();renderEditor();draw();refreshUndo();}
  function invalidatePlan(){if(active())planned.delete(active().region_id);draw();showResult(null);renderLists();}

  $('newRegionBtn').onclick=()=>{
    const used=new Set(project.regions.map(r=>r.region_id));let id=['A','B','C'].find(x=>!used.has(x));if(!id){let n=1;while(used.has(`AREA${n}`))n++;id=`AREA${n}`;}
    snapshot();const name=id.length===1?`${id}区`:id;
    project.regions.push({region_id:id,display_name:name,route_id:'perimeter',boundary:[],options:{margin:.15,direction:'ccw',laps:3}});
    selectedRegion=id;selectedCharger=null;draft=[];drawMode='boundary';renderAll();setModeButtons();
  };
  $('deleteRegionBtn').onclick=()=>{if(!active())return;snapshot();project.regions=project.regions.filter(r=>r.region_id!==selectedRegion);planned.delete(selectedRegion);selectedRegion=project.regions[0]?.region_id||null;renderAll();};
  $('deleteChargerBtn').onclick=()=>{if(!selectedCharger)return;snapshot();project.chargers=project.chargers.filter(c=>c.charger_id!==selectedCharger);selectedCharger=null;renderAll();};
  $('relocateChargerBtn').onclick=()=>{if(!selectedCharger)return;pendingCharger=null;drawMode='charger-position';setModeButtons();setBanner('点击新的位置，再点击朝向；标注后会替换当前选中的充电点。',false);};
  function changeCharger(field,value){const c=project.chargers.find(x=>x.charger_id===selectedCharger);if(!c)return;snapshot();if(field==='charger_id'){if(!value||project.chargers.some(o=>o!==c&&o.charger_id===value)){setBanner('charger_id不能为空且必须唯一');renderAll();return;}c.charger_id=value;selectedCharger=value;}else if(field==='display_name')c.display_name=value;else c.staging_pose[field]=Number(value);renderAll();}
  $('chargerId').onchange=()=>changeCharger('charger_id',$('chargerId').value.trim());$('chargerName').onchange=()=>changeCharger('display_name',$('chargerName').value.trim());
  const chargerFieldIds = {x:'chargerX', y:'chargerY', yaw:'chargerYaw'};
  for(const [field,id] of Object.entries(chargerFieldIds))$(id).onchange=()=>changeCharger(field,$(id).value);
  $('regionId').onchange=()=>{const r=active();if(!r)return;const next=$('regionId').value.trim();if(!next||project.regions.some(x=>x!==r&&x.region_id===next)){setBanner('region_id不能为空且必须唯一');$('regionId').value=r.region_id;return;}snapshot();const old=r.region_id;r.region_id=next;const cached=planned.get(old);planned.delete(old);if(cached)planned.set(next,cached);selectedRegion=next;renderAll();};
  $('regionName').onchange=()=>{if(!active())return;snapshot();active().display_name=$('regionName').value.trim();renderAll();};
  $('routeId').onchange=()=>{if(!active())return;snapshot();active().route_id=$('routeId').value.trim();};
  $('margin').onchange=$('direction').onchange=$('laps').onchange=()=>{const r=active();if(!r)return;snapshot();r.options={margin:Number($('margin').value),direction:$('direction').value,laps:Number($('laps').value)};invalidatePlan();};
  $('planBtn').onclick=async()=>{const r=active();if(!r||r.boundary.length<3){setBanner('先完成至少3个边界顶点');return;}r.options={margin:Number($('margin').value),direction:$('direction').value,laps:Number($('laps').value)};r.route_id=$('routeId').value.trim()||'perimeter';showResult({feasible:false,reason:'正在计算…'});try{const result=await api('/api/plan',{boundary:r.boundary,options:r.options});planned.set(r.region_id,result);showResult(result);renderLists();draw();setBanner(result.feasible?'边缘环线已生成并完成离线几何检查。':'当前区域没有通过离线安全检查。',!result.feasible);}catch(e){showResult({feasible:false,reason:e.message});setBanner(e.message);}};

  function setModeButtons(){const drawing=drawMode==='boundary',charging=drawMode==='charger-position'||drawMode==='charger-heading';$('finishBoundaryBtn').classList.toggle('hidden',!drawing);$('cancelDrawBtn').classList.toggle('hidden',!(drawing||charging));canvas.style.cursor=drawing||charging?'crosshair':'grab';}
  $('finishBoundaryBtn').onclick=finishBoundary;
  $('cancelDrawBtn').onclick=()=>{draft=[];pendingCharger=null;drawMode='select';setModeButtons();draw();};
  function finishBoundary(){if(draft.length<3){setBanner('边界至少需要3个顶点');return;}snapshot();active().boundary=draft.map(p=>p.slice());draft=[];drawMode='select';invalidatePlan();setModeButtons();renderAll();}
  $('addChargerBtn').onclick=()=>{drawMode='charger-position';selectedCharger=null;selectedRegion=null;pendingCharger=null;setModeButtons();renderAll();setBanner('点击充电staging位置，然后再点击车辆朝向。',false);};
  $('deleteRegionBtn');
  $('fitBtn').onclick=()=>{zoom=1;panX=panY=0;resize();};
  $('undoBtn').onclick=undo;$('redoBtn').onclick=redo;
  document.addEventListener('keydown',e=>{const editing=e.target instanceof HTMLInputElement||e.target instanceof HTMLSelectElement;if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){e.preventDefault();undo();}else if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='y'){e.preventDefault();redo();}else if(e.key==='Enter'&&drawMode==='boundary'&&!editing){finishBoundary();}else if(e.key==='Escape'&&(drawMode!=='select')){$('cancelDrawBtn').click();}});

  function pointerXY(e){const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];}
  function nearestVertex(pos){let best=null,bd=13;for(const r of project.regions){for(let i=0;i<r.boundary.length;i++){const s=toScreen(r.boundary[i]),d=Math.hypot(s[0]-pos[0],s[1]-pos[1]);if(d<bd){best={region:r,index:i};bd=d;}}}return best;}
  canvas.addEventListener('pointerdown',e=>{
    const pos=pointerXY(e),world=screenToMap(...pos);
    if(e.shiftKey&&drawMode==='select'){
      const vertex=nearestVertex(pos);
      if(vertex){if(vertex.region.boundary.length<=3){setBanner('区域边界至少保留3个顶点。');return;}snapshot();vertex.region.boundary.splice(vertex.index,1);selectedRegion=vertex.region.region_id;planned.delete(selectedRegion);renderAll();return;}
    }
    if(e.button===2||e.button===1){drag={kind:'pan',x:pos[0],y:pos[1],px:panX,py:panY};canvas.setPointerCapture(e.pointerId);return;}
    if(drawMode==='boundary'){draft.push(world);draw();return;}
    if(drawMode==='charger-position'){
      pendingCharger={phase:'heading',point:world,preview:world,replace:selectedCharger};drawMode='charger-heading';setModeButtons();draw();return;
    }
    if(drawMode==='charger-heading'){
      const dx=world[0]-pendingCharger.point[0],dy=world[1]-pendingCharger.point[1];if(Math.hypot(dx,dy)<.12){setBanner('朝向箭头太短，请在至少12厘米外点击。');return;}
      snapshot();let c;if(pendingCharger.replace){c=project.chargers.find(x=>x.charger_id===pendingCharger.replace);c.staging_pose={x:pendingCharger.point[0],y:pendingCharger.point[1],yaw:Math.atan2(dy,dx)};}else{let n=1;while(project.chargers.some(c=>c.charger_id===`charger${n}`))n++;c={charger_id:`charger${n}`,display_name:`充电点${n}`,staging_pose:{x:pendingCharger.point[0],y:pendingCharger.point[1],yaw:Math.atan2(dy,dx)}};project.chargers.push(c);}selectedCharger=c.charger_id;selectedRegion=null;pendingCharger=null;drawMode='select';setModeButtons();renderAll();setBanner('充电staging位置已标注，导出时会检查footprint安全裕度。',false);return;
    }
    const vertex=nearestVertex(pos);if(vertex){selectedRegion=vertex.region.region_id;selectedCharger=null;drag={kind:'vertex',...vertex};snapshot();canvas.setPointerCapture(e.pointerId);renderAll();return;}
    const containing=[...project.regions].reverse().find(r=>pointInside(world,r.boundary));
    if(containing){selectedRegion=containing.region_id;selectedCharger=null;renderAll();return;}
    drag={kind:'pan',x:pos[0],y:pos[1],px:panX,py:panY};canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove',e=>{
    const pos=pointerXY(e),world=screenToMap(...pos);$('coords').textContent=`光标坐标：x ${world[0].toFixed(3)} m, y ${world[1].toFixed(3)} m (map)`;
    if(drawMode==='charger-heading'&&pendingCharger){pendingCharger.preview=world;draw();}
    if(drag?.kind==='pan'){panX=drag.px+pos[0]-drag.x;panY=drag.py+pos[1]-drag.y;draw();}
    if(drag?.kind==='vertex'){const r=project.regions.find(x=>x.region_id===drag.region.region_id);r.boundary[drag.index]=world;invalidatePlan();}
  });
  canvas.addEventListener('pointerup',()=>{if(drag?.kind==='vertex'){renderAll();}drag=null;});
  canvas.addEventListener('pointercancel',()=>{drag=null;});canvas.addEventListener('contextmenu',e=>e.preventDefault());
  canvas.addEventListener('wheel',e=>{e.preventDefault();const [x,y]=pointerXY(e),before=screenToMap(x,y),factor=e.deltaY<0?1.12:.89;zoom=Math.max(.6,Math.min(8,zoom*factor));const after=toScreen(before);panX+=x-after[0];panY+=y-after[1];draw();},{passive:false});
  window.addEventListener('resize',resize);

  $('saveProjectBtn').onclick=()=>download(`${safeFilename(info.identity.map_id)}_editor_project.json`,project);
  $('loadProjectBtn').onclick=()=>$('fileInput').click();
  $('fileInput').onchange=async e=>{const file=e.target.files[0];if(!file)return;try{const parsed=JSON.parse(await file.text());
      if(parsed.project_schema==='cleannav.map_editor_project.v1'){
        if(!sameIdentity(parsed.map,info.identity))throw new Error('项目绑定的地图身份不匹配。请先切换到该项目对应的原地图。');
        snapshot();project=parsed;planned.clear();selectedRegion=project.regions[0]?.region_id||null;selectedCharger=null;draft=[];pendingCharger=null;drawMode='select';setModeButtons();renderAll();setBanner('编辑项目已加载。自动路线需要重新计算确认。',false);
      }else if(parsed.schema_version===1&&parsed.map&&Array.isArray(parsed.regions)){
        const result=await api('/api/validate-registry',parsed);if(!sameIdentity(parsed.map,info.identity))throw new Error('Registry地图身份不匹配。');
        snapshot();project={project_schema:'cleannav.map_editor_project.v1',map:clone(parsed.map),regions:parsed.regions.map(r=>({region_id:r.region_id,display_name:r.display_name||r.region_id,route_id:r.routes?.[0]?.route_id||'perimeter',boundary:r.boundary,options:{margin:.15,direction:'ccw',laps:3}})),chargers:parsed.chargers||[]};planned.clear();selectedRegion=project.regions[0]?.region_id||null;selectedCharger=null;draft=[];pendingCharger=null;drawMode='select';setModeButtons();renderAll();setBanner(`Mission Registry已通过真实RegionRegistry加载验证后打开；路线需重算。${result.loaded_by}`,false);
      }else throw new Error('文件既不是本工具编辑项目，也不是Mission Registry v1。');
    }catch(err){setBanner(err.message);}finally{e.target.value='';}};
  $('exportBtn').onclick=async()=>{try{if(!project.regions.length)throw new Error('至少需要一个区域。');const result=await api('/api/export',project);download(`${safeFilename(info.identity.map_id)}_region_registry.json`,result.registry);download(`${safeFilename(info.identity.map_id)}_route_validation.json`,{status:result.status,map:result.registry.map,regions:result.validation});setBanner('Registry已通过JSON Schema及Mission RegionRegistry加载验证。规划报告标明仅为离线几何验证。',false);}catch(e){setBanner(`无法导出：${e.message}`);}};
  $('mapSelect').onchange=async e=>{if((project.regions.length||project.chargers.length)&&!confirm('切换地图会放弃当前未保存编辑，继续吗？')){e.target.value=window.currentMapKey;return;}try{await api('/api/load-map',{key:e.target.value});window.currentMapKey=e.target.value;project=null;await loadInfo();setBanner('地图已切换。当前地图身份已绑定。',false);}catch(err){setBanner(err.message);}};
  async function init(){try{await loadMapChoices();window.currentMapKey=$('mapSelect').value;await loadInfo();$('regionList').addEventListener('click',()=>{});setModeButtons();refreshUndo();}catch(e){setBanner(`地图加载失败：${e.message}`);}}
  function pointInside(p,poly){let inside=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){const [xi,yi]=poly[i],[xj,yj]=poly[j];const cross=((yi>p[1])!==(yj>p[1]))&&(p[0]<(xj-xi)*(p[1]-yi)/(yj-yi+1e-30)+xi);if(cross)inside=!inside;}return inside;}
  init();
};
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', initializeMapEditor, {once:true});
else initializeMapEditor();
