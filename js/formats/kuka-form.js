// ══════════════════════════════════════════════════════
// FORMAT: KUKA Formular — Infografik-Stil
// ══════════════════════════════════════════════════════

var fvExpandedLine      = -1;
var fvPTPSubtypeOverride = {};

FormatRegistry.register({
  id:    'kuka-form',
  label: 'Formular',
  activate: function () {
    document.getElementById('code-input').style.display = 'none';
    document.getElementById('gutter').style.display     = 'none';
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'flex';
    fvBuild(fvExpandedLine);
  },
  deactivate: function () {
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'none';
    document.getElementById('code-input').style.display = '';
    document.getElementById('gutter').style.display     = '';
  }
});

// ── Parser ──────────────────────────────────────────
function fvParseLine(raw) {
  var t = raw.trim();
  var mC = t.match(/^CIRC\s+\{([^}]+)\}\s*,\s*\{([^}]+)\}\s*(C_DIS|C_PTP|C_VEL|C_ORI)?\s*$/i);
  if (mC) return { moveType:'CIRC', subtype:'circ', aux:parsePos(mC[1]), end:parsePos(mC[2]), verl:mC[3]||'' };
  var mL = t.match(/^(LIN|SLIN)\s+\{([^}]+)\}\s*(C_DIS|C_PTP|C_VEL|C_ORI)?\s*$/i);
  if (mL) return { moveType:mL[1].toUpperCase(), subtype:'cart', pos:parsePos(mL[2]), verl:mL[3]||'' };
  var mP = t.match(/^PTP\s+\{([^}]+)\}\s*(C_PTP)?\s*$/i);
  if (mP) {
    if (/A[123456]\s/.test(mP[1])) {
      var ap={A1:0,A2:0,A3:0,A4:0,A5:0,A6:0}, am, rx=/A([123456])\s+([-+]?\d+(?:\.\d+)?)/g;
      while((am=rx.exec(mP[1]))!==null) ap['A'+am[1]]=parseFloat(am[2]);
      return { moveType:'PTP', subtype:'axis', pos:ap, verl:mP[2]||'' };
    }
    return { moveType:'PTP', subtype:'cart', pos:parsePos(mP[1]), verl:mP[2]||'' };
  }
  return null;
}

// ── Serialisierung ───────────────────────────────────
function fvToKRL(data) {
  var v = data.verl ? ' '+data.verl : '';
  if (data.moveType==='CIRC') return 'CIRC '+fvFmtPos(data.aux)+', '+fvFmtPos(data.end)+v;
  if (data.subtype==='axis') {
    var p=data.pos;
    return 'PTP {A1 '+fvN(p.A1)+', A2 '+fvN(p.A2)+', A3 '+fvN(p.A3)+', A4 '+fvN(p.A4)+', A5 '+fvN(p.A5)+', A6 '+fvN(p.A6)+'}'+v;
  }
  return data.moveType+' '+fvFmtPos(data.pos)+v;
}
function fvFmtPos(p) { return '{X '+fvN(p.X)+', Y '+fvN(p.Y)+', Z '+fvN(p.Z)+', A '+fvN(p.A)+', B '+fvN(p.B)+', C '+fvN(p.C)+'}'; }
function fvN(n) { return parseFloat(n||0).toFixed(3); }

// ── Systemvariablen ───────────────────────────────────
var FV_SYSVARS = [
  { rx: /^\$BASE\s*=\s*BASE_DATA\[(\d+)\]/i,  id:'base',    label:'Koordinatensystem', unit:'BASE_DATA[N]', toKRL:function(v){ return '$BASE = BASE_DATA['+v+']'; } },
  { rx: /^\$TOOL\s*=\s*TOOL_DATA\[(\d+)\]/i,  id:'tool',    label:'TCP / Werkzeug',    unit:'TOOL_DATA[N]', toKRL:function(v){ return '$TOOL=TOOL_DATA['+v+']'; } },
  { rx: /^\$advance\s*=\s*([\d.]+)/i,          id:'advance', label:'Vorlauf',           unit:'Punkte',       toKRL:function(v){ return '$advance='+v; } },
  { rx: /^\$VEL\.CP\s*=\s*([\d.]+)/i,          id:'velcp',   label:'Geschwindigkeit',   unit:'m/s',          toKRL:function(v){ return '$VEL.CP='+v; } },
  { rx: /^\$VEL\.PTP\s*=\s*([\d.]+)/i,         id:'velptp',  label:'PTP-Geschw.',       unit:'%',            toKRL:function(v){ return '$VEL.PTP='+v; } },
  { rx: /^\$ACC\.CP\s*=\s*([\d.]+)/i,          id:'acccp',   label:'Beschleunigung',    unit:'m/s²',         toKRL:function(v){ return '$ACC.CP='+v; } },
];
function fvParseSysVar(raw) {
  var t = raw.trim();
  for (var i = 0; i < FV_SYSVARS.length; i++) {
    var m = t.match(FV_SYSVARS[i].rx);
    if (m) return { sysvar: FV_SYSVARS[i], value: m[1] };
  }
  return null;
}

// ── Koordinaten-Vorschau für Kopfzeile ───────────────
function fvPreviewHTML(mv) {
  var html = '<div class="fv-preview">';
  if (mv.subtype === 'axis') {
    var p = mv.pos;
    ['A1','A2','A3','A4','A5','A6'].forEach(function(ax) {
      html += '<div class="fv-pv-group"><span class="fv-pv-lbl">'+ax+'</span><span class="fv-pv-val">'+fvN(p[ax])+'°</span></div>';
    });
  } else if (mv.subtype === 'circ') {
    var e = mv.end;
    ['X','Y','Z'].forEach(function(f) {
      html += '<div class="fv-pv-group"><span class="fv-pv-lbl">'+f+'</span><span class="fv-pv-val">'+fvN(e[f])+'</span></div>';
    });
    html += '<span class="fv-pv-lbl" style="margin-left:4px;opacity:.5">CIRC</span>';
  } else {
    var p = mv.pos;
    ['X','Y','Z','A','B','C'].forEach(function(f) {
      html += '<div class="fv-pv-group"><span class="fv-pv-lbl">'+f+'</span><span class="fv-pv-val">'+fvN(p[f])+'</span></div>';
    });
  }
  html += '</div>';
  return html;
}

// ── View aufbauen ────────────────────────────────────
function fvBuild(expandLine) {
  var fv = document.getElementById('krl-form-view');
  if (!fv || fv.style.display === 'none') return;
  var lines = document.getElementById('code-input').value.split(/\r?\n/);
  var html  = '';
  var moveIdx = 0;

  for (var i = 0; i < lines.length; i++) {
    var raw = lines[i];
    var mv  = fvParseLine(raw);

    if (mv) {
      if (mv.moveType==='PTP' && fvPTPSubtypeOverride[i]!==undefined)
        mv.subtype = fvPTPSubtypeOverride[i];

      var isExp = (expandLine === i);
      moveIdx++;

      html += '<div class="fv-card'+(isExp?' fv-open':'')+'" data-line="'+i+'">';
      // Kopfzeile
      html += '<div class="fv-head" onclick="fvToggle('+i+')">';
      html += '<span class="fv-num">'+moveIdx+'</span>';
      html += '<span class="fv-badge fv-badge-'+mv.moveType.toLowerCase()+'">'+mv.moveType+'</span>';
      html += fvPreviewHTML(mv);
      if (mv.verl) html += '<span class="fv-verl-tag">'+mv.verl+'</span>';
      html += '<span class="fv-chevron">'+(isExp?'▲':'▼')+'</span>';
      html += '</div>';

      if (isExp) {
        html += '<div class="fv-form">';
        html += fvFormHTML(mv, i);
        html += '<div class="fv-acts">'
              + '<button class="fv-btn fv-apply" onclick="fvApply('+i+')">✓ Übernehmen</button>'
              + '<button class="fv-btn fv-del"   onclick="fvDelete('+i+')">✕ Löschen</button>'
              + '</div>';
        html += '</div>';
      }
      html += '</div>';

    } else {
      var sv = fvParseSysVar(raw);
      if (sv) {
        var isExpSv = (expandLine === i);
        html += '<div class="fv-card fv-sv-card'+(isExpSv?' fv-open':'')+'" data-line="'+i+'">';
        html += '<div class="fv-head" onclick="fvToggle('+i+')">';
        html += '<span class="fv-num">'+(i+1)+'</span>';
        html += '<span class="fv-badge fv-badge-sv">'+sv.sysvar.id.toUpperCase()+'</span>';
        html += '<span class="fv-sv-label">'+sv.sysvar.label+'</span>';
        html += '<span class="fv-sv-val">'+fvEsc(sv.value)+'</span>';
        html += '<span class="fv-verl-tag">'+sv.sysvar.unit+'</span>';
        html += '<span class="fv-chevron">'+(isExpSv?'▲':'▼')+'</span>';
        html += '</div>';
        if (isExpSv) {
          html += '<div class="fv-form">';
          html += '<div class="fv-section-hdr"><div class="fv-section-num">1</div>'
                + '<span class="fv-section-lbl">'+sv.sysvar.label+'</span>'
                + '<div class="fv-section-line"></div></div>';
          html += '<div class="fv-ctrl-row">';
          html += '<span class="fv-coord-lbl" style="min-width:auto">Wert</span>';
          html += '<input class="fv-inp" id="fv-sv-'+i+'" type="text" value="'+fvEsc(sv.value)+'" style="max-width:160px">';
          html += '<span class="fv-coord-unit">'+sv.sysvar.unit+'</span>';
          html += '</div>';
          html += '<div class="fv-acts">'
                + '<button class="fv-btn fv-apply" onclick="fvApplySV('+i+')">✓ Übernehmen</button>'
                + '<button class="fv-btn fv-del" onclick="fvDelete('+i+')">✕ Löschen</button>'
                + '</div>';
          html += '</div>';
        }
        html += '</div>';
      } else {
        var cls = raw.trim().startsWith(';') ? 'fv-comment' :
                  raw.trim() === '' ? 'fv-empty' : 'fv-plain';
        html += '<div class="fv-row '+cls+'" data-line="'+i+'">';
        html += '<span class="fv-lineno">'+(i+1)+'</span>';
        if (cls !== 'fv-empty')
          html += '<span class="fv-plaintext">'+fvEsc(raw.trim())+'</span>';
        html += '</div>';
      }
    }

    html += '<div class="fv-insert-bar" onclick="fvInsertNew('+i+')">'
          + '<span class="fv-insert-btn">+ Neu</span></div>';
  }

  fv.innerHTML = html;
  if (expandLine >= 0) {
    var el = fv.querySelector('[data-line="'+expandLine+'"]');
    if (el) el.scrollIntoView({ block:'nearest' });
  }
}

// ── Formular-HTML ────────────────────────────────────
function fvFormHTML(mv, i) {
  var html = '';
  var verlOpts = mv.moveType==='PTP' ? ['','C_PTP'] : ['','C_DIS','C_VEL','C_ORI'];

  // Sektion 1: Befehl
  html += '<div class="fv-section">';
  html += '<div class="fv-section-hdr"><div class="fv-section-num">1</div>'
        + '<span class="fv-section-lbl">Befehlstyp</span>'
        + '<div class="fv-section-line"></div></div>';
  html += '<div class="fv-ctrl-row">';
  html += '<span class="fv-ctrl-lbl">Typ</span>';
  html += '<select class="fv-sel" id="fv-type-'+i+'" onchange="fvTypeChange('+i+')">';
  ['LIN','PTP','SLIN','CIRC'].forEach(function(tp){
    html += '<option value="'+tp+'"'+(mv.moveType===tp?' selected':'')+'>'+tp+'</option>';
  });
  html += '</select>';
  html += '<span class="fv-ctrl-lbl" style="margin-left:10px">Verschleifung</span>';
  html += '<select class="fv-sel" id="fv-verl-'+i+'">';
  verlOpts.forEach(function(vo){
    html += '<option value="'+vo+'"'+(mv.verl===vo?' selected':'')+'>'+(vo||'—')+'</option>';
  });
  html += '</select>';
  if (mv.moveType==='PTP') {
    html += '<span class="fv-ctrl-lbl" style="margin-left:10px">Modus</span>';
    html += '<button class="fv-toggle-btn'+(mv.subtype==='cart'?' fv-on':'')+'" onclick="fvPTPToggle('+i+',\'cart\')">X/Y/Z</button>';
    html += '<button class="fv-toggle-btn'+(mv.subtype==='axis'?' fv-on':'')+'" onclick="fvPTPToggle('+i+',\'axis\')">A1–A6</button>';
  }
  html += '</div>';
  html += '</div>';

  // Sektion 2: Position
  html += '<div class="fv-section">';
  if (mv.subtype === 'circ') {
    html += '<div class="fv-section-hdr"><div class="fv-section-num">2</div>'
          + '<span class="fv-section-lbl">Hilfspunkt</span>'
          + '<div class="fv-section-line"></div></div>';
    html += '<div class="fv-coord-grid">'+fvCartInputs(mv.aux,'aux',i)+'</div>';
    html += '<div class="fv-section-hdr" style="margin-top:6px"><div class="fv-section-num">3</div>'
          + '<span class="fv-section-lbl">Endpunkt</span>'
          + '<div class="fv-section-line"></div></div>';
    html += '<div class="fv-coord-grid">'+fvCartInputs(mv.end,'end',i)+'</div>';
  } else if (mv.subtype === 'axis') {
    html += '<div class="fv-section-hdr"><div class="fv-section-num">2</div>'
          + '<span class="fv-section-lbl">Achswinkel</span>'
          + '<div class="fv-section-line"></div></div>';
    html += '<div class="fv-axis-grid">';
    ['A1','A2','A3','A4','A5','A6'].forEach(function(ax){
      html += '<div class="fv-coord-row"><span class="fv-coord-lbl">'+ax+'</span>'
            + '<input class="fv-inp" id="fv-'+ax+'-'+i+'" type="number" step="0.001" value="'+fvN(mv.pos[ax])+'">'
            + '<span class="fv-coord-unit">°</span></div>';
    });
    html += '</div>';
  } else {
    html += '<div class="fv-section-hdr"><div class="fv-section-num">2</div>'
          + '<span class="fv-section-lbl">Position &amp; Orientierung</span>'
          + '<div class="fv-section-line"></div></div>';
    html += '<div class="fv-coord-grid">'+fvCartInputs(mv.pos,'pos',i)+'</div>';
  }
  html += '</div>';

  return html;
}

function fvCartInputs(p, prefix, i) {
  var fields = [['X','mm'],['A','°'],['Y','mm'],['B','°'],['Z','mm'],['C','°']];
  return fields.map(function(f){
    return '<div class="fv-coord-row"><span class="fv-coord-lbl">'+f[0]+'</span>'
         + '<input class="fv-inp" id="fv-'+prefix+'-'+f[0]+'-'+i+'" type="number" step="0.1" value="'+fvN(p[f[0]])+'">'
         + '<span class="fv-coord-unit">'+f[1]+'</span></div>';
  }).join('');
}

// ── Event-Handler ────────────────────────────────────
function fvToggle(lineIdx) {
  fvExpandedLine = (fvExpandedLine===lineIdx) ? -1 : lineIdx;
  fvBuild(fvExpandedLine);
}

function fvPTPToggle(lineIdx, subtype) {
  fvPTPSubtypeOverride[lineIdx] = subtype;
  fvExpandedLine = lineIdx;
  fvBuild(lineIdx);
}

function fvTypeChange(lineIdx) {
  var typeSel = document.getElementById('fv-type-'+lineIdx);
  var verlSel = document.getElementById('fv-verl-'+lineIdx);
  if (!typeSel) return;
  var newType = typeSel.value;
  var curVerl = verlSel ? verlSel.value : '';
  var opts = newType==='PTP' ? ['','C_PTP'] : ['','C_DIS','C_VEL','C_ORI'];
  if (opts.indexOf(curVerl)<0) curVerl='';
  if (verlSel) {
    verlSel.innerHTML='';
    opts.forEach(function(vo){ var o=document.createElement('option'); o.value=vo; o.textContent=vo||'—'; if(vo===curVerl)o.selected=true; verlSel.appendChild(o); });
  }
  if (newType!=='PTP') delete fvPTPSubtypeOverride[lineIdx];
  var ta = document.getElementById('code-input');
  var mv = fvParseLine(ta.value.split(/\r?\n/)[lineIdx]||'');
  if (!mv) return;
  mv.moveType = newType;
  if (newType==='CIRC') { mv.subtype='circ'; mv.aux=mv.aux||{X:0,Y:0,Z:0,A:0,B:0,C:0}; mv.end=mv.end||mv.pos||{X:0,Y:0,Z:0,A:0,B:0,C:0}; }
  else if (newType!=='PTP') mv.subtype='cart';
  if (fvPTPSubtypeOverride[lineIdx]) mv.subtype=fvPTPSubtypeOverride[lineIdx];
  var formDiv = document.querySelector('#krl-form-view [data-line="'+lineIdx+'"] .fv-form');
  if (!formDiv) return;
  formDiv.innerHTML = fvFormHTML(mv,lineIdx)
    + '<div class="fv-acts"><button class="fv-btn fv-apply" onclick="fvApply('+lineIdx+')">✓ Übernehmen</button>'
    + '<button class="fv-btn fv-del" onclick="fvDelete('+lineIdx+')">✕ Löschen</button></div>';
}

function fvReadData(lineIdx) {
  var ta = document.getElementById('code-input');
  var mv = fvParseLine(ta.value.split(/\r?\n/)[lineIdx]||'');
  if (!mv) return null;
  var typeSel=document.getElementById('fv-type-'+lineIdx);
  var verlSel=document.getElementById('fv-verl-'+lineIdx);
  mv.moveType = typeSel ? typeSel.value : mv.moveType;
  mv.verl     = verlSel ? verlSel.value : mv.verl;
  if (mv.moveType==='PTP' && fvPTPSubtypeOverride[lineIdx]) mv.subtype=fvPTPSubtypeOverride[lineIdx];
  if (mv.moveType==='CIRC') mv.subtype='circ';
  function readCart(prefix) {
    var p={};
    ['X','Y','Z','A','B','C'].forEach(function(f){ var inp=document.getElementById('fv-'+prefix+'-'+f+'-'+lineIdx); p[f]=inp?parseFloat(inp.value)||0:0; });
    return p;
  }
  if (mv.subtype==='axis') {
    var ap={};
    ['A1','A2','A3','A4','A5','A6'].forEach(function(ax){ var inp=document.getElementById('fv-'+ax+'-'+lineIdx); ap[ax]=inp?parseFloat(inp.value)||0:0; });
    mv.pos=ap;
  } else if (mv.subtype==='circ') { mv.aux=readCart('aux'); mv.end=readCart('end'); }
  else mv.pos=readCart('pos');
  return mv;
}

function fvApplySV(lineIdx) {
  var ta    = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var sv    = fvParseSysVar(lines[lineIdx]);
  if (!sv) return;
  var inp = document.getElementById('fv-sv-'+lineIdx);
  if (!inp) return;
  lines[lineIdx] = sv.sysvar.toKRL(inp.value.trim());
  ta.value = lines.join('\n');
  fvExpandedLine = -1;
  fvBuild(-1);
}

function fvApply(lineIdx) {
  var data=fvReadData(lineIdx); if (!data) return;
  var ta=document.getElementById('code-input');
  var lines=ta.value.split(/\r?\n/);
  lines[lineIdx]=fvToKRL(data);
  ta.value=lines.join('\n');
  delete fvPTPSubtypeOverride[lineIdx];
  fvExpandedLine=-1; fvBuild(-1);
}

function fvDelete(lineIdx) {
  var ta=document.getElementById('code-input');
  var lines=ta.value.split(/\r?\n/);
  lines.splice(lineIdx,1); ta.value=lines.join('\n');
  delete fvPTPSubtypeOverride[lineIdx];
  fvExpandedLine=-1; fvBuild(-1);
}

function fvInsertNew(afterLineIdx) {
  var ta=document.getElementById('code-input');
  var lines=ta.value.split(/\r?\n/);
  lines.splice(afterLineIdx+1,0,'LIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000}');
  ta.value=lines.join('\n');
  fvExpandedLine=afterLineIdx+1; fvBuild(fvExpandedLine);
}

function fvEsc(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}
