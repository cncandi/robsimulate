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
    var wrap = document.getElementById('fv-form-wrap');
    if (wrap) wrap.style.display = 'flex';
    var tb = document.getElementById('fv-toolbar');
    if (tb && !tb.innerHTML.trim()) fvToolbarInit();
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'flex';
    fvBuild(fvExpandedLine);
  },
  deactivate: function () {
    var wrap = document.getElementById('fv-form-wrap');
    if (wrap) wrap.style.display = 'none';
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

// ── Systemvariablen + Sonderbefehle ──────────────────
var FV_SYSVARS = [
  // Konfiguration
  { rx: /^\$BASE\s*=\s*BASE_DATA\[(\d+)\]/i,  id:'base',    label:'Koordinatensystem', unit:'#',    color:'#4488cc', min:1, max:30,  step:1,    toKRL:function(v){ return '$BASE = BASE_DATA['+Math.round(v)+']'; } },
  { rx: /^\$TOOL\s*=\s*TOOL_DATA\[(\d+)\]/i,  id:'tool',    label:'TCP / Werkzeug',    unit:'#',    color:'#4488cc', min:1, max:30,  step:1,    toKRL:function(v){ return '$TOOL=TOOL_DATA['+Math.round(v)+']'; } },
  { rx: /^\$advance\s*=\s*([\d.]+)/i,          id:'advance', label:'Vorlauf',           unit:'Pkt',  color:'#4488cc', min:0, max:5,   step:1,    toKRL:function(v){ return '$advance='+Math.round(v); } },
  // Geschwindigkeit
  { rx: /^\$VEL\.CP\s*=\s*([\d.]+)/i,          id:'velcp',   label:'Geschwindigkeit',   unit:'m/s',  color:'#f05500', min:0.001, max:0.167, step:0.001, toKRL:function(v){ return '$VEL.CP='+parseFloat(v).toFixed(3); } },
  { rx: /^\$VEL\.PTP\s*=\s*([\d.]+)/i,         id:'velptp',  label:'PTP-Geschwindigkeit', unit:'%',  color:'#f05500', min:1, max:100, step:1,    toKRL:function(v){ return '$VEL.PTP='+Math.round(v); } },
  { rx: /^\$ACC\.CP\s*=\s*([\d.]+)/i,          id:'acccp',   label:'Beschleunigung',    unit:'m/s²', color:'#f05500', min:0.1, max:10, step:0.1,  toKRL:function(v){ return '$ACC.CP='+parseFloat(v).toFixed(1); } },
  // Warten / Stopp
  { rx: /^WAIT\s+SEC\s+([\d.]+)/i,             id:'wait',    label:'Warten',            unit:'s',    color:'#00aa88', min:0, max:60,  step:0.1,  toKRL:function(v){ return 'WAIT SEC '+parseFloat(v).toFixed(1); } },
  { rx: /^HALT$/i,                              id:'halt',    label:'Programm anhalten', unit:'',     color:'#cc4444', min:null, max:null, step:null, toKRL:function(v){ return 'HALT'; }, noValue:true },
  // Digitale I/O
  { rx: /^\$OUT\[(\d+)\]\s*=\s*(TRUE|FALSE|ON|OFF|1|0)/i, id:'dout', label:'Digitaler Ausgang', unit:'', color:'#cc8800', min:1, max:100, step:1,
    parse2: function(t){ var m=t.match(/^\$OUT\[(\d+)\]\s*=\s*(TRUE|FALSE|ON|OFF|1|0)/i); return m?{n:m[1],v:m[2]}:null; },
    toKRL: function(v){ return v; } },
  { rx: /^\$IN\[(\d+)\]/i,                     id:'din',     label:'Digitaler Eingang', unit:'',     color:'#8888cc', min:1, max:100, step:1,
    parse2: function(t){ var m=t.match(/^\$IN\[(\d+)\]/i); return m?{n:m[1]}:null; },
    toKRL: function(v){ return v; }, readOnly:true },
  // Analoge I/O
  { rx: /^\$ANOUT\[(\d+)\]\s*=\s*([\d.\-+]+)/i, id:'aout',  label:'Analoger Ausgang',  unit:'V',    color:'#cc8800', min:1, max:100, step:1,
    parse2: function(t){ var m=t.match(/^\$ANOUT\[(\d+)\]\s*=\s*([\d.\-+]+)/i); return m?{n:m[1],v:m[2]}:null; },
    toKRL: function(v){ return v; } },
  { rx: /^\$ANIN\[(\d+)\]/i,                   id:'ain',     label:'Analoger Eingang',  unit:'V',    color:'#8888cc', min:1, max:100, step:1,
    parse2: function(t){ var m=t.match(/^\$ANIN\[(\d+)\]/i); return m?{n:m[1]}:null; },
    toKRL: function(v){ return v; }, readOnly:true },
  // Variablen
  { rx: /^(INT|REAL|BOOL|CHAR)\s+(\w+)\s*(?:=\s*(.+))?/i, id:'var', label:'Variable', unit:'', color:'#8844cc',
    parse2: function(t){ var m=t.match(/^(INT|REAL|BOOL|CHAR)\s+(\w+)\s*(?:=\s*(.+))?/i); return m?{type:m[1],name:m[2],val:m[3]||''}:null; },
    toKRL: function(v){ return v; } },
];

function fvParseSysVar(raw) {
  var t = raw.trim();
  for (var i = 0; i < FV_SYSVARS.length; i++) {
    var sv = FV_SYSVARS[i];
    var m = t.match(sv.rx);
    if (m) {
      // Einfache Sysvars: 1 Capture-Gruppe = Wert
      if (sv.noValue) return { sysvar: sv, value: '', raw: t };
      if (sv.parse2) return { sysvar: sv, value: t, raw: t };
      return { sysvar: sv, value: m[1], raw: t };
    }
  }
  return null;
}

// ── Sysvar Formular rendern ──────────────────────────
function fvSVFormHTML(sv, value, i) {
  var html = '<div class="fv-form">';
  html += '<div class="fv-section-hdr"><div class="fv-section-num" style="background:'+sv.sysvar.color+'">1</div>'
        + '<span class="fv-section-lbl" style="color:'+sv.sysvar.color+'">'+sv.sysvar.label.toUpperCase()+'</span>'
        + '<div class="fv-section-line" style="background:'+sv.sysvar.color+'40"></div></div>';

  if (sv.sysvar.noValue) {
    // HALT: kein Input
    html += '<div class="fv-ctrl-row"><span class="fv-coord-unit">Stoppt das Programm — kein Parameter</span></div>';
  } else if (sv.sysvar.parse2) {
    // Komplexe Sysvars (I/O, Variablen): Freitext
    html += '<div class="fv-ctrl-row">';
    html += '<span class="fv-coord-lbl" style="min-width:auto">Code</span>';
    html += '<input class="fv-inp" id="fv-sv-'+i+'" type="text" value="'+fvEsc(value)+'">';
    html += '</div>';
  } else if (sv.sysvar.min !== null) {
    // Slider + Nummer
    var numVal = parseFloat(value) || sv.sysvar.min;
    html += '<div class="fv-ctrl-row" style="flex-direction:column;align-items:stretch;gap:6px">';
    html += '<div style="display:flex;align-items:center;gap:8px">';
    html += '<input class="fv-slider" id="fv-sv-sl-'+i+'" type="range"'
          + ' min="'+sv.sysvar.min+'" max="'+sv.sysvar.max+'" step="'+sv.sysvar.step+'"'
          + ' value="'+numVal+'"'
          + ' oninput="document.getElementById(\'fv-sv-'+i+'\').value=parseFloat(this.value).toFixed('+fvSliderDecimals(sv.sysvar.step)+')"'
          + ' style="flex:1;accent-color:'+sv.sysvar.color+'">';
    html += '<input class="fv-inp" id="fv-sv-'+i+'" type="number"'
          + ' min="'+sv.sysvar.min+'" max="'+sv.sysvar.max+'" step="'+sv.sysvar.step+'"'
          + ' value="'+numVal+'"'
          + ' oninput="document.getElementById(\'fv-sv-sl-'+i+'\').value=this.value"'
          + ' style="max-width:90px">';
    html += '<span class="fv-coord-unit">'+sv.sysvar.unit+'</span>';
    html += '</div>';
    html += '<div style="display:flex;justify-content:space-between;font-size:.8em;color:var(--txt3)">'
          + '<span>'+sv.sysvar.min+'</span><span>'+sv.sysvar.max+'</span></div>';
    html += '</div>';
  }

  html += '<div class="fv-acts">'
        + '<button class="fv-btn fv-apply" onclick="fvApplySV('+i+')">✓ Übernehmen</button>'
        + '<button class="fv-btn fv-del" onclick="fvDelete('+i+')">✕ Löschen</button>'
        + '</div>';
  html += '</div>';
  return html;
}

function fvSliderDecimals(step) {
  var s = String(step);
  var dot = s.indexOf('.');
  return dot < 0 ? 0 : s.length - dot - 1;
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
      html += '<div class="fv-head" onclick="fvSetCursor('+i+');fvToggle('+i+')">';
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
        var svColor = sv.sysvar.color || '#4488cc';
        html += '<div class="fv-card fv-sv-card'+(isExpSv?' fv-open':'')+'" data-line="'+i+'" style="border-left-color:'+svColor+'">';
        html += '<div class="fv-head" onclick="fvSetCursor('+i+');fvToggle('+i+')">';
        html += '<span class="fv-num">'+(i+1)+'</span>';
        html += '<span class="fv-badge fv-badge-sv" style="background:'+svColor+'40;color:'+svColor+'">'+sv.sysvar.id.toUpperCase()+'</span>';
        html += '<span class="fv-sv-label" style="color:#ccc">'+sv.sysvar.label+'</span>';
        if (!sv.sysvar.noValue) html += '<span class="fv-sv-val">'+fvEsc(sv.value)+'</span>';
        if (sv.sysvar.unit) html += '<span class="fv-verl-tag">'+sv.sysvar.unit+'</span>';
        html += '<span class="fv-chevron">'+(isExpSv?'▲':'▼')+'</span>';
        html += '</div>';
        if (isExpSv) html += fvSVFormHTML(sv, sv.value, i);
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

// ══════════════════════════════════════════════════════
// TOOLBAR
// ══════════════════════════════════════════════════════
var fvToolbarInsertAfter = -1; // Zeile nach der eingefügt wird (-1 = ans Ende)

function fvToolbarInit() {
  var tb = document.getElementById('fv-toolbar');
  if (!tb) return;

  // SVG Icons als Inline-Helper
  function ico(paths, color) {
    return '<svg width="22" height="22" viewBox="0 0 22 22" fill="none" xmlns="http://www.w3.org/2000/svg">'
      + paths.map(function(p){ return '<path d="'+p.d+'" stroke="'+(p.c||color||'#f05500')+'" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"'+(p.fill?' fill="'+p.fill+'"':'')+'/>' }).join('')
      + '</svg>';
  }

  var ICONS = {
    move:    ico([{d:'M4 11h14'},{d:'M15 7l4 4-4 4'},{d:'M4 7v8'}]),
    flow:    ico([{d:'M11 4v14'},{d:'M7 8l4-4 4 4'},{d:'M5 16h12'}], '#00aa88'),
    process: ico([{d:'M4 11h14'},{d:'M11 4v14'},{d:'M6 6l10 10'},{d:'M16 6L6 16'}], '#f05500'),
    digital: ico([{d:'M4 7h3v8H4z',fill:'rgba(80,80,200,.3)'},{d:'M9 7h3v8H9z',fill:'rgba(80,80,200,.15)'},{d:'M15 11h3'},{d:'M15 8v6'}], '#8888ee'),
    analog:  ico([{d:'M4 14 Q7 4 11 11 Q15 18 18 8'}], '#cc8800'),
    data:    ico([{d:'M5 6h12v3H5z',fill:'rgba(100,60,200,.3)'},{d:'M5 11h12v3H5z',fill:'rgba(100,60,200,.15)'},{d:'M8 17h6'}], '#8844cc'),
  };

  var groups = [
    { id:'move',    icon: ICONS.move,    label:'Bewegung', color:'#f05500',
      items: [
        { label:'PTP',  badge:'PTP',  bc:'rgba(255,170,0,.9)', bcolor:'#000', insert: 'PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000}' },
        { label:'LIN',  badge:'LIN',  bc:'#f05500',            bcolor:'#fff', insert: 'LIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000} C_DIS' },
        { label:'SLIN', badge:'SLIN', bc:'rgba(0,170,255,.8)', bcolor:'#fff', insert: 'SLIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000} C_DIS' },
        { label:'CIRC', badge:'CIRC', bc:'rgba(170,68,255,.8)',bcolor:'#fff', insert: 'CIRC {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000}, {X 100.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000} C_DIS' },
      ]
    },
    { id:'flow', icon: ICONS.flow, label:'Ablauf', color:'#00aa88',
      items: [
        { label:'HALT', badge:'HALT', bc:'#cc4444', bcolor:'#fff', insert: 'HALT' },
        { label:'WAIT', badge:'WAIT', bc:'#00aa88', bcolor:'#fff', insert: 'WAIT SEC 1.0' },
      ]
    },
    { id:'process', icon: ICONS.process, label:'Prozess', color:'#f05500',
      items: [
        { label:'VEL.CP',  badge:'VEL',  bc:'#f05500', bcolor:'#fff', insert: '$VEL.CP=0.167' },
        { label:'VEL.PTP', badge:'VPTP', bc:'#f05500', bcolor:'#fff', insert: '$VEL.PTP=100' },
        { label:'ACC.CP',  badge:'ACC',  bc:'rgba(240,85,0,.6)', bcolor:'#fff', insert: '$ACC.CP=1.0' },
      ]
    },
    { id:'digital', icon: ICONS.digital, label:'Digital', color:'#8888ee',
      items: [
        { label:'Eingang', badge:'DIN',  bc:'#8888cc', bcolor:'#fff', insert: '; $IN[1]  ; lesen: IF $IN[1] THEN ...' },
        { label:'Ausgang', badge:'DOUT', bc:'#cc8800', bcolor:'#fff', insert: '$OUT[1]=TRUE' },
      ]
    },
    { id:'analog', icon: ICONS.analog, label:'Analog', color:'#cc8800',
      items: [
        { label:'Eingang', badge:'AIN',  bc:'#8888cc', bcolor:'#fff', insert: '; $ANIN[1]  ; lesen in Variable' },
        { label:'Ausgang', badge:'AOUT', bc:'#cc8800', bcolor:'#fff', insert: '$ANOUT[1]=0.0' },
      ]
    },
    { id:'data', icon: ICONS.data, label:'Daten', color:'#8844cc',
      items: [
        { label:'TOOL',  badge:'TOOL', bc:'#4488cc', bcolor:'#fff', insert: '$TOOL=TOOL_DATA[24]' },
        { label:'BASE',  badge:'BASE', bc:'#4488cc', bcolor:'#fff', insert: '$BASE = BASE_DATA[1]' },
        { label:'INT',   badge:'INT',  bc:'#8844cc', bcolor:'#fff', insert: 'INT myVar=0' },
        { label:'REAL',  badge:'REAL', bc:'#8844cc', bcolor:'#fff', insert: 'REAL myVal=0.0' },
        { label:'BOOL',  badge:'BOOL', bc:'#8844cc', bcolor:'#fff', insert: 'BOOL myFlag=FALSE' },
      ]
    },
  ];

  var html = '';
  groups.forEach(function(g, gi) {
    if (gi > 0) html += '<div class="fvtb-sep"></div>';
    html += '<div class="fvtb-btn" id="fvtb-'+g.id+'" onclick="fvTbToggle(\''+g.id+'\')">';
    html += g.icon;
    html += '<span>'+g.label+'</span>';
    html += '<div class="fvtb-menu" id="fvtb-menu-'+g.id+'">';
    g.items.forEach(function(item, ii) {
      var key = g.id + '_' + ii;
      fvTbTemplates[key] = item.insert;
      html += '<div class="fvtb-item" onclick="fvTbInsert(\'' + key + '\',event)">'
            + '<span class="fvtb-item-badge" style="background:' + item.bc + ';color:' + item.bcolor + '">' + item.badge + '</span>'
            + item.label + '</div>';
    });
    html += '</div></div>';
  });

  tb.innerHTML = html;

  // Klick außerhalb schließt Submenüs
  document.addEventListener('click', function(e) {
    if (!e.target.closest('#fv-toolbar')) {
      document.querySelectorAll('.fvtb-btn.open').forEach(function(b){ b.classList.remove('open'); });
    }
  });
}

function fvTbToggle(id) {
  var btn = document.getElementById('fvtb-'+id);
  if (!btn) return;
  var wasOpen = btn.classList.contains('open');
  document.querySelectorAll('.fvtb-btn.open').forEach(function(b){ b.classList.remove('open'); });
  if (!wasOpen) btn.classList.add('open');
}

var fvTbTemplates = {};

function fvTbInsert(key, e) {
  if (e) { e.stopPropagation(); e.preventDefault(); }
  document.querySelectorAll('.fvtb-btn.open').forEach(function(b){ b.classList.remove('open'); });
  var krl = fvTbTemplates[key];
  if (!krl) return;
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var after = fvToolbarInsertAfter >= 0 ? fvToolbarInsertAfter : lines.length - 1;
  // Vor END einfügen
  while (after >= 0 && lines[after] && lines[after].trim().toUpperCase() === 'END') after--;
  lines.splice(after + 1, 0, krl);
  ta.value = lines.join('\n');
  fvExpandedLine = after + 1;
  fvBuild(fvExpandedLine);
}

// Toolbar ein/ausblenden mit Formular
FormatRegistry.register._origKukaForm = FormatRegistry.register._origKukaForm;
(function patchActivate() {
  var fmts = FormatRegistry._formats || [];
  // Suche kuka-form Format direkt
})();

function fvSetCursor(lineIdx) {
  fvToolbarInsertAfter = lineIdx;
  // Visuell markieren welche Zeile als Einfügeposition gilt
  document.querySelectorAll('#krl-form-view [data-line]').forEach(function(el){
    el.classList.remove('fv-cursor-line');
  });
  var el = document.querySelector('#krl-form-view [data-line="'+lineIdx+'"]');
  if (el) el.classList.add('fv-cursor-line');
}
