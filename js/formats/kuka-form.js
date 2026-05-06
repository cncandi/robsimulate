// ══════════════════════════════════════════════════════
// FORMAT: KUKA Formular — Infografik-Stil
// ══════════════════════════════════════════════════════

var fvExpandedLine      = -1;
var fvPTPSubtypeOverride = {};
var fvGroupCollapsed     = {};  // {startLine: bool}

FormatRegistry.register({
  id:    'kuka-form',
  label: 'Formular',
  icon:  '<svg viewBox="0 0 16 16" width="14" height="14" style="vertical-align:middle;margin-right:5px;opacity:.85">'
       + '<rect x="1" y="1" width="14" height="14" rx="2" fill="none" stroke="#8ab4d4" stroke-width="1.2"/>'
       + '<line x1="4" y1="5" x2="12" y2="5" stroke="#8ab4d4" stroke-width="1.2"/>'
       + '<line x1="4" y1="8" x2="12" y2="8" stroke="#8ab4d4" stroke-width="1.2"/>'
       + '<line x1="4" y1="11" x2="9" y2="11" stroke="#8ab4d4" stroke-width="1.2"/>'
       + '</svg>',
  activate: function () {
    // code-input intern auf KRL-Stand bringen (fvBuild liest daraus)
    // Das ist nur interne Zwischenebene — kein anderes Format liest code-input zurück
    var ci = document.getElementById('code-input');
    if (ci && typeof generateKRL === 'function' && typeof parsedData !== 'undefined') {
      var krl = generateKRL(parsedData);
      if (krl) ci.value = krl;
    }
    ci.style.display = 'none';
    document.getElementById('gutter').style.display = 'none';
    var wrap = document.getElementById('fv-form-wrap');
    if (wrap) wrap.style.display = 'flex';
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
  },

  // kuka-form ist kanonische Quelle — parsedData ist bereits aktuell
  // Kein _parse nötig; Registry überspringt Parse wenn _parse fehlt
  _parse: null
});

// ── Parser ──────────────────────────────────────────
function fvParseGroup(raw) {
  var t = raw.trim();
  var mg = t.match(/^;\s*#GROUP\s*(.*)/i);
  if (mg) return { type:'group', name: mg[1].trim() || 'Gruppe' };
  if (/^;\s*#ENDGROUP/i.test(t)) return { type:'endgroup' };
  return null;
}

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
  // Kommentar (muss als erstes stehen damit ; zuerst erkannt wird)
  { rx: /^;\s*(.*)$/, id:'comment', label:'Kommentar', color:'#666666',
    getVal: function(t){ var m=t.match(/^;[ 	]*(.*)/); return {text:m?m[1]:''}; },
    toKRL:  function(text){ return '; ' + text; },
    ioType: 'comment' },
  // Konfiguration
  { rx: /^\$BASE\s*=\s*BASE_DATA\[(\d+)\]/i,  id:'base',    label:'Koordinatensystem', unit:'#',    color:'#4488cc', min:1, max:30,  step:1,    toKRL:function(v){ return '$BASE = BASE_DATA['+Math.round(v)+']'; } },
  { rx: /^\$TOOL\s*=\s*TOOL_DATA\[(\d+)\]/i,  id:'tool',    label:'TCP / Werkzeug',    unit:'#',    color:'#4488cc', min:1, max:30,  step:1,    toKRL:function(v){ return '$TOOL=TOOL_DATA['+Math.round(v)+']'; } },
  { rx: /^\$advance\s*=\s*([\d.]+)/i,          id:'advance', label:'Vorlauf',           unit:'Pkt',  color:'#4488cc', min:0, max:5,   step:1,    toKRL:function(v){ return '$advance='+Math.round(v); } },
  // Geschwindigkeit
  { rx: /^\$VEL\.CP\s*=\s*([\d.]+)/i, id:'velcp', badge:'SPEED', label:'Geschwindigkeit',
    unit:'mm/min', color:'#f05500', min:1000, max:10000, step:10,
    scale: 60000,
    getVal: function(t){ var m=t.match(/[\d.]+$/); return m ? Math.round(parseFloat(m[0])*60000) : 6000; },
    toKRL: function(v){ return '$VEL.CP='+Math.max(0.0001,(parseFloat(v)||1000)/60000).toFixed(4); },
    displayVal: function(v){ return Math.round(parseFloat(v)*60000)+' mm/min'; } },
  { rx: /^\$VEL\.PTP\s*=\s*([\d.]+)/i,         id:'velptp',  label:'PTP-Geschwindigkeit', unit:'%',  color:'#f05500', min:1, max:100, step:1,    toKRL:function(v){ return '$VEL.PTP='+Math.round(v); } },
  { rx: /^\$ACC\.CP\s*=\s*([\d.]+)/i,          id:'acccp',   label:'Beschleunigung',    unit:'m/s²', color:'#f05500', min:0.1, max:10, step:0.1,  toKRL:function(v){ return '$ACC.CP='+parseFloat(v).toFixed(1); } },
  // Warten / Stopp
  { rx: /^WAIT\s+SEC\s+([\d.]+)/i,             id:'wait',    label:'Warten',            unit:'s',    color:'#00aa88', min:0, max:60,  step:0.1,  toKRL:function(v){ return 'WAIT SEC '+parseFloat(v).toFixed(1); } },
  { rx: /^HALT$/i,                              id:'halt',    label:'Programm anhalten', unit:'',     color:'#cc4444', min:null, max:null, step:null, toKRL:function(v){ return 'HALT'; }, noValue:true },
  // Digitale I/O
  { rx: /^\$OUT\[(\d+)\]\s*=\s*(TRUE|FALSE|ON|OFF|1|0)/i, id:'dout', label:'Digitaler Ausgang', color:'#cc8800',
    getVal: function(t){ var m=t.match(/^\$OUT\[(\d+)\]\s*=\s*(TRUE|FALSE|ON|OFF|1|0)/i); return m?{n:parseInt(m[1]),v:m[2].toUpperCase()}:{n:1,v:'TRUE'}; },
    toKRL: function(n,v){ return '$OUT['+n+']='+v; },
    ioType:'dout' },
  { rx: /^;?\s*\$IN\[(\d+)\]/i, id:'din', label:'Digitaler Eingang', color:'#8888cc',
    getVal: function(t){ var m=t.match(/^;?\s*\$IN\[(\d+)\]/i); return m?{n:parseInt(m[1])}:{n:1}; },
    toKRL: function(n){ return '$IN['+n+']'; },
    ioType:'din' },
  // Analoge I/O
  { rx: /^\$ANOUT\[(\d+)\]\s*=\s*([\d.\-+]+)/i, id:'aout', label:'Analoger Ausgang', unit:'V', color:'#cc8800',
    getVal: function(t){ var m=t.match(/^\$ANOUT\[(\d+)\]\s*=\s*([\d.\-+]+)/i); return m?{n:parseInt(m[1]),v:parseFloat(m[2])}:{n:1,v:0}; },
    toKRL: function(n,v){ return '$ANOUT['+n+']='+parseFloat(v).toFixed(2); },
    ioType:'aout' },
  { rx: /^;?\s*\$ANIN\[(\d+)\]/i, id:'ain', label:'Analoger Eingang', unit:'V', color:'#8888cc',
    getVal: function(t){ var m=t.match(/^;?\s*\$ANIN\[(\d+)\]/i); return m?{n:parseInt(m[1])}:{n:1}; },
    toKRL: function(n){ return '$ANIN['+n+']'; },
    ioType:'ain' },
  // Variablen
  { rx: /^(INT|REAL|BOOL|CHAR)\s+(\w+)\s*(?:=\s*(.+))?/i, id:'var', label:'Variable', color:'#8844cc',
    getVal: function(t){ var m=t.match(/^(INT|REAL|BOOL|CHAR)\s+(\w+)\s*(?:=\s*(.+))?/i); return m?{type:m[1],name:m[2],val:m[3]||''}:{type:'INT',name:'myVar',val:'0'}; },
    toKRL: function(type,name,val){ return val?type+' '+name+'='+val:type+' '+name; },
    ioType:'var' },
  // CALC: Zuweisung/Berechnung von INT- oder REAL-Variablen
  // Regex: Bezeichner = Ausdruck (kein $, kein Typ-Schlüsselwort vorne)
  { rx: /^([a-zA-Z_][a-zA-Z0-9_]*)\s*=\s*(.+)$/, id:'calc', label:'Berechnung', color:'#aa7700',
    getVal: function(t){ var m=t.match(/^([a-zA-Z_][a-zA-Z0-9_]*)\s*=\s*(.+)$/); return m?{target:m[1],expr:m[2].trim()}:{target:'myVar',expr:'0.0'}; },
    toKRL: function(target,expr){ return target+' = '+expr; },
    ioType:'calc' },
];

function fvParseSysVar(raw) {
  var t = raw.trim();
  for (var i = 0; i < FV_SYSVARS.length; i++) {
    var sv = FV_SYSVARS[i];
    if (!t.match(sv.rx)) continue;
    var m = t.match(sv.rx);
    if (sv.noValue)  return { sysvar: sv, value: '', raw: t };
    if (sv.ioType)   return { sysvar: sv, value: t, raw: t, io: sv.getVal(t) };
    return { sysvar: sv, value: m[1], raw: t };
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
    // HALT: Info-Text, kein Input — trotzdem fv-sv-{i} als Hidden damit fvApplySV funktioniert
    html += '<div class="fv-ctrl-row"><span class="fv-coord-unit">Stoppt das Programm — kein Parameter</span></div>';
    html += '<input type="hidden" id="fv-sv-'+i+'" value="HALT">';
  } else if (sv.sysvar.ioType) {
    // I/O und Variablen: strukturiertes Formular
    var io = sv.io || (sv.sysvar.getVal ? sv.sysvar.getVal(value) : {});
    var ioT = sv.sysvar.ioType;
    html += '<div class="fv-ctrl-row">';
    if (ioT === 'dout' || ioT === 'din' || ioT === 'aout' || ioT === 'ain') {
      html += '<span class="fv-coord-lbl" style="min-width:auto">Nr.</span>';
      html += '<input class="fv-inp" id="fv-sv-n-'+i+'" type="number" min="1" max="100" step="1" value="'+(io.n||1)+'" style="max-width:70px">';
    }
    if (ioT === 'dout') {
      html += '<span class="fv-coord-lbl" style="min-width:auto;margin-left:10px">Status</span>';
      var doutOn = io.v==='TRUE'||io.v==='ON'||io.v==='1';
      html += '<select class="fv-sel" id="fv-sv-v-'+i+'">'
            + '<option value="TRUE"'+(doutOn?' selected':'')+'>AN</option>'
            + '<option value="FALSE"'+(!doutOn?' selected':'')+'>AUS</option>'
            + '</select>';
    }
    if (ioT === 'aout') {
      html += '<span class="fv-coord-lbl" style="min-width:auto;margin-left:10px">-10</span>';
      html += '<input class="fv-slider" id="fv-sv-v-'+i+'" type="range" min="-10" max="10" step="0.1" value="'+(io.v||0)+'" oninput="document.getElementById(\'fv-sv-vn-'+i+'\').textContent=parseFloat(this.value).toFixed(1)+\' V\'" style="flex:1;accent-color:#cc8800">';
      html += '<span id="fv-sv-vn-'+i+'" style="min-width:48px;color:var(--acc);font-size:.9em;text-align:right">'+(io.v||0)+' V</span>';
      html += '<span class="fv-coord-lbl" style="min-width:auto">+10</span>';
    }
    if (ioT === 'din') {
      html += '<span class="fv-coord-unit" style="margin-left:8px;padding:2px 8px;border-radius:3px;background:rgba(136,136,204,.15);color:#aac">Status: nur Lesen</span>';
    }
    if (ioT === 'ain') {
      html += '<span class="fv-coord-lbl" style="margin-left:8px;min-width:auto">Wert</span>';
      html += '<input class="fv-inp" id="fv-sv-v-'+i+'" type="number" min="-10" max="10" step="0.1" value="0" style="max-width:70px">';
      html += '<span class="fv-coord-unit">V</span>';
    }
    html += '<input type="hidden" id="fv-sv-'+i+'" value="'+fvEsc(value)+'">';
    html += '</div>';
    if (ioT === 'var') {
      var io2 = sv.sysvar.getVal(value);
      html += '<div class="fv-ctrl-row">';
      html += '<span class="fv-coord-lbl" style="min-width:auto">Typ</span>';
      html += '<select class="fv-sel" id="fv-sv-type-'+i+'">';
      ['INT','REAL','BOOL','CHAR'].forEach(function(tp){
        html += '<option'+(io2.type&&io2.type.toUpperCase()===tp?' selected':'')+'>'+tp+'</option>';
      });
      html += '</select>';
      html += '<span class="fv-coord-lbl" style="margin-left:8px">Name</span>';
      html += '<input class="fv-inp" id="fv-sv-name-'+i+'" type="text" value="'+fvEsc(io2.name||'')+'" style="max-width:120px">';
      html += '<span class="fv-coord-lbl" style="margin-left:8px">= </span>';
      html += '<input class="fv-inp" id="fv-sv-val-'+i+'" type="text" value="'+fvEsc(io2.val||'')+'" style="max-width:90px">';
      html += '</div>';
    }
    if (ioT === 'comment') {
      var io0 = sv.sysvar.getVal(value);
      html += '<div class="fv-ctrl-row">';
      html += '<input class="fv-inp" id="fv-sv-txt-'+i+'" type="text" value="'+fvEsc(io0.text||'')+'" style="flex:1" placeholder="Kommentartext...">';
      html += '</div>';
    }
    if (ioT === 'calc') {
      var io3 = sv.sysvar.getVal(value);
      // Verfügbare INT/REAL Variablen aus vorherigen Zeilen sammeln
      var declVars = fvGetCalcVars(i);
      html += '<div class="fv-ctrl-row">';
      html += '<span class="fv-coord-lbl" style="min-width:auto">Variable</span>';
      html += '<select class="fv-sel" id="fv-sv-target-'+i+'">';
      if (declVars.length === 0) html += '<option value="'+fvEsc(io3.target)+'">'+fvEsc(io3.target)+'</option>';
      declVars.forEach(function(v){
        html += '<option value="'+v+'"'+(v===io3.target?' selected':'')+'>'+v+'</option>';
      });
      html += '</select>';
      html += '<span class="fv-coord-lbl" style="margin-left:8px">=</span>';
      html += '<input class="fv-inp" id="fv-sv-expr-'+i+'" type="text" value="'+fvEsc(io3.expr||'')+'" style="flex:1;min-width:120px" placeholder="Ausdruck: z.B. myVar * 2.0 + 1">';
      html += '</div>';
      if (declVars.length > 0) {
        html += '<div style="color:var(--txt3);font-size:.78em;margin-top:4px">Verfügbare Variablen: '+declVars.join(", ")+'</div>';
      } else {
        html += '<div style="color:#cc6600;font-size:.78em;margin-top:4px">⚠ Keine INT/REAL Variable davor deklariert</div>';
      }
    }
  } else if (sv.sysvar.min !== null) {
    // Slider + Nummer
    var scale = sv.sysvar.scale || 1;
    var numVal = Math.round((parseFloat(value) || sv.sysvar.min/scale) * scale);
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
  // Gruppen-Struktur vorberechnen: welche Zeilen gehören zu welcher Gruppe
  var lineGroupStart = {};  // lineIdx → startLine der Gruppe
  var groupStack = [];
  for (var gi = 0; gi < lines.length; gi++) {
    var gp = fvParseGroup(lines[gi]);
    if (gp && gp.type === 'group') { groupStack.push(gi); }
    else if (gp && gp.type === 'endgroup' && groupStack.length) {
      var gs = groupStack.pop();
      for (var gj = gs+1; gj < gi; gj++) lineGroupStart[gj] = gs;
      lineGroupStart[gi] = gs; // ENDGROUP gehört auch zur Gruppe
    }
  }

  var skipUntil = -1; // Zeilen überspringen wenn Gruppe collapsed

  for (var i = 0; i < lines.length; i++) {
    var raw = lines[i];
    var mv  = fvParseLine(raw);

    // Gruppe: Header rendern
    var grpData = fvParseGroup(raw);
    if (grpData && grpData.type === 'group') {
      skipUntil = -1; // Reset
      var collapsed = !!fvGroupCollapsed[i];
      html += '<div class="fv-group-hdr" data-line="'+i+'">';
      html += '<span class="fv-group-arrow" onclick="fvGroupToggle('+i+')">'+(collapsed?'▶':'▼')+'</span>';
      html += '<span class="fv-group-name" ondblclick="event.stopPropagation();fvGroupRename('+i+',this)" onclick="fvGroupToggle('+i+')">'+fvEsc(grpData.name)+'</span>';
      html += '<div class="fv-row-acts" onclick="event.stopPropagation()">'
            + '<button class="fv-row-btn" title="Nach oben" onclick="fvMoveGroup('+i+',-1)">↑</button>'
            + '<button class="fv-row-btn" title="Nach unten" onclick="fvMoveGroup('+i+',1)">↓</button>'
            + '<button class="fv-row-btn fv-row-del" title="Gruppe löschen" onclick="fvDeleteGroup('+i+')">✕</button>'
            + '</div>';
      html += '</div>';
      if (collapsed) {
        // Alle Zeilen bis ENDGROUP überspringen — Zähler merken
        for (var si = i+1; si < lines.length; si++) {
          var sg = fvParseGroup(lines[si]);
          if (sg && sg.type === 'endgroup') { skipUntil = si; break; }
        }
      }
      html += '<div class="fv-insert-bar" onclick="fvInsertMenu('+i+',event)"><span class="fv-insert-btn">+</span></div>';
      continue;
    }
    if (grpData && grpData.type === 'endgroup') {
      if (skipUntil >= i) { skipUntil = -1; html += '<div class="fv-insert-bar" onclick="fvInsertMenu('+i+',event)"><span class="fv-insert-btn">+</span></div>'; continue; }
      skipUntil = -1;
      html += '<div class="fv-group-end" data-line="'+i+'">'
            + '<span onclick="fvInsertMenu('+i+',event)" style="flex:1">▪ ENDGROUP</span>'
            + '<div class="fv-row-acts" onclick="event.stopPropagation()">'
            + '<button class="fv-row-btn" onclick="fvMoveRow('+i+',-1)">↑</button>'
            + '<button class="fv-row-btn" onclick="fvMoveRow('+i+',1)">↓</button>'
            + '</div>'
            + '</div>';
      html += '<div class="fv-insert-bar" onclick="fvInsertMenu('+i+',event)"><span class="fv-insert-btn">+</span></div>';
      continue;
    }
    // Überspringen wenn in collapsed Gruppe
    if (skipUntil >= 0 && i <= skipUntil) continue;

    if (mv) {
      if (mv.moveType==='PTP' && fvPTPSubtypeOverride[i]!==undefined)
        mv.subtype = fvPTPSubtypeOverride[i];

      var isExp = (expandLine === i);
      html += '<div class="fv-card'+(isExp?' fv-open':'')+'" data-line="'+i+'">';
      // Kopfzeile
      html += '<div class="fv-head" onclick="fvSetCursor('+i+');fvToggle('+i+')">';
      html += '<span class="fv-dh" title="Verschieben (Drag)">⠿</span>';
      html += '<span class="fv-num">'+(i+1)+'</span>';
      html += '<span class="fv-badge fv-badge-'+mv.moveType.toLowerCase()+'">'+mv.moveType+'</span>';
      html += fvPreviewHTML(mv);
      if (mv.verl) html += '<span class="fv-verl-tag">'+mv.verl+'</span>';
      html += '<span class="fv-chevron">'+(isExp?'▲':'▼')+'</span>';
      html += '<div class="fv-row-acts" onclick="event.stopPropagation()">'
            + '<button class="fv-row-btn" title="Nach oben" onclick="fvMoveRow('+i+',-1)">↑</button>'
            + '<button class="fv-row-btn" title="Nach unten" onclick="fvMoveRow('+i+',1)">↓</button>'
            + '<button class="fv-row-btn fv-row-del" title="Löschen" onclick="fvDelete('+i+')">✕</button>'
            + '</div>';
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
        html += '<span class="fv-dh" title="Verschieben (Drag)">⠿</span>';
        html += '<span class="fv-num">'+(i+1)+'</span>';
        html += '<span class="fv-badge fv-badge-sv" style="background:'+svColor+'40;color:'+svColor+'">'+(sv.sysvar.badge||sv.sysvar.id.toUpperCase())+'</span>';
        html += '<span class="fv-sv-label">'+sv.sysvar.label+'</span>';
        if (!sv.sysvar.noValue) {
          var dispVal = sv.value;
          if (sv.sysvar.displayVal) { dispVal = sv.sysvar.displayVal(sv.value); }
          else if (sv.sysvar.ioType && sv.io) {
            var io = sv.io; var ioT2 = sv.sysvar.ioType;
            if (io.n !== undefined) dispVal = '['+io.n+']';
            if (io.v !== undefined) {
              var isDig = ioT2==='dout'||ioT2==='din';
              var vOn = io.v==='TRUE'||io.v==='ON'||io.v==='1';
              dispVal += ' = '+(isDig?(vOn?'AN':'AUS'):io.v);
            }
          }
          html += '<span class="fv-sv-val">'+fvEsc(dispVal)+'</span>';
        }
        if (sv.sysvar.unit) html += '<span class="fv-verl-tag">'+sv.sysvar.unit+'</span>';
        html += '<span class="fv-chevron">'+(isExpSv?'▲':'▼')+'</span>';
        html += '<div class="fv-row-acts" onclick="event.stopPropagation()">'
              + '<button class="fv-row-btn" title="Nach oben" onclick="fvMoveRow('+i+',-1)">↑</button>'
              + '<button class="fv-row-btn" title="Nach unten" onclick="fvMoveRow('+i+',1)">↓</button>'
              + '<button class="fv-row-btn fv-row-del" title="Löschen" onclick="fvDelete('+i+')">✕</button>'
              + '</div>';
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

    html += '<div class="fv-insert-bar" onclick="fvInsertMenu('+i+',event)">'
          + '<span class="fv-insert-btn">+</span></div>';
  }

  fv.innerHTML = html;
  if (expandLine >= 0) {
    var el = fv.querySelector('[data-line="'+expandLine+'"]');
    if (el) el.scrollIntoView({ block:'nearest' });
  }
  // Drag & Drop nach dem Rendern anbinden
  _fvBindDrag(fv);
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
          + '<div class="fv-section-line"></div>'
          + '<button class="fv-pos-btn" onclick="fvTakeRobotPos(\'end\','+i+')">⊕ Aktuelle Position</button>'
          + '</div>';
    html += '<div class="fv-coord-grid">'+fvCartInputs(mv.end,'end',i)+'</div>';
  } else if (mv.subtype === 'axis') {
    html += '<div class="fv-section-hdr"><div class="fv-section-num">2</div>'
          + '<span class="fv-section-lbl">Achswinkel</span>'
          + '<div class="fv-section-line"></div>'
          + '<button class="fv-pos-btn" onclick="fvTakeRobotPos(\'axis\','+i+')">⊕ Aktuelle Achsen</button>'
          + '</div>';
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
          + '<div class="fv-section-line"></div>'
          + '<button class="fv-pos-btn" onclick="fvTakeRobotPos(\'cart\','+i+')">⊕ Aktuelle Position</button>'
          + '</div>';
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
// ── Insert-Menü ──────────────────────────────────────
var fvInsertMenuLine = -1;

function fvInsertMenu(afterLine, e) {
  e.stopPropagation();
  fvInsertMenuLine = afterLine;

  // Bestehendes Popup entfernen
  var old = document.getElementById('fv-insert-popup');
  if (old) { old.remove(); if (parseInt(old.dataset.line) === afterLine) return; }

  var bar = e.currentTarget;
  var rect = bar.getBoundingClientRect();
  var fv   = document.getElementById('krl-form-view');
  var fvR  = fv.getBoundingClientRect();

  var popup = document.createElement('div');
  popup.id = 'fv-insert-popup';
  popup.dataset.line = afterLine;
  popup.className = 'fv-insert-popup';

  var groups = [
    { label:'Bewegung', color:'#f05500', items:[
      { badge:'PTP',  bc:'rgba(255,170,0,.9)', bcolor:'#000', key:'move_0' },
      { badge:'LIN',  bc:'#f05500',            bcolor:'#fff', key:'move_1' },
      { badge:'SLIN', bc:'rgba(0,170,255,.8)', bcolor:'#fff', key:'move_2' },
      { badge:'CIRC', bc:'rgba(170,68,255,.8)',bcolor:'#fff', key:'move_3' },
    ]},
    { label:'Ablauf',   color:'#00aa88', items:[
      { badge:'HALT', bc:'#cc4444', bcolor:'#fff', key:'flow_0' },
      { badge:'WAIT', bc:'#00aa88', bcolor:'#fff', key:'flow_1' },
    ]},
    { label:'Prozess',  color:'#f05500', items:[
      { badge:'VEL',  bc:'#f05500', bcolor:'#fff', key:'process_0' },
      { badge:'VPTP', bc:'rgba(240,85,0,.6)', bcolor:'#fff', key:'process_1' },
      { badge:'ACC',  bc:'rgba(240,85,0,.4)', bcolor:'#fff', key:'process_2' },
    ]},
    { label:'Digital',  color:'#8888ee', items:[
      { badge:'DIN',  bc:'#8888cc', bcolor:'#fff', key:'digital_0' },
      { badge:'DOUT', bc:'#cc8800', bcolor:'#fff', key:'digital_1' },
    ]},
    { label:'Analog',   color:'#cc8800', items:[
      { badge:'AIN',  bc:'#8888cc', bcolor:'#fff', key:'analog_0' },
      { badge:'AOUT', bc:'#cc8800', bcolor:'#fff', key:'analog_1' },
    ]},
    { label:'Daten',    color:'#8844cc', items:[
      { badge:'TOOL', bc:'#4488cc', bcolor:'#fff', key:'data_0' },
      { badge:'BASE', bc:'#4488cc', bcolor:'#fff', key:'data_1' },
      { badge:'INT',  bc:'#8844cc', bcolor:'#fff', key:'data_2' },
      { badge:'REAL', bc:'#8844cc', bcolor:'#fff', key:'data_3' },
      { badge:'BOOL', bc:'#8844cc', bcolor:'#fff', key:'data_4' },
      { badge:'CALC', bc:'#aa7700', bcolor:'#fff', key:'data_5' },
      { badge:';',    bc:'#666666', bcolor:'#fff', key:'data_6' },
    ]},
    { label:'Gruppe',   color:'#449966', items:[
      { badge:'GRP',  bc:'#336644', bcolor:'#aaffcc', key:'group_0' },
      { badge:'END',  bc:'#224433', bcolor:'#aaffcc', key:'group_1' },
    ]},
  ];

  var html = '';
  groups.forEach(function(g) {
    html += '<div class="fvip-section"><span class="fvip-lbl" style="color:'+g.color+'">'+g.label+'</span><div class="fvip-items">';
    g.items.forEach(function(item) {
      html += '<div class="fvip-item" onclick="fvTbInsertAt(\'' + item.key + '\', ' + afterLine + ')">'
            + '<span class="fvtb-item-badge" style="background:'+item.bc+';color:'+item.bcolor+'">'+item.badge+'</span>'
            + '</div>';
    });
    html += '</div></div>';
  });
  popup.innerHTML = html;

  // Positionierung: unterhalb der Insert-Bar
  popup.style.position = 'fixed';
  popup.style.top  = (rect.bottom + 2) + 'px';
  popup.style.left = Math.max(4, rect.left) + 'px';
  document.body.appendChild(popup);

  // Klick außerhalb schließt
  setTimeout(function() {
    document.addEventListener('click', function close(ev) {
      if (!popup.contains(ev.target)) { popup.remove(); document.removeEventListener('click', close); }
    });
  }, 0);
}

function fvTbInsertAt(key, afterLine) {
  var old = document.getElementById('fv-insert-popup');
  if (old) old.remove();
  var krl = fvTbTemplates[key];
  if (!krl) return;
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var after = afterLine;
  lines.splice(after + 1, 0, krl);
  ta.value = lines.join('\n');
  fvExpandedLine = after + 1;
  fvBuild(fvExpandedLine);
}

// Drag & Drop state
var _fvDragSrc = -1;
function fvDragStart(e, idx) {
  _fvDragSrc = idx;
  e.dataTransfer.effectAllowed = 'move';
  e.currentTarget.style.opacity = '0.4';
}
function fvDragOver(e) {
  e.preventDefault();
  e.dataTransfer.dropEffect = 'move';
  e.currentTarget.style.outline = '2px solid var(--acc)';
}
function fvDragLeave(e) {
  e.currentTarget.style.outline = '';
}
function fvDrop(e, targetIdx) {
  e.preventDefault();
  e.currentTarget.style.outline = '';
  document.querySelectorAll('[draggable]').forEach(function(el){ el.style.opacity=''; });
  if (_fvDragSrc < 0 || _fvDragSrc === targetIdx) return;
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var src = lines.splice(_fvDragSrc, 1)[0];
  var dst = targetIdx > _fvDragSrc ? targetIdx - 1 : targetIdx;
  lines.splice(dst, 0, src);
  ta.value = lines.join('\n');
  _fvDragSrc = -1;
  if (typeof parseAndLoad === 'function') parseAndLoad();
  fvBuild(dst);
}


// ── Drag & Drop — Container-Handler mit Einfügelinie ─────────
var _fvDragSrcIdx = -1;
var _fvDropLine   = null;

function _fvGetDropLine() {
  if (!_fvDropLine) {
    _fvDropLine = document.createElement('div');
    _fvDropLine.className = 'fv-drop-line';
    _fvDropLine.style.pointerEvents = 'none';
  }
  return _fvDropLine;
}
function _fvClearDropLine() {
  if (_fvDropLine && _fvDropLine.parentNode)
    _fvDropLine.parentNode.removeChild(_fvDropLine);
}

// Gibt die Karte und "before/after" zur Mausposition y zurück
function _fvCardAtY(fv, y) {
  var best = null, bestBefore = true, bestDist = Infinity;
  var cards = fv.querySelectorAll('.fv-card[data-line], .fv-group-hdr[data-line], .fv-group-end[data-line]');
  cards.forEach(function(card) {
    var r = card.getBoundingClientRect();
    var mid = r.top + r.height / 2;
    var dist = Math.abs(y - mid);
    if (dist < bestDist) {
      bestDist  = dist;
      best      = card;
      bestBefore = y < mid;
    }
  });
  return best ? { card: best, before: bestBefore } : null;
}

function _fvBindDrag(fv) {
  // Handle — macht die Karte draggable
  fv.querySelectorAll('.fv-dh').forEach(function(handle) {
    handle.addEventListener('mousedown', function() {
      handle.closest('[data-line]').draggable = true;
    });
  });

  // Dragstart auf jeder Karte
  fv.querySelectorAll('[data-line]').forEach(function(card) {
    card.addEventListener('dragstart', function(e) {
      if (!card.draggable) return;
      _fvDragSrcIdx = parseInt(card.getAttribute('data-line'));
      e.dataTransfer.effectAllowed = 'move';
      setTimeout(function() { card.classList.add('fv-dragging'); }, 0);
    });
    card.addEventListener('dragend', function() {
      card.draggable = false;
      card.classList.remove('fv-dragging');
      _fvClearDropLine();
      _fvDragSrcIdx = -1;
    });
  });

  // Einziger dragover-Handler auf dem Container
  fv.addEventListener('dragover', function(e) {
    if (_fvDragSrcIdx < 0) return;
    e.preventDefault();
    e.dataTransfer.dropEffect = 'move';

    var hit = _fvCardAtY(fv, e.clientY);
    if (!hit) return;

    var dl = _fvGetDropLine();
    fv.insertBefore(dl, hit.before ? hit.card : hit.card.nextSibling);
  });

  fv.addEventListener('dragleave', function(e) {
    if (!fv.contains(e.relatedTarget)) _fvClearDropLine();
  });

  // Drop ebenfalls auf Container
  fv.addEventListener('drop', function(e) {
    e.preventDefault();
    var src = _fvDragSrcIdx;
    _fvClearDropLine();
    _fvDragSrcIdx = -1;
    if (src < 0) return;

    var hit = _fvCardAtY(fv, e.clientY);
    if (!hit) return;

    var targetIdx = parseInt(hit.card.getAttribute('data-line'));
    var dst = hit.before ? targetIdx : targetIdx + 1;

    // No-op: selbe oder direkt nachfolgende Position
    if (src === dst || src + 1 === dst) return;

    var ta    = document.getElementById('code-input');
    var lines = ta.value.split(/\r?\n/);
    var moved = lines.splice(src, 1)[0];
    var realDst = dst > src ? dst - 1 : dst;
    lines.splice(realDst, 0, moved);
    ta.value = lines.join('\n');
    if (typeof parseAndLoad === 'function') parseAndLoad();
    fvBuild(realDst);
  });
}
function fvMoveRow(lineIdx, dir) {
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var target = lineIdx + dir;
  if (target < 0 || target >= lines.length) return;
  var tmp = lines[lineIdx];
  lines[lineIdx] = lines[target];
  lines[target] = tmp;
  ta.value = lines.join('\n');
  // Expanded line mitbewegen
  if (fvExpandedLine === lineIdx) fvExpandedLine = target;
  else if (fvExpandedLine === target) fvExpandedLine = lineIdx;
  fvBuild(fvExpandedLine);
}

function fvToggle(lineIdx) {
  var opening = (fvExpandedLine !== lineIdx);
  fvExpandedLine = opening ? lineIdx : -1;
  // Roboter sofort bewegen wenn Move-Karte geöffnet wird
  if (opening) fvGoToLine(lineIdx);
  fvBuild(fvExpandedLine);
}

function fvGoToLine(lineIdx) {
  if (typeof parsedData === 'undefined' || !parsedData || !parsedData.positions) return;
  // Wegpunkt mit dieser Zeilennummer suchen
  var pos = null, posIdx = -1;
  for (var i = 0; i < parsedData.positions.length; i++) {
    if (parsedData.positions[i].lineNum === lineIdx) { pos = parsedData.positions[i]; posIdx = i; break; }
  }
  if (!pos) return;
  // IK-Tabelle verwenden wenn vorhanden
  if (typeof ikTable !== 'undefined' && ikTable[posIdx] && ikTable[posIdx].ok) {
    if (typeof applyAngles === 'function') applyAngles(ikTable[posIdx].angles);
    return;
  }
  // Fallback: IK direkt lösen
  if (typeof solveIK === 'function' && typeof jointAngles !== 'undefined') {
    var res = solveIK(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, jointAngles);
    if (res && res.ok && typeof applyAngles === 'function') applyAngles(res.angles);
  }
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

// ── Roboterposition übernehmen ───────────────────────
function fvTakeRobotPos(mode, lineIdx) {
  // jointAngles und fkAll sind global in simulator.js
  if (typeof jointAngles === 'undefined') return;

  if (mode === 'axis') {
    // A1–A6 direkt aus jointAngles
    ['A1','A2','A3','A4','A5','A6'].forEach(function(ax, idx) {
      var inp = document.getElementById('fv-'+ax+'-'+lineIdx);
      if (inp) inp.value = (jointAngles[idx] || 0).toFixed(3);
    });
  } else {
    // Kartesische Position aus FK
    var fkResult = typeof fkAll === 'function' ? fkAll(jointAngles) : null;
    if (!fkResult) return;
    var tcp = fkResult.pts[7];           // [X, Y, Z]
    var rot = fkResult.tcp_rot;          // Rotationsmatrix

    // Eulerwinkel aus Rotationsmatrix (ZYX = A, B, C)
    var b = Math.atan2(-rot[2][0], Math.sqrt(rot[0][0]*rot[0][0]+rot[1][0]*rot[1][0]));
    var cosB = Math.cos(b);
    var a, cval;
    if (Math.abs(cosB) > 1e-6) {
      a = Math.atan2(rot[1][0]/cosB, rot[0][0]/cosB);
      cval = Math.atan2(rot[2][1]/cosB, rot[2][2]/cosB);
    } else {
      a = 0;
      cval = Math.atan2(-rot[1][2], rot[1][1]);
    }
    var toDeg = 180 / Math.PI;
    var prefix = (mode === 'end') ? 'end' : 'pos';
    var vals = { X:tcp[0], Y:tcp[1], Z:tcp[2],
                 A:a*toDeg, B:b*toDeg, C:cval*toDeg };
    ['X','Y','Z','A','B','C'].forEach(function(f) {
      var inp = document.getElementById('fv-'+prefix+'-'+f+'-'+lineIdx);
      if (inp) inp.value = (vals[f]).toFixed(3);
    });
  }
  // Visuelles Feedback
  var btn = document.querySelector('#krl-form-view [data-line="'+lineIdx+'"] .fv-pos-btn');
  if (btn) { btn.style.background='rgba(0,200,100,.3)'; setTimeout(function(){ btn.style.background=''; }, 600); }
}

// Liefert die bis lineIdx deklarierten INT/REAL Variablen
function fvGetCalcVars(lineIdx) {
  var ta = document.getElementById('code-input');
  if (!ta) return [];
  var vars = [];
  var lines = ta.value.split(/\r?\n/);
  for (var j = 0; j < lineIdx && j < lines.length; j++) {
    var m = lines[j].trim().match(/^(INT|REAL)\s+(\w+)/i);
    if (m) vars.push(m[2]);
  }
  return vars;
}

function fvApplySV(lineIdx) {
  var ta    = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var sv    = fvParseSysVar(lines[lineIdx]);
  if (!sv) return;
  var newLine;
  var ioT = sv.sysvar.ioType;
  if (sv.sysvar.noValue) {
    newLine = sv.sysvar.toKRL('');
  } else if (ioT === 'dout') {
    var n = document.getElementById('fv-sv-n-'+lineIdx);
    var v = document.getElementById('fv-sv-v-'+lineIdx);
    newLine = sv.sysvar.toKRL(n?n.value:1, v?v.value:'TRUE');
  } else if (ioT === 'aout') {
    var n = document.getElementById('fv-sv-n-'+lineIdx);
    var v = document.getElementById('fv-sv-v-'+lineIdx);
    newLine = sv.sysvar.toKRL(n?n.value:1, v?v.value:0);
  } else if (ioT === 'din') {
    var n = document.getElementById('fv-sv-n-'+lineIdx);
    newLine = sv.sysvar.toKRL(n?n.value:1);
  } else if (ioT === 'ain') {
    var n = document.getElementById('fv-sv-n-'+lineIdx);
    newLine = sv.sysvar.toKRL(n?n.value:1);  // Volt-Wert ist Simulation, nicht im KRL
  } else if (ioT === 'var') {
    var type = document.getElementById('fv-sv-type-'+lineIdx);
    var name = document.getElementById('fv-sv-name-'+lineIdx);
    var val  = document.getElementById('fv-sv-val-'+lineIdx);
    // Doppelte Deklaration verhindern
    var newName = name ? name.value.trim() : 'myVar';
    var dupLines = ta.value.split(/\r?\n/);
    for (var di = 0; di < dupLines.length; di++) {
      if (di === lineIdx) continue;
      var dm = dupLines[di].trim().match(/^(INT|REAL|BOOL|CHAR)\s+(\w+)/i);
      if (dm && dm[2] === newName) {
        alert('Variable "' + newName + '" ist bereits in Zeile ' + (di+1) + ' deklariert.');
        return;
      }
    }
    newLine = sv.sysvar.toKRL(type?type.value:'INT', newName, val?val.value:'');
  } else if (ioT === 'comment') {
    var txt = document.getElementById('fv-sv-txt-'+lineIdx);
    newLine = sv.sysvar.toKRL(txt ? txt.value : '');
  } else if (ioT === 'calc') {
    var tgt  = document.getElementById('fv-sv-target-'+lineIdx);
    var expr = document.getElementById('fv-sv-expr-'+lineIdx);
    newLine = sv.sysvar.toKRL(tgt?tgt.value:'myVar', expr?expr.value.trim():'0.0');
  } else {
    var inp = document.getElementById('fv-sv-'+lineIdx);
    if (!inp) return;
    newLine = sv.sysvar.toKRL(inp.value.trim());
  }
  lines[lineIdx] = newLine;
  ta.value = lines.join('\n');
  fvExpandedLine = -1;
  // Neu parsen damit velCP und andere Systemvariablen sofort wirken
  if (typeof parseAndLoad === 'function') parseAndLoad();
  fvBuild(-1);
}

function fvApply(lineIdx) {
  var data=fvReadData(lineIdx); if (!data) return;
  var ta=document.getElementById('code-input');
  var lines=ta.value.split(/\r?\n/);
  lines[lineIdx]=fvToKRL(data);
  ta.value=lines.join('\n');
  delete fvPTPSubtypeOverride[lineIdx];
  // Neu parsen damit IK-Tabelle aktuell ist
  if (typeof parseAndLoad === 'function') parseAndLoad();
  // Roboter sofort zur neuen Position fahren
  fvGoToLine(lineIdx);
  fvExpandedLine=lineIdx; fvBuild(lineIdx);
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
        { label:'Eingang', badge:'DIN',  bc:'#8888cc', bcolor:'#fff', insert: '$IN[1]' },
        { label:'Ausgang', badge:'DOUT', bc:'#cc8800', bcolor:'#fff', insert: '$OUT[1]=TRUE' },
      ]
    },
    { id:'analog', icon: ICONS.analog, label:'Analog', color:'#cc8800',
      items: [
        { label:'Eingang', badge:'AIN',  bc:'#8888cc', bcolor:'#fff', insert: '$ANIN[1]' },
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
        { label:'CALC',  badge:'CALC', bc:'#aa7700', bcolor:'#fff', insert: 'myVar = myVar + 1.0' },
        { label:'KOM.',  badge:';',    bc:'#666666', bcolor:'#fff', insert: '; Kommentar' },
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
  var menu = document.getElementById('fvtb-menu-'+id);
  var wasOpen = btn.classList.contains('open');
  document.querySelectorAll('.fvtb-btn.open').forEach(function(b){ b.classList.remove('open'); });
  // Alle Menus zurücksetzen
  document.querySelectorAll('.fvtb-menu').forEach(function(m){ m.style.left=''; m.style.top=''; });
  if (!wasOpen && menu) {
    btn.classList.add('open');
    // position: fixed am Button — nicht durch overflow:hidden geclippt
    var r = btn.getBoundingClientRect();
    var menuH = Math.min(menu.scrollHeight, window.innerHeight * 0.8);
    var top = (r.bottom + 4 + menuH > window.innerHeight) ? Math.max(4, r.top - menuH - 4) : r.bottom + 4;
    menu.style.left = r.left + 'px';
    menu.style.top  = top + 'px';
  }
}

var fvTbTemplates = {
  move_0:    'PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000}',
  move_1:    'LIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000} C_DIS',
  move_2:    'SLIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000} C_DIS',
  move_3:    'CIRC {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000}, {X 100.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000} C_DIS',
  flow_0:    'HALT',
  flow_1:    'WAIT SEC 1.0',
  process_0: '$VEL.CP=0.167',
  process_1: '$VEL.PTP=100',
  process_2: '$ACC.CP=1.0',
  digital_0: '$IN[1]',
  digital_1: '$OUT[1]=TRUE',
  analog_0:  '$ANIN[1]',
  analog_1:  '$ANOUT[1]=0.0',
  data_0:    '$TOOL=TOOL_DATA[24]',
  data_1:    '$BASE = BASE_DATA[1]',
  data_2:    'INT myVar=0',
  data_3:    'REAL myVal=0.0',
  data_4:    'BOOL myFlag=FALSE',
  data_5:    'myVar = myVar + 1.0',
  data_6:    '; Kommentar',
  group_0:   '; #GROUP Gruppenname',
  group_1:   '; #ENDGROUP',
};

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

function fvMoveGroup(lineIdx, dir) {
  // Findet den zugehörigen ENDGROUP und verschiebt den ganzen Block
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  // Ende der Gruppe finden
  var depth = 0, endLine = -1;
  for (var j = lineIdx; j < lines.length; j++) {
    var gp = fvParseGroup(lines[j]);
    if (gp && gp.type === 'group') depth++;
    if (gp && gp.type === 'endgroup') { depth--; if (depth === 0) { endLine = j; break; } }
  }
  if (endLine < 0) { fvMoveRow(lineIdx, dir); return; }
  var block = lines.splice(lineIdx, endLine - lineIdx + 1);
  var target = lineIdx + dir;
  var ins = Math.max(0, Math.min(lines.length, dir < 0 ? 0 : target));
  for (var bi = 0; bi < block.length; bi++) lines.splice(ins + bi, 0, block[bi]);
  ta.value = lines.join('\n');
  // Collapsed-State aktualisieren
  var newStart = dir < 0 ? Math.max(0, target) : target;
  if (fvGroupCollapsed[lineIdx] !== undefined) {
    fvGroupCollapsed[newStart] = fvGroupCollapsed[lineIdx];
    delete fvGroupCollapsed[lineIdx];
  }
  fvBuild(fvExpandedLine);
}

function fvDeleteGroup(lineIdx) {
  // Löscht GROUP bis ENDGROUP inklusive
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  var depth = 0, endLine = lineIdx;
  for (var j = lineIdx; j < lines.length; j++) {
    var gp = fvParseGroup(lines[j]);
    if (gp && gp.type === 'group') depth++;
    if (gp && gp.type === 'endgroup') { depth--; if (depth === 0) { endLine = j; break; } }
  }
  lines.splice(lineIdx, endLine - lineIdx + 1);
  ta.value = lines.join('\n');
  delete fvGroupCollapsed[lineIdx];
  fvExpandedLine = -1;
  fvBuild(-1);
}

function fvGroupToggle(lineIdx) {
  fvGroupCollapsed[lineIdx] = !fvGroupCollapsed[lineIdx];
  fvBuild(fvExpandedLine);
}

function fvGroupRename(lineIdx, el) {
  fvGroupCollapsed[lineIdx] = false;
  var current = el.textContent.trim();

  // Direkt im DOM Input einsetzen ohne fvBuild
  var nameSpan = document.querySelector('#krl-form-view [data-line="'+lineIdx+'"] .fv-group-name');
  if (!nameSpan) return;

  var inp = document.createElement('input');
  inp.className = 'fv-group-inp';
  inp.value = current;
  nameSpan.replaceWith(inp);
  inp.focus(); inp.select();

  var saved = false;
  function save() {
    if (saved) return; saved = true;
    var newName = inp.value.trim() || current;
    var ta = document.getElementById('code-input');
    var lines = ta.value.split(/\r?\n/);
    lines[lineIdx] = '; #GROUP ' + newName;
    ta.value = lines.join('\n');
    fvBuild(fvExpandedLine);
  }
  inp.addEventListener('blur', save);
  inp.addEventListener('keydown', function(e) {
    if (e.key === 'Enter') { e.preventDefault(); save(); }
    if (e.key === 'Escape') { saved = true; fvBuild(fvExpandedLine); }
  });
}

function fvSetCursor(lineIdx) {
  fvToolbarInsertAfter = lineIdx;
  // Visuell markieren welche Zeile als Einfügeposition gilt
  document.querySelectorAll('#krl-form-view [data-line]').forEach(function(el){
    el.classList.remove('fv-cursor-line');
  });
  var el = document.querySelector('#krl-form-view [data-line="'+lineIdx+'"]');
  if (el) el.classList.add('fv-cursor-line');
}
