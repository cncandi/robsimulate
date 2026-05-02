// ══════════════════════════════════════════════════════
// FORMAT: KUKA Formular
// ── Inline-Editor für LIN / PTP / SLIN / CIRC ────────
// ══════════════════════════════════════════════════════

// ── State ────────────────────────────────────────────
var fvExpandedLine     = -1;
var fvPTPSubtypeOverride = {}; // lineIdx -> 'axis'|'cart'

// ── Registrierung ────────────────────────────────────
FormatRegistry.register({
  id:    'kuka-form',
  label: 'KUKA Formular',

  activate: function () {
    document.getElementById('code-input').style.display = 'none';
    document.getElementById('gutter').style.display     = 'none';
    var fv = document.getElementById('krl-form-view');
    if (fv) { fv.style.display = 'flex'; }
    fvBuild(fvExpandedLine);
  },

  deactivate: function () {
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'none';
    document.getElementById('code-input').style.display = '';
    document.getElementById('gutter').style.display     = '';
  }
});

// ── Zeilen-Parser ─────────────────────────────────────
function fvParseLine(raw) {
  var t = raw.trim();

  // CIRC {aux}, {end} [versch]
  var mC = t.match(/^CIRC\s+\{([^}]+)\}\s*,\s*\{([^}]+)\}\s*(C_DIS|C_PTP|C_VEL|C_ORI)?\s*$/i);
  if (mC) return { moveType:'CIRC', subtype:'circ',
                   aux: parsePos(mC[1]), end: parsePos(mC[2]), verl: mC[3]||'' };

  // LIN / SLIN
  var mL = t.match(/^(LIN|SLIN)\s+\{([^}]+)\}\s*(C_DIS|C_PTP|C_VEL|C_ORI)?\s*$/i);
  if (mL) return { moveType: mL[1].toUpperCase(), subtype:'cart',
                   pos: parsePos(mL[2]), verl: mL[3]||'' };

  // PTP
  var mP = t.match(/^PTP\s+\{([^}]+)\}\s*(C_PTP)?\s*$/i);
  if (mP) {
    var inner = mP[1];
    if (/A[123456]\s/.test(inner)) {
      var ap = {A1:0,A2:0,A3:0,A4:0,A5:0,A6:0}, am;
      var rx = /A([123456])\s+([-+]?\d+(?:\.\d+)?)/g;
      while ((am = rx.exec(inner)) !== null) ap['A'+am[1]] = parseFloat(am[2]);
      return { moveType:'PTP', subtype:'axis', pos:ap, verl:mP[2]||'' };
    }
    return { moveType:'PTP', subtype:'cart', pos:parsePos(inner), verl:mP[2]||'' };
  }
  return null;
}

// ── KRL serialisieren ─────────────────────────────────
function fvToKRL(data) {
  var v = data.verl ? ' '+data.verl : '';
  if (data.moveType === 'CIRC')
    return 'CIRC '+fvFmtPos(data.aux)+', '+fvFmtPos(data.end)+v;
  if (data.subtype === 'axis') {
    var p = data.pos;
    return 'PTP {A1 '+fvN(p.A1)+', A2 '+fvN(p.A2)+', A3 '+fvN(p.A3)+
           ', A4 '+fvN(p.A4)+', A5 '+fvN(p.A5)+', A6 '+fvN(p.A6)+'}'+v;
  }
  return data.moveType+' '+fvFmtPos(data.pos)+v;
}
function fvFmtPos(p) {
  return '{X '+fvN(p.X)+', Y '+fvN(p.Y)+', Z '+fvN(p.Z)+
         ', A '+fvN(p.A)+', B '+fvN(p.B)+', C '+fvN(p.C)+'}';
}
function fvN(n) { return parseFloat(n||0).toFixed(3); }

// ── View aufbauen ─────────────────────────────────────
function fvBuild(expandLine) {
  var fv = document.getElementById('krl-form-view');
  if (!fv || fv.style.display === 'none') return;
  var lines = document.getElementById('code-input').value.split(/\r?\n/);
  var html  = '';

  for (var i = 0; i < lines.length; i++) {
    var raw = lines[i];
    var mv  = fvParseLine(raw);

    if (mv) {
      if (mv.moveType === 'PTP' && fvPTPSubtypeOverride[i] !== undefined)
        mv.subtype = fvPTPSubtypeOverride[i];

      var isExp   = (expandLine === i);
      var summary = raw.trim().substring(0,44)+(raw.trim().length>44?'…':'');

      html += '<div class="fv-row fv-move" data-line="'+i+'">';
      html += '<div class="fv-head" onclick="fvToggle('+i+')">';
      html += '<span class="fv-badge fv-badge-'+mv.moveType.toLowerCase()+'">'+mv.moveType+'</span>';
      html += '<span class="fv-summary">'+fvEsc(summary)+'</span>';
      html += '<span class="fv-chevron">'+(isExp?'▲':'▼')+'</span>';
      html += '</div>';

      if (isExp) {
        html += '<div class="fv-form">';
        html += fvFormHTML(mv, i);
        html += '<div class="fv-acts">'
               +'<button class="fv-btn fv-apply" onclick="fvApply('+i+')">✓ Übernehmen</button>'
               +'<button class="fv-btn fv-del"   onclick="fvDelete('+i+')">✕ Löschen</button>'
               +'</div>';
        html += '</div>';
      }
      html += '</div>';

    } else {
      var cls = raw.trim().startsWith(';') ? 'fv-comment' :
                raw.trim() === '' ? 'fv-empty' : 'fv-plain';
      html += '<div class="fv-row '+cls+'" data-line="'+i+'">'
            + (cls !== 'fv-empty' ? '<span class="fv-plaintext">'+fvEsc(raw.trim())+'</span>' : '')
            + '</div>';
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

function fvFormHTML(mv, i) {
  var html = '';
  var verlOpts = mv.moveType === 'PTP' ? ['','C_PTP'] : ['','C_DIS','C_VEL','C_ORI'];

  // Typ + Verschleifung
  html += '<div class="fv-row2">';
  html += '<label>Typ<select class="fv-sel" id="fv-type-'+i+'" onchange="fvTypeChange('+i+')">';
  ['LIN','PTP','SLIN','CIRC'].forEach(function(tp){
    html += '<option value="'+tp+'"'+(mv.moveType===tp?' selected':'')+'>'+tp+'</option>';
  });
  html += '</select></label>';
  html += '<label>Versch.<select class="fv-sel" id="fv-verl-'+i+'">';
  verlOpts.forEach(function(vo){
    html += '<option value="'+vo+'"'+(mv.verl===vo?' selected':'')+'>'+(vo||'—')+'</option>';
  });
  html += '</select></label>';
  html += '</div>';

  // PTP Subtype-Toggle
  if (mv.moveType === 'PTP') {
    html += '<div style="display:flex;gap:4px;margin-bottom:5px">'
          + '<button class="fv-btn'+(mv.subtype==='cart'?' fv-sub-on':'')+'" onclick="fvPTPToggle('+i+',\'cart\')">X/Y/Z</button>'
          + '<button class="fv-btn'+(mv.subtype==='axis'?' fv-sub-on':'')+'" onclick="fvPTPToggle('+i+',\'axis\')">A1–A6</button>'
          + '</div>';
  }

  if (mv.subtype === 'axis') {
    html += '<div class="fv-grid2">';
    ['A1','A2','A3','A4','A5','A6'].forEach(function(ax){
      html += '<label>'+ax+'<input class="fv-inp" id="fv-'+ax+'-'+i+'" type="number" step="0.001" value="'+fvN(mv.pos[ax])+'"><span>°</span></label>';
    });
    html += '</div>';
  } else if (mv.subtype === 'circ') {
    html += '<div class="fv-circ-lbl">Hilfspunkt</div><div class="fv-grid3">'+fvCartInputs(mv.aux,'aux',i)+'</div>';
    html += '<div class="fv-circ-lbl">Endpunkt</div><div class="fv-grid3">'+fvCartInputs(mv.end,'end',i)+'</div>';
  } else {
    html += '<div class="fv-grid3">'+fvCartInputs(mv.pos,'pos',i)+'</div>';
  }
  return html;
}

function fvCartInputs(p, prefix, i) {
  var html = '';
  [['X','mm'],['A','°'],['Y','mm'],['B','°'],['Z','mm'],['C','°']].forEach(function(f){
    html += '<label>'+f[0]+'<input class="fv-inp" id="fv-'+prefix+'-'+f[0]+'-'+i+
            '" type="number" step="0.1" value="'+fvN(p[f[0]])+'"><span>'+f[1]+'</span></label>';
  });
  return html;
}

// ── Event-Handler ─────────────────────────────────────
function fvToggle(lineIdx) {
  fvExpandedLine = (fvExpandedLine === lineIdx) ? -1 : lineIdx;
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
  var opts = newType === 'PTP' ? ['','C_PTP'] : ['','C_DIS','C_VEL','C_ORI'];
  if (opts.indexOf(curVerl) < 0) curVerl = '';
  if (verlSel) {
    verlSel.innerHTML = '';
    opts.forEach(function(vo){
      var opt = document.createElement('option');
      opt.value = vo; opt.textContent = vo||'—';
      if (vo===curVerl) opt.selected = true;
      verlSel.appendChild(opt);
    });
  }
  if (newType !== 'PTP') delete fvPTPSubtypeOverride[lineIdx];
  // Formular-Bereich neu rendern
  var ta  = document.getElementById('code-input');
  var mv  = fvParseLine(ta.value.split(/\r?\n/)[lineIdx]||'');
  if (!mv) return;
  mv.moveType = newType;
  if (newType === 'CIRC') {
    mv.subtype = 'circ';
    mv.aux = mv.aux || {X:0,Y:0,Z:0,A:0,B:0,C:0};
    mv.end = mv.end || mv.pos || {X:0,Y:0,Z:0,A:0,B:0,C:0};
  } else if (newType !== 'PTP') {
    mv.subtype = 'cart';
  }
  if (fvPTPSubtypeOverride[lineIdx]) mv.subtype = fvPTPSubtypeOverride[lineIdx];
  var formDiv = document.querySelector('#krl-form-view [data-line="'+lineIdx+'"] .fv-form');
  if (!formDiv) return;
  formDiv.innerHTML = fvFormHTML(mv, lineIdx)
    + '<div class="fv-acts">'
    + '<button class="fv-btn fv-apply" onclick="fvApply('+lineIdx+')">✓ Übernehmen</button>'
    + '<button class="fv-btn fv-del"   onclick="fvDelete('+lineIdx+')">✕ Löschen</button>'
    + '</div>';
}

function fvReadData(lineIdx) {
  var ta   = document.getElementById('code-input');
  var mv   = fvParseLine(ta.value.split(/\r?\n/)[lineIdx]||'');
  if (!mv) return null;
  var typeSel = document.getElementById('fv-type-'+lineIdx);
  var verlSel = document.getElementById('fv-verl-'+lineIdx);
  mv.moveType = typeSel ? typeSel.value : mv.moveType;
  mv.verl     = verlSel ? verlSel.value : mv.verl;
  if (mv.moveType === 'PTP' && fvPTPSubtypeOverride[lineIdx])
    mv.subtype = fvPTPSubtypeOverride[lineIdx];
  if (mv.moveType === 'CIRC') mv.subtype = 'circ';

  function readCart(prefix) {
    var p = {};
    ['X','Y','Z','A','B','C'].forEach(function(f){
      var inp = document.getElementById('fv-'+prefix+'-'+f+'-'+lineIdx);
      p[f] = inp ? parseFloat(inp.value)||0 : 0;
    });
    return p;
  }
  if (mv.subtype === 'axis') {
    var ap = {};
    ['A1','A2','A3','A4','A5','A6'].forEach(function(ax){
      var inp = document.getElementById('fv-'+ax+'-'+lineIdx);
      ap[ax] = inp ? parseFloat(inp.value)||0 : 0;
    });
    mv.pos = ap;
  } else if (mv.subtype === 'circ') {
    mv.aux = readCart('aux');
    mv.end = readCart('end');
  } else {
    mv.pos = readCart('pos');
  }
  return mv;
}

function fvApply(lineIdx) {
  var data = fvReadData(lineIdx);
  if (!data) return;
  var ta    = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  lines[lineIdx] = fvToKRL(data);
  ta.value = lines.join('\n');
  delete fvPTPSubtypeOverride[lineIdx];
  fvExpandedLine = -1;
  fvBuild(-1);
}

function fvDelete(lineIdx) {
  var ta    = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  lines.splice(lineIdx, 1);
  ta.value = lines.join('\n');
  delete fvPTPSubtypeOverride[lineIdx];
  fvExpandedLine = -1;
  fvBuild(-1);
}

function fvInsertNew(afterLineIdx) {
  var ta    = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/);
  lines.splice(afterLineIdx+1, 0, 'LIN {X 0.000, Y 0.000, Z 0.000, A 0.000, B 0.000, C 0.000}');
  ta.value = lines.join('\n');
  fvExpandedLine = afterLineIdx+1;
  fvBuild(fvExpandedLine);
}

function fvEsc(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}
