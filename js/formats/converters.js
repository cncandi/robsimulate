// ══════════════════════════════════════════════════════════════════════════
// ROBOT FORMAT CONVERTERS  —  Formular ↔ Hersteller-Code
// Quelle der Befehlsmappings: CAD/CAM Reitz – Roboterbefehle im Vergleich
// ══════════════════════════════════════════════════════════════════════════
(function () {
  'use strict';

  // ── Logo helper ─────────────────────────────────────────────────────────
  function logo(file, h) {
    h = h || 14;
    return '<img src="logos/' + file + '" height="' + h + '" '
      + 'style="vertical-align:middle;margin-right:6px;max-width:56px;object-fit:contain">';
  }

  // ── Velocity helpers ────────────────────────────────────────────────────
  function velMmS(v) { return Math.max(1, Math.round((v || 0.167) * 1000)); }
  function velPct(v) { return Math.max(1, Math.min(100, Math.round((v || 0.167) / 2.0 * 100))); }

  // ── Format position ─────────────────────────────────────────────────────
  function fmtP(pos, d) {
    d = d == null ? 3 : d;
    return {
      X: pos.X.toFixed(d), Y: pos.Y.toFixed(d), Z: pos.Z.toFixed(d),
      A: pos.A.toFixed(d), B: pos.B.toFixed(d), C: pos.C.toFixed(d)
    };
  }

  // ── Core generator ──────────────────────────────────────────────────────
  // Iteriert direkt über parsedData.positions.
  // Jede Position trägt: type ('LIN'|'PTP'|'SLIN'|'CIRC'|'CIRC_AUX'), velCP, X..C
  // Tool/Base/Vars werden aus dem KRL-Code (code-input) extrahiert.
  // Hauptgenerator für DIRECT_STREAM Formate
  // Iteriert parsedData.steps (inkl. I/O, Wait etc.) und nutzt applyTpl
  function generate(parsedData, cfg) {
    if (!parsedData) return '';
    var positions = parsedData.positions || [];
    var steps     = parsedData.steps     || [];
    var fmtId     = cfg._formatId || '';
    var lines = [];
    var push  = function(l) { if (l != null && l !== '') lines.push(l); };

    // Kopf: Tool/Base/Vars aus steps sammeln
    var toolN = 1, baseN = 1, vars = [];
    steps.forEach(function(s) {
      if (s.type === 'tool') toolN = s.n;
      if (s.type === 'base') baseN = s.n;
      if (s.type === 'var')  vars.push(s);
    });

    // Programmkopf
    var hf = (fmtId && typeof fmtHfLoad === 'function') ? fmtHfLoad(fmtId) : null;
    if (hf && hf.header) hf.header.split('\n').forEach(push);
    else if (cfg.header) cfg.header(toolN, baseN, vars).forEach(push);

    // Template-Helfer
    var curVel = 0.167, curToolN = toolN, curBaseN = baseN;
    function t(key, vars2, fallback) {
      if (fmtId && typeof applyTpl === 'function') {
        var r = applyTpl(fmtId, key, vars2);
        if (r !== null) return r;
      }
      return fallback ? fallback() : null;
    }

    // Wenn steps vorhanden: steps-basiert (vollständig mit I/O)
    if (steps.length) {
      steps.forEach(function(s) {
        switch(s.type) {
          case 'comment': if (cfg.comment) push(cfg.comment(s.text||'')); break;
          case 'velcp':   curVel = s.v || 0.167; if (cfg.velLine) push(cfg.velLine(curVel)); break;
          case 'tool':
            curToolN = s.n;
            push(t('tool',{N:s.n,TOOL:s.n},cfg.tool?function(){return cfg.tool(s.n);}:null)); break;
          case 'base':
            curBaseN = s.n;
            push(t('base',{N:s.n,BASE:s.n},cfg.base?function(){return cfg.base(s.n);}:null)); break;
          case 'var': {
            var vt=s.varType||'REAL', vi=s.val!=null?s.val:(vt==='BOOL'?'FALSE':vt==='INT'?'0':'0.0');
            var vk=vt==='INT'?'varInt':vt==='BOOL'?'varBool':'varReal';
            push(t(vk,{TYPE:vt,NAME:s.name||'v',INITVAL:vi},
              cfg.varDecl?function(){return cfg.varDecl(vt,s.name,s.val);}:null)); break;
          }
          case 'move': {
            var pos=positions[s.posIdx]; if(!pos) break;
            var isLin=s.moveType==='LIN'||s.moveType==='SLIN';
            var p=fmtP(pos), idx=(s.posIdx||0)+1;
            var mv={N:idx,X:pos.X.toFixed(3),Y:pos.Y.toFixed(3),Z:pos.Z.toFixed(3),
              A:pos.A.toFixed(3),B:pos.B.toFixed(3),C:pos.C.toFixed(3),
              VEL_MMS:velMmS(curVel),VEL_PCT:velPct(curVel),
              VEL_MS:(curVel||0.167).toFixed(4),TOOL:curToolN,BASE:curBaseN};
            push(t(isLin?'moveL':'moveJ',mv,isLin
              ?(cfg.moveL?function(){return cfg.moveL(p,curVel,idx,pos);}:null)
              :(cfg.moveJ?function(){return cfg.moveJ(p,curVel,idx,pos);}:null))); break;
          }
          case 'circ': {
            var pv=positions[s.viaIdx],pt=positions[s.posIdx]; if(!pv||!pt) break;
            var cv={N:(s.posIdx||0)+1,VN:(s.viaIdx||0)+1,
              X:pt.X.toFixed(3),Y:pt.Y.toFixed(3),Z:pt.Z.toFixed(3),
              A:pt.A.toFixed(3),B:pt.B.toFixed(3),C:pt.C.toFixed(3),
              VX:pv.X.toFixed(3),VY:pv.Y.toFixed(3),VZ:pv.Z.toFixed(3),
              VA:pv.A.toFixed(3),VB:pv.B.toFixed(3),VC:pv.C.toFixed(3),
              VEL_MMS:velMmS(curVel),VEL_PCT:velPct(curVel),TOOL:curToolN,BASE:curBaseN};
            push(t('moveC',cv,cfg.moveC?function(){return cfg.moveC(fmtP(pv),fmtP(pt),curVel,(s.posIdx||0)+1);}:null)); break;
          }
          case 'halt':   push(t('halt',{},cfg.halt?function(){return cfg.halt();}:null)); break;
          case 'brake':  push(t('halt',{},cfg.brake?function(){return cfg.brake();}:cfg.halt?function(){return cfg.halt();}:null)); break;
          case 'dout': {
            var dv=s.v==='TRUE'||s.v==='1'||s.v==='ON';
            push(t('dout',{CH:s.n,VAL:dv?'TRUE':'FALSE'},cfg.dout?function(){return cfg.dout(s.n,dv);}:null)); break;
          }
          case 'din':   push(t('din',{CH:s.n},cfg.dinWait?function(){return cfg.dinWait(s.n);}:null)); break;
          case 'aout': {
            var af=parseFloat(s.v||0).toFixed(2);
            push(t('aout',{CH:s.n,VAL_F:af},cfg.aout?function(){return cfg.aout(s.n,s.v);}:null)); break;
          }
          case 'ain':   push(t('ain',{CH:s.n},cfg.ainRead?function(){return cfg.ainRead(s.n);}:null)); break;
          case 'wait': {
            var wt=parseFloat(s.t||0).toFixed(1);
            push(t('wait',{T:wt,T_MS:Math.round(parseFloat(s.t||0)*1000)},
              cfg.waitSec?function(){return cfg.waitSec(s.t);}:null)); break;
          }
          case 'waitFor': push(t('din',{CH:1},cfg.waitFor?function(){return cfg.waitFor(s.cond||'');}:cfg.dinWait?function(){return cfg.dinWait(1);}:null)); break;
        }
      });
    } else {
      // Fallback: nur Positionen (ältere parsedData ohne steps)
      positions.forEach(function(pos,i) {
        var vel=curVel, idx=i+1, p=fmtP(pos);
        var mv={N:idx,X:pos.X.toFixed(3),Y:pos.Y.toFixed(3),Z:pos.Z.toFixed(3),
          A:pos.A.toFixed(3),B:pos.B.toFixed(3),C:pos.C.toFixed(3),
          VEL_MMS:velMmS(vel),VEL_PCT:velPct(vel),VEL_MS:(vel).toFixed(4),TOOL:curToolN,BASE:curBaseN};
        var line = t('moveL',mv,cfg.moveL?function(){return cfg.moveL(p,vel,idx,pos);}:null);
        if (line) push(line);
      });
    }

    // Optionale Punkt-Sektion (FANUC-Style)
    if (cfg.ptpSection && positions.length) {
      var ptpLines = cfg.ptpSection(positions);
      if (ptpLines && ptpLines.length) ptpLines.forEach(push);
    }

    // Programmfuß
    if (hf && hf.footer) hf.footer.split('\n').forEach(push);
    else if (cfg.footer) cfg.footer().forEach(push);
    return lines.join('\n');
  }

  // ── Minimal parser: extracts positions ─────────────────────────────────
  function parsePositions(text, patterns) {
    var positions = [], steps = [];
    var lines = text.split(/\r?\n/);
    lines.forEach(function (line, lineIdx) {
      var t = line.trim();
      patterns.forEach(function (pat) {
        var m = t.match(pat.rx);
        if (!m) return;
        var pos = { X: 0, Y: 0, Z: 0, A: 0, B: 0, C: 0, S: null, T: null, lineNum: lineIdx };
        if (m.groups) {
          ['X','Y','Z','A','B','C'].forEach(function (k) {
            if (m.groups[k] !== undefined) pos[k] = parseFloat(m.groups[k]);
          });
        }
        var idx = positions.length;
        positions.push(pos);
        steps.push({ type: 'move', moveType: pat.ptp ? 'PTP' : 'LIN', posIdx: idx, lineNum: lineIdx });
      });
    });
    return { positions: positions, steps: steps, finalState: { variables: {}, digitalIn: {}, digitalOut: {}, analogOut: {} } };
  }

  // ── Shared activate — impl direkt via Closure ──────────────────────────
  function makeActivate(impl) {
    return function () {
      var ci = document.getElementById('code-input');
      var gt = document.getElementById('gutter');
      var fv = document.getElementById('krl-form-view');
      var fw = document.getElementById('fv-form-wrap');
      if (ci) ci.style.display = '';
      if (gt) gt.style.display = '';
      if (fv) fv.style.display = 'none';
      if (fw) fw.style.display = 'none';
      // Formular ist immer Quelle — parsedData direkt verwenden
      var pd = (typeof parsedData !== 'undefined') ? parsedData : { positions: [], steps: [], finalState: {} };
      if (impl && impl._generate) {
        ci.value = impl._generate(pd);
        if (typeof rebuildGutter === 'function') rebuildGutter();
      }
    };
  }

  // ══════════════════════════════════════════════════════════════════════════
  // FORMAT IMPLEMENTATIONS
  // ══════════════════════════════════════════════════════════════════════════

  var IMPLS = {};

  // ── ABB RAPID / IRC5 / OmniCore ─────────────────────────────────────────
  // ABB RAPID — MIXED_MODULE
  // Struktur: MODULE → VAR-Deklarationen → CONST robtargets → PROC main() → Bewegungen → ENDPROC → ENDMODULE
  IMPLS.abb = {
    _generate: function (pd) {
      if (!pd) return '';
      var positions = pd.positions || [];
      var steps = pd.steps || [];
      var lines = [];
      var hf = (typeof fmtHfLoad === 'function') ? fmtHfLoad('abb') : null;
      var toolN = 1, baseN = 1, vars = [];
      steps.forEach(function(s) {
        if (s.type === 'tool') toolN = s.n;
        if (s.type === 'base') baseN = s.n;
        if (s.type === 'var') vars.push(s);
      });

      // Programmkopf (Settings oder Default)
      var hdr = (hf && hf.header) ? hf.header : 'MODULE PP_MAIN';
      hdr.split('\n').forEach(function(l) { lines.push(l); });

      // VAR-Deklarationen
      vars.forEach(function(v) {
        var typ = v.varType === 'BOOL' ? 'bool' : 'num';
        var val = v.val || (v.varType === 'BOOL' ? 'FALSE' : v.varType === 'REAL' ? '0.0' : '0');
        lines.push('  VAR ' + typ + ' ' + v.name + ' := ' + val + ';');
      });

      // CONST robtarget — alle Positionen vorab deklarieren (MIXED_MODULE)
      positions.forEach(function(pos, i) {
        var name = 'p' + (i + 1);
        lines.push('  CONST robtarget ' + name + ' := [[' +
          parseFloat(pos.X).toFixed(3) + ',' + parseFloat(pos.Y).toFixed(3) + ',' + parseFloat(pos.Z).toFixed(3) +
          '],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];');
      });

      lines.push('  PROC main()');
      if (toolN) lines.push('    ! Tool: tool' + toolN);
      if (baseN) lines.push('    ! WObj: wobj' + baseN);

      // Bewegungs-Block
      var curVel = 0.167;
      steps.forEach(function(s) {
        switch (s.type) {
          case 'comment':  lines.push('    ! ' + (s.text || '')); break;
          case 'velcp':    curVel = s.v || 0.167; break;
          case 'halt':     lines.push('    Stop;'); break;
          case 'dout':     lines.push('    SetDO do' + s.n + ', ' + ((s.v === 'TRUE' || s.v === '1') ? '1' : '0') + ';'); break;
          case 'din':      lines.push('    WaitDI di' + s.n + ', 1;'); break;
          case 'aout':     lines.push('    SetAO ao' + s.n + ', ' + parseFloat(s.v || 0).toFixed(2) + ';'); break;
          case 'ain':      lines.push('    r := AInput(ai' + s.n + ');'); break;
          case 'wait':     lines.push('    WaitTime ' + parseFloat(s.t || 0).toFixed(1) + ';'); break;
          case 'waitFor':  lines.push('    WaitUntil ' + (s.cond || 'di1 = 1') + ';'); break;
          case 'move': {
            var pos = positions[s.posIdx]; if (!pos) break;
            var nm = 'p' + (s.posIdx + 1);
            var v = velMmS(curVel);
            if (s.moveType === 'LIN' || s.moveType === 'SLIN')
              lines.push('    MoveL ' + nm + ', v' + v + ', fine, tool' + toolN + '\\WObj:=wobj' + baseN + ';');
            else
              lines.push('    MoveJ ' + nm + ', v' + v + ', z10, tool' + toolN + '\\WObj:=wobj' + baseN + ';');
            break;
          }
          case 'circ': {
            var pv = positions[s.viaIdx], pt = positions[s.posIdx]; if (!pv || !pt) break;
            lines.push('    MoveC p' + (s.viaIdx+1) + ', p' + (s.posIdx+1) + ', v' + velMmS(curVel) + ', fine, tool' + toolN + '\\WObj:=wobj' + baseN + ';');
            break;
          }
        }
      });

      lines.push('  ENDPROC');
      var ftr = (hf && hf.footer) ? hf.footer : 'ENDMODULE';
      ftr.split('\n').forEach(function(l) { lines.push(l); });
      return lines.join('\n');
    }
  };

  // FANUC TP — POINT_SECTION_THEN_INSTRUCTIONS
  // Struktur: /PROG → /MN (Instruktionen mit P[n]-Referenzen) → /POS (Koordinaten) → /END
  IMPLS.fanuc = {
    _generate: function (pd) {
      if (!pd) return '';
      var positions = pd.positions || [];
      var steps = pd.steps || [];
      var lines = [];
      var hf = (typeof fmtHfLoad === 'function') ? fmtHfLoad('fanuc') : null;
      var toolN = 1, baseN = 1, ln = 1;
      steps.forEach(function(s) {
        if (s.type === 'tool') toolN = s.n;
        if (s.type === 'base') baseN = s.n;
      });

      // Programmkopf
      var hdr = (hf && hf.header) ? hf.header : '/PROG PP_MAIN\n/ATTR\nOWNER = MNEDITOR;\n/MN';
      hdr.split('\n').forEach(function(l) { lines.push(l); });
      lines.push(' ' + (ln++) + ': UTOOL_NUM=' + toolN + ' ;');
      lines.push(' ' + (ln++) + ': UFRAME_NUM=' + baseN + ' ;');

      // Instruktionen (referenzieren P[n])
      var posCounter = {};  // posIdx → P[n] Nummer
      var pNum = 1;
      var curVel = 0.167;
      steps.forEach(function(s) {
        switch (s.type) {
          case 'velcp':    curVel = s.v || 0.167; break;
          case 'comment':  lines.push(' ' + (ln++) + ': ! ' + (s.text||'') + ' ;'); break;
          case 'halt':     lines.push(' ' + (ln++) + ': PAUSE ;'); break;
          case 'dout':     lines.push(' ' + (ln++) + ': DO[' + s.n + ']=' + ((s.v==='TRUE'||s.v==='1')?'ON':'OFF') + ' ;'); break;
          case 'din':      lines.push(' ' + (ln++) + ': WAIT DI[' + s.n + ']=ON ;'); break;
          case 'aout':     lines.push(' ' + (ln++) + ': AO[' + s.n + ']=' + parseFloat(s.v||0).toFixed(2) + ' ;'); break;
          case 'wait':     lines.push(' ' + (ln++) + ': WAIT ' + parseFloat(s.t||0).toFixed(2) + '(sec) ;'); break;
          case 'move': {
            if (posCounter[s.posIdx] === undefined) posCounter[s.posIdx] = pNum++;
            var pn = posCounter[s.posIdx];
            if (s.moveType === 'LIN' || s.moveType === 'SLIN')
              lines.push(' ' + (ln++) + ':L P[' + pn + '] ' + velMmS(curVel) + 'mm/sec FINE ;');
            else
              lines.push(' ' + (ln++) + ':J P[' + pn + '] ' + velPct(curVel) + '% FINE ;');
            break;
          }
          case 'circ': {
            if (posCounter[s.viaIdx] === undefined) posCounter[s.viaIdx] = pNum++;
            if (posCounter[s.posIdx] === undefined) posCounter[s.posIdx] = pNum++;
            lines.push(' ' + (ln++) + ':C P[' + posCounter[s.viaIdx] + ']');
            lines.push('   P[' + posCounter[s.posIdx] + '] ' + velMmS(curVel) + 'mm/sec FINE ;');
            break;
          }
        }
      });

      // /POS Sektion — Koordinaten der referenzierten Punkte
      lines.push('/POS');
      Object.keys(posCounter).forEach(function(posIdx) {
        var pos = positions[parseInt(posIdx)]; if (!pos) return;
        var n = posCounter[posIdx];
        lines.push('P[' + n + ']{');
        lines.push('   GP1:');
        lines.push("    UF : " + baseN + ", UT : " + toolN + ",        CONFIG : 'N U T, 0, 0, 0',");
        lines.push('    X =' + parseFloat(pos.X).toFixed(3) + '  mm, Y =' + parseFloat(pos.Y).toFixed(3) + '  mm, Z =' + parseFloat(pos.Z).toFixed(3) + '  mm,');
        lines.push('    W =' + parseFloat(pos.A).toFixed(3) + '  deg, P =' + parseFloat(pos.B).toFixed(3) + '  deg, R =' + parseFloat(pos.C).toFixed(3) + '  deg');
        lines.push('};');
      });

      // Programmfuß
      var ftr = (hf && hf.footer) ? hf.footer : '/END';
      ftr.split('\n').forEach(function(l) { lines.push(l); });
      return lines.join('\n');
    }
  };

  // Yaskawa INFORM JBI — POINT_TABLE_THEN_INST
  // Struktur: /JOB Header → //POS (C00000 mit Koordinaten) → //INST (NOP + MOVJ/MOVL/MOVC + END)
  IMPLS.yaskawa = {
    _generate: function (pd) {
      if (!pd) return '';
      var positions = pd.positions || [];
      var steps = pd.steps || [];
      var lines = [];
      var hf = (typeof fmtHfLoad === 'function') ? fmtHfLoad('yaskawa') : null;
      var toolN = 1, baseN = 1, vars = [];
      steps.forEach(function(s) {
        if (s.type === 'tool') toolN = s.n;
        if (s.type === 'base') baseN = s.n;
        if (s.type === 'var') vars.push(s);
      });

      // Phase 1: alle Positionen aus steps sammeln (in Reihenfolge, mit Index)
      var usedPositions = []; // [{posIdx, cName}]
      var posMap = {};        // posIdx → C-Name
      var cCounter = 0;
      steps.forEach(function(s) {
        if (s.type === 'move' && posMap[s.posIdx] === undefined) {
          var cName = 'C' + String(cCounter).padStart(5, '0');
          posMap[s.posIdx] = cName;
          usedPositions.push({ posIdx: s.posIdx, cName: cName });
          cCounter++;
        }
        if (s.type === 'circ') {
          [s.viaIdx, s.posIdx].forEach(function(idx) {
            if (posMap[idx] === undefined) {
              var cName = 'C' + String(cCounter).padStart(5, '0');
              posMap[idx] = cName;
              usedPositions.push({ posIdx: idx, cName: cName });
              cCounter++;
            }
          });
        }
      });

      // /JOB Header
      var hdr = (hf && hf.header) ? hf.header
        : '/JOB\n//NAME PP_MAIN';
      hdr.split('\n').forEach(function(l) { lines.push(l); });

      // //POS Sektion mit echten Koordinaten
      lines.push('//POS');
      lines.push('///NPOS ' + usedPositions.length + ',0,0,0,0,0');
      lines.push('///TOOL ' + toolN);
      lines.push('///POSTYPE ROBOT');
      lines.push('///RECTAN');
      lines.push('///UNIT MM,RAD,SEC,CM/MIN,DEG/SEC,PERCENT');
      usedPositions.forEach(function(entry) {
        var pos = positions[entry.posIdx];
        if (!pos) return;
        lines.push(entry.cName + '=' +
          parseFloat(pos.X).toFixed(3) + ',' +
          parseFloat(pos.Y).toFixed(3) + ',' +
          parseFloat(pos.Z).toFixed(3) + ',' +
          parseFloat(pos.A * Math.PI / 180).toFixed(6) + ',' +
          parseFloat(pos.B * Math.PI / 180).toFixed(6) + ',' +
          parseFloat(pos.C * Math.PI / 180).toFixed(6));
      });

      // //INST Sektion
      lines.push('//INST');
      lines.push('///ATTR SC,RW');
      lines.push('NOP');

      // Variablen
      vars.forEach(function(v) {
        var pfx = v.varType === 'INT' ? 'I' : v.varType === 'REAL' ? 'R' : 'B';
        lines.push('SET ' + pfx + '000 ' + (v.val || '0'));
      });

      // Befehle
      var curVel = 0.167;
      steps.forEach(function(s) {
        switch (s.type) {
          case 'velcp':    curVel = s.v || 0.167; break;
          case 'comment':  lines.push("'" + (s.text || '')); break;
          case 'halt':     lines.push('PAUSE'); break;
          case 'dout':     lines.push('DOUT OT#(' + s.n + ') ' + ((s.v==='TRUE'||s.v==='1')?'ON':'OFF')); break;
          case 'din':      lines.push('WAIT IN#(' + s.n + ')=ON'); break;
          case 'aout':     lines.push('AOUT AO#(' + s.n + ') ' + parseFloat(s.v||0).toFixed(2)); break;
          case 'wait':     lines.push('TIMER T=' + parseFloat(s.t||0).toFixed(2)); break;
          case 'move': {
            var cName = posMap[s.posIdx]; if (!cName) break;
            if (s.moveType === 'LIN' || s.moveType === 'SLIN')
              lines.push('MOVL ' + cName + ' V=' + velMmS(curVel) + '.0 PL=0');
            else
              lines.push('MOVJ ' + cName + ' VJ=' + velPct(curVel) + '.00');
            break;
          }
          case 'circ': {
            var cv = posMap[s.viaIdx], ct = posMap[s.posIdx]; if (!cv||!ct) break;
            lines.push('MOVC ' + cv + ' V=' + velMmS(curVel) + '.0');
            lines.push('MOVC ' + ct + ' V=' + velMmS(curVel) + '.0');
            break;
          }
        }
      });

      // Programmfuß
      var ftr = (hf && hf.footer) ? hf.footer : 'END';
      ftr.split('\n').forEach(function(l) { lines.push(l); });
      return lines.join('\n');
    }
  };

  // ── Kawasaki AS Language  // ── Kawasaki AS Language ─────────────────────────────────────────────────
  IMPLS.kawasaki = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'kawasaki',
        header: function (t, b, vars) {
          var h = ['.PROGRAM main()'];
          vars.forEach(function (v) {
            h.push('  ' + v.name + ' = ' + (v.val || (v.varType === 'REAL' ? '1.0' : '1')));
          });
          if (t) h.push('  TOOL tool' + t);
          if (b) h.push('  BASE base' + b);
          return h;
        },
        footer: function () { return ['.END']; },
        moveL:   function (p, vel, idx) { return '  LMOVE p' + idx; },
        moveJ:   function (p, vel, idx) { return '  JMOVE p' + idx; },
        moveC:   function (pv, pt, vel, idx) { return '  C1MOVE p' + (idx - 1) + '\n  C2MOVE p' + idx; },
        halt:    function ()     { return '  HALT'; },
        dout:    function (n, v) { return v ? '  SIGNAL ' + n : '  SIGNAL -' + n; },
        dinWait: function (n)    { return '  WAIT SIG(' + n + ')'; },
        aout:    function (n, v) { return '  AOUT ' + n + ', ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AIN(' + n + ')'; },
        waitSec: function (t)    { return '  TWAIT ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  TOOL tool' + n; },
        base:    function (n)    { return '  BASE base' + n; },
        comment: function (t)    { return '  ;' + t; },
        ptpSection: function (positions) {
          var h = ['; == Point definitions =='];
          positions.forEach(function (pos, i) {
            h.push('; p' + (i + 1) + ' = TRANS(' + parseFloat(pos.X).toFixed(2) + ', ' + parseFloat(pos.Y).toFixed(2) + ', ' + parseFloat(pos.Z).toFixed(2) + ', ' + parseFloat(pos.A).toFixed(2) + ', ' + parseFloat(pos.B).toFixed(2) + ', ' + parseFloat(pos.C).toFixed(2) + ')');
          });
          return h;
        },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /LMOVE\s+(\w+)/, ptp: false },
        { rx: /JMOVE\s+(\w+)/, ptp: true  },
      ]);
    }
  };

  // ── Stäubli VAL3 / CS8 / CS9 ────────────────────────────────────────────
  IMPLS.staubli = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'staubli',
        header: function (t, b, vars) {
          var h = ['begin'];
          vars.forEach(function (v) {
            var typ = v.varType === 'BOOL' ? 'bool' : 'num';
            var val = v.val || (v.varType === 'BOOL' ? 'false' : v.varType === 'REAL' ? '1.0' : '1');
            h.push('  ' + typ + ' ' + v.name + ' := ' + val);
          });
          return h;
        },
        footer: function () { return ['end']; },
        moveL:   function (p, vel, idx) { return '  movel(p' + idx + ', tTool, mNomSpeed)'; },
        moveJ:   function (p, vel, idx) { return '  movej(p' + idx + ', tTool, mNomSpeed)'; },
        moveC:   function (pv, pt, vel, idx) { return '  movec(p' + (idx - 1) + ', p' + idx + ', tTool, mNomSpeed)'; },
        halt:    function ()     { return '  stopMove()'; },
        dout:    function (n, v) { return '  dout' + n + ' := ' + (v ? 'true' : 'false'); },
        dinWait: function (n)    { return '  wait(din' + n + ' == true)'; },
        aout:    function (n, v) { return '  aout' + n + ' := ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r := ain' + n; },
        waitSec: function (t)    { return '  delay(' + parseFloat(t).toFixed(1) + ')'; },
        tool:    function (n)    { return '  // Tool: tTool' + n; },
        base:    function (n)    { return '  // Frame: fBase' + n; },
        comment: function (t)    { return '  // ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /movel\(/, ptp: false },
        { rx: /movej\(/, ptp: true  },
      ]);
    }
  };

  // ── Universal Robots URScript / PolyScope ───────────────────────────────
  IMPLS.ur = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'ur',
        header: function (t, b, vars) {
          var h = ['def main():'];
          vars.forEach(function (v) {
            var val = v.val || (v.varType === 'BOOL' ? 'False' : v.varType === 'REAL' ? '0.0' : '0');
            h.push('  ' + v.name + ' = ' + val);
          });
          if (t) h.push('  set_tcp(p[0,0,0.120,0,0,0])  # tool ' + t);
          return h;
        },
        footer: function () { return ['end']; },
        moveL:   function (p, vel, idx, pos) { return '  movel(p[' + parseFloat(pos.X).toFixed(4) + ',' + parseFloat(pos.Y).toFixed(4) + ',' + parseFloat(pos.Z).toFixed(4) + ',0,0,0], a=1.2, v=' + (vel || 0.167).toFixed(4) + ')'; },
        moveJ:   function (p, vel, idx, pos) { return '  movej(p[' + parseFloat(pos.X).toFixed(4) + ',' + parseFloat(pos.Y).toFixed(4) + ',' + parseFloat(pos.Z).toFixed(4) + ',0,0,0], a=1.2, v=' + (vel || 0.167).toFixed(4) + ')'; },
        moveC:   function (pv, pt, vel)      { return '  movec(p[' + pv.X + ',' + pv.Y + ',' + pv.Z + ',0,0,0], p[' + pt.X + ',' + pt.Y + ',' + pt.Z + ',0,0,0], a=1.2, v=' + (vel || 0.167).toFixed(4) + ')'; },
        halt:    function ()     { return '  stopl(1.0)'; },
        dout:    function (n, v) { return '  set_standard_digital_out(' + (n - 1) + ', ' + (v ? 'True' : 'False') + ')'; },
        dinWait: function (n)    { return '  while not get_standard_digital_in(' + (n - 1) + '):\n    sleep(0.01)\n  end'; },
        aout:    function (n, v) { return '  set_standard_analog_out(' + (n - 1) + ', ' + parseFloat(v).toFixed(2) + ')'; },
        ainRead: function (n)    { return '  r = get_standard_analog_in(' + (n - 1) + ')'; },
        waitSec: function (t)    { return '  sleep(' + parseFloat(t).toFixed(1) + ')'; },
        tool:    function (n)    { return '  set_tcp(p[0,0,0.120,0,0,0])  # tool ' + n; },
        comment: function (t)    { return '  # ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /movel\(p\[(?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: false },
        { rx: /movej\(p\[(?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: true  },
      ]);
    }
  };

  // ── Adept V+ / legacy ───────────────────────────────────────────────────
  IMPLS.adept = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'adept',
        header: function (t, b, vars) {
          var h = ['.PROGRAM main()'];
          var vn = vars.map(function (v) { return v.name; }).join(', ');
          if (vn) h.push('  LOCAL ' + vn);
          if (t) h.push('  TOOL tool' + t);
          if (b) h.push('  BASE base' + b);
          return h;
        },
        footer: function () { return ['  HALT', '.END']; },
        moveL:   function (p, vel, idx) { return '  MOVES p' + idx; },
        moveJ:   function (p, vel, idx) { return '  MOVE p' + idx; },
        moveC:   function (pv, pt, vel, idx) { return '  MOVEC p' + (idx - 1) + ', p' + idx; },
        halt:    function ()     { return '  HALT'; },
        dout:    function (n, v) { return v ? '  SIGNAL ' + n : '  SIGNAL -' + n; },
        dinWait: function (n)    { return '  WAIT SIG(' + n + ')'; },
        aout:    function (n, v) { return '  AOUT ' + n + ' = ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AIN(' + n + ')'; },
        waitSec: function (t)    { return '  WAIT ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  TOOL tool' + n; },
        base:    function (n)    { return '  BASE base' + n; },
        comment: function (t)    { return '  ; ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVES\s+(\w+)/, ptp: false },
        { rx: /\bMOVE\b\s+(\w+)/, ptp: true },
      ]);
    }
  };

  // ── Omron V+ / eV+ / ACE ───────────────────────────────────────────────
  IMPLS.omron = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'omron',
        header: function (t, b, vars) {
          var h = ['.PROGRAM main()'];
          var vn = vars.map(function (v) { return v.name; }).join(', ');
          if (vn) h.push('  LOCAL ' + vn);
          if (t) h.push('  TOOL tool' + t);
          if (b) h.push('  BASE base' + b);
          return h;
        },
        footer: function () { return ['  HALT', '.END']; },
        moveL:   function (p, vel, idx) { return '  MOVES p' + idx; },
        moveJ:   function (p, vel, idx) { return '  JMOVE p' + idx; },
        moveC:   function (pv, pt, vel, idx) { return '  MOVEC p' + (idx - 1) + ', p' + idx; },
        halt:    function ()     { return '  HALT'; },
        dout:    function (n, v) { return v ? '  SIGNAL ' + n : '  SIGNAL -' + n; },
        dinWait: function (n)    { return '  WAIT SIG(' + n + ')'; },
        aout:    function (n, v) { return '  AOUT ' + n + ' = ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AIN(' + n + ')'; },
        waitSec: function (t)    { return '  TWAIT ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  TOOL tool' + n; },
        base:    function (n)    { return '  BASE base' + n; },
        comment: function (t)    { return '  ; ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVES\s+(\w+)/, ptp: false },
        { rx: /JMOVE\s+(\w+)/, ptp: true },
      ]);
    }
  };

  // ── Epson SPEL+ / RC+ ───────────────────────────────────────────────────
  IMPLS.epson = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'epson',
        header: function (t, b, vars) {
          var h = ['Function Main'];
          vars.forEach(function (v) {
            h.push('  ' + (v.varType === 'INT' ? 'Integer' : v.varType === 'REAL' ? 'Real' : 'Boolean') + ' ' + v.name);
          });
          if (t) h.push('  Tool ' + t);
          if (b) h.push('  Local ' + b);
          return h;
        },
        footer: function () { return ['Fend']; },
        moveL:   function (p, vel, idx) { return '  Move P' + idx; },
        moveJ:   function (p, vel, idx) { return '  Go P' + idx; },
        moveC:   function (pv, pt, vel, idx) { return '  Arc P' + (idx - 1) + ', P' + idx; },
        halt:    function ()     { return '  Halt'; },
        dout:    function (n, v) { return v ? '  On ' + n : '  Off ' + n; },
        dinWait: function (n)    { return '  Wait Sw(' + n + ')=On'; },
        aout:    function (n, v) { return '  AOut ' + n + ', ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AIn(' + n + ')'; },
        waitSec: function (t)    { return '  Wait ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  Tool ' + n; },
        base:    function (n)    { return '  Local ' + n; },
        comment: function (t)    { return "  ' " + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /Move\s+P(\d+)/, ptp: false },
        { rx: /Go\s+P(\d+)/,   ptp: true  },
      ]);
    }
  };

  // ── Comau PDL2 / C5G ───────────────────────────────────────────────────
  // Comau PDL2 — DIRECT/MIXED
  // Struktur: PROGRAM → VAR-Block (Variablen + Positionen) → BEGIN → Bewegungen → END
  IMPLS.comau = {
    _generate: function (pd) {
      if (!pd) return '';
      var positions = pd.positions || [];
      var steps = pd.steps || [];
      var lines = [];
      var hf = (typeof fmtHfLoad === 'function') ? fmtHfLoad('comau') : null;
      var toolN = 1, baseN = 1, vars = [];
      steps.forEach(function(s) {
        if (s.type === 'tool') toolN = s.n;
        if (s.type === 'base') baseN = s.n;
        if (s.type === 'var') vars.push(s);
      });

      // Programmkopf
      var hdr = (hf && hf.header) ? hf.header : 'PROGRAM PP_MAIN';
      hdr.split('\n').forEach(function(l) { lines.push(l); });

      // VAR-Block: Variablen + alle Positionen vorab deklarieren (MIXED)
      lines.push('VAR');
      vars.forEach(function(v) {
        var typ = v.varType === 'INT' ? 'INTEGER' : v.varType === 'REAL' ? 'REAL' : 'BOOLEAN';
        lines.push('  ' + v.name + ' : ' + typ);
      });
      // Positionen als POSITION-Typ
      positions.forEach(function(pos, i) {
        lines.push('  p' + (i+1) + ' : POSITION := POS(' +
          parseFloat(pos.X).toFixed(3) + ', ' + parseFloat(pos.Y).toFixed(3) + ', ' + parseFloat(pos.Z).toFixed(3) + ', ' +
          parseFloat(pos.A).toFixed(3) + ', ' + parseFloat(pos.B).toFixed(3) + ', ' + parseFloat(pos.C).toFixed(3) +
          ', \'' + (toolN) + '\')');
      });

      lines.push('BEGIN');
      lines.push('  $TOOL := tool' + toolN);
      lines.push('  $BASE := base' + baseN);

      var curVel = 0.167;
      steps.forEach(function(s) {
        switch (s.type) {
          case 'velcp':    curVel = s.v || 0.167; break;
          case 'comment':  lines.push('  -- ' + (s.text||'')); break;
          case 'halt':     lines.push('  PAUSE'); break;
          case 'dout':     lines.push('  $DOUT[' + s.n + '] := ' + ((s.v==='TRUE'||s.v==='1')?'TRUE':'FALSE')); break;
          case 'din':      lines.push('  WAIT FOR $DIN[' + s.n + '] = TRUE'); break;
          case 'aout':     lines.push('  $AOUT[' + s.n + '] := ' + parseFloat(s.v||0).toFixed(2)); break;
          case 'ain':      lines.push('  r := $AIN[' + s.n + ']'); break;
          case 'wait':     lines.push('  DELAY ' + parseFloat(s.t||0).toFixed(1)); break;
          case 'move': {
            var nm = 'p' + (s.posIdx + 1);
            if (s.moveType === 'LIN' || s.moveType === 'SLIN')
              lines.push('  MOVE LINEAR TO ' + nm + ' WITH $SPD_OPT:=SPD_MM_SEC,$SPD_LIN:=' + velMmS(curVel));
            else
              lines.push('  MOVE JOINT TO ' + nm);
            break;
          }
          case 'circ': {
            lines.push('  MOVE CIRCULAR TO p' + (s.posIdx+1) + ' VIA p' + (s.viaIdx+1));
            break;
          }
        }
      });

      var ftr = (hf && hf.footer) ? hf.footer : 'END PP_MAIN';
      ftr.split('\n').forEach(function(l) { lines.push(l); });
      return lines.join('\n');
    }
  };

  // ── AUBO Script / Teach Pendant  // ── AUBO Script / Teach Pendant ─────────────────────────────────────────
  IMPLS.aubo = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'aubo',
        header: function (t, b, vars) {
          var h = [];
          vars.forEach(function (v) {
            var val = v.val || (v.varType === 'BOOL' ? 'false' : v.varType === 'REAL' ? '1.0' : '1');
            h.push('local ' + v.name + ' = ' + val);
          });
          if (t) h.push('set_tool_kinematics_param({0,0,0.120,0,0,0})  -- tool ' + t);
          return h;
        },
        footer: function () { return ['robot_slow_stop()']; },
        moveL:   function (p, vel, idx, pos) { return 'move_line({' + parseFloat(pos.X).toFixed(4) + ',' + parseFloat(pos.Y).toFixed(4) + ',' + parseFloat(pos.Z).toFixed(4) + ',0,0,0}, true)'; },
        moveJ:   function (p, vel, idx, pos) { return 'move_joint({' + parseFloat(pos.X).toFixed(4) + ',' + parseFloat(pos.Y).toFixed(4) + ',' + parseFloat(pos.Z).toFixed(4) + ',0,0,0}, true)'; },
        halt:    function ()     { return 'robot_slow_stop()'; },
        dout:    function (n, v) { return 'set_robot_io_status("DO",' + n + ',' + (v ? 'true' : 'false') + ')'; },
        dinWait: function (n)    { return 'while get_robot_io_status("DI",' + n + ') == false do\n  sleep(0.01)\nend'; },
        aout:    function (n, v) { return 'set_robot_io_status("AO",' + n + ',' + parseFloat(v).toFixed(2) + ')'; },
        ainRead: function (n)    { return 'r = get_robot_io_status("AI",' + n + ')'; },
        waitSec: function (t)    { return 'sleep(' + parseFloat(t).toFixed(1) + ')'; },
        tool:    function (n)    { return 'set_tool_kinematics_param({0,0,0.120,0,0,0})  -- tool ' + n; },
        comment: function (t)    { return '-- ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /move_line\(\{(?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: false },
        { rx: /move_joint\(\{(?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: true },
      ]);
    }
  };

  // ── Dobot DobotScript / CR API ──────────────────────────────────────────
  IMPLS.dobot = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'dobot',
        header: function (t, b, vars) {
          var h = [];
          vars.forEach(function (v) {
            h.push(v.name + ' = ' + (v.val || (v.varType === 'BOOL' ? 'false' : v.varType === 'REAL' ? '1.0' : '1')));
          });
          if (t) h.push('Tool(' + t + ')');
          if (b) h.push('User(' + b + ')');
          return h;
        },
        footer: function () { return ['PauseScript()']; },
        moveL:   function (p, vel, idx, pos) { return 'MovL(' + parseFloat(pos.X).toFixed(3) + ',' + parseFloat(pos.Y).toFixed(3) + ',' + parseFloat(pos.Z).toFixed(3) + ',' + parseFloat(pos.A).toFixed(3) + ',' + parseFloat(pos.B).toFixed(3) + ',' + parseFloat(pos.C).toFixed(3) + ')'; },
        moveJ:   function (p, vel, idx, pos) { return 'MovJ(' + parseFloat(pos.X).toFixed(3) + ',' + parseFloat(pos.Y).toFixed(3) + ',' + parseFloat(pos.Z).toFixed(3) + ',' + parseFloat(pos.A).toFixed(3) + ',' + parseFloat(pos.B).toFixed(3) + ',' + parseFloat(pos.C).toFixed(3) + ')'; },
        moveC:   function (pv, pt, vel)      { return 'Arc(' + pv.X + ',' + pv.Y + ',' + pv.Z + ',0,0,0, ' + pt.X + ',' + pt.Y + ',' + pt.Z + ',0,0,0)'; },
        halt:    function ()     { return 'Stop()'; },
        dout:    function (n, v) { return 'DO(' + n + ', ' + (v ? 'ON' : 'OFF') + ')'; },
        dinWait: function (n)    { return 'while DI(' + n + ') == 0 do\n  Sleep(10)\nend'; },
        aout:    function (n, v) { return 'AO(' + n + ', ' + parseFloat(v).toFixed(2) + ')'; },
        ainRead: function (n)    { return 'r = AI(' + n + ')'; },
        waitSec: function (t)    { return 'Wait(' + Math.round(parseFloat(t) * 1000) + ')'; },
        tool:    function (n)    { return 'Tool(' + n + ')'; },
        base:    function (n)    { return 'User(' + n + ')'; },
        comment: function (t)    { return '# ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MovL\((?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: false },
        { rx: /MovJ\((?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: true  },
      ]);
    }
  };

  // ── Denso PACScript / RC8 ───────────────────────────────────────────────
  IMPLS.denso = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'denso',
        header: function (t, b, vars) {
          var h = ['PROGRAM mini_denso'];
          vars.forEach(function (v) {
            h.push('  ' + (v.varType === 'INT' ? 'INTEGER' : v.varType === 'REAL' ? 'REAL' : 'BOOLEAN') + ' ' + v.name);
          });
          if (t) h.push('  TOOL ' + t);
          if (b) h.push('  WORK ' + b);
          return h;
        },
        footer: function () { return ['END']; },
        moveL:   function (p, vel, idx) { return '  MOVE L, P' + idx; },
        moveJ:   function (p, vel, idx) { return '  MOVE P, P' + idx; },
        moveC:   function (pv, pt, vel, idx) { return '  MOVE C, P' + (idx - 1) + ', P' + idx; },
        halt:    function ()     { return '  STOP'; },
        dout:    function (n, v) { return '  IO[' + n + '] = ' + (v ? 'ON' : 'OFF'); },
        dinWait: function (n)    { return '  WAIT IO[' + n + '] = ON'; },
        aout:    function (n, v) { return '  AO[' + n + '] = ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AI[' + n + ']'; },
        waitSec: function (t)    { return '  DELAY ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  TOOL ' + n; },
        base:    function (n)    { return '  WORK ' + n; },
        comment: function (t)    { return "  ' " + t; },
        ptpSection: function (positions) {
          var h = [];
          positions.forEach(function (pos, i) {
            h.push("  ' P" + (i + 1) + " = (" + parseFloat(pos.X).toFixed(3) + ', ' + parseFloat(pos.Y).toFixed(3) + ', ' + parseFloat(pos.Z).toFixed(3) + ', ' + parseFloat(pos.A).toFixed(3) + ', ' + parseFloat(pos.B).toFixed(3) + ', ' + parseFloat(pos.C).toFixed(3) + ')');
          });
          return h;
        },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVE L,\s*P(\d+)/, ptp: false },
        { rx: /MOVE P,\s*P(\d+)/, ptp: true  },
      ]);
    }
  };

  // ── Nachi FD Robot Language ─────────────────────────────────────────────
  IMPLS.nachi = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'nachi',
        header: function (t, b, vars) {
          var h = ['PROGRAM MAIN_NACHI'];
          vars.forEach(function (v) { h.push('  ' + v.varType + ' ' + v.name); });
          if (t) h.push('  TOOL ' + t);
          if (b) h.push('  BASE ' + b);
          return h;
        },
        footer: function () { return ['END']; },
        moveL:   function (p, vel, idx) { return '  MOVEX L, P' + idx + ', S=' + velMmS(vel); },
        moveJ:   function (p, vel, idx) { return '  MOVEX A, P' + idx + ', S=' + velPct(vel); },
        moveC:   function (pv, pt, vel, idx) { return '  MOVEX C, P' + (idx - 1) + ', P' + idx + ', S=' + velMmS(vel); },
        halt:    function ()     { return '  STOP'; },
        dout:    function (n, v) { return '  OUT[' + n + ']=' + (v ? 'ON' : 'OFF'); },
        dinWait: function (n)    { return '  WAITI IN[' + n + ']=ON'; },
        aout:    function (n, v) { return '  AOUT[' + n + ']=' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AIN[' + n + ']'; },
        waitSec: function (t)    { return '  WAIT ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  TOOL ' + n; },
        base:    function (n)    { return '  BASE ' + n; },
        comment: function (t)    { return '  ;' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVEX L/, ptp: false },
        { rx: /MOVEX A/, ptp: true  },
      ]);
    }
  };

  // ── Hanwha HCR Task Builder ─────────────────────────────────────────────
  IMPLS.hanwha = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'hanwha',
        header: function (t, b, vars) {
          var h = ['// Hanwha HCR'];
          vars.forEach(function (v) {
            var typ = v.varType === 'INT' ? 'int' : v.varType === 'REAL' ? 'double' : 'bool';
            h.push(typ + ' ' + v.name + ' = ' + (v.val || (v.varType === 'BOOL' ? 'false' : v.varType === 'REAL' ? '1.0' : '1')));
          });
          if (t) h.push('setTCP(' + t + ')');
          if (b) h.push('setUserFrame(' + b + ')');
          return h;
        },
        footer: function () { return ['Stop()']; },
        moveL:   function (p, vel, idx) { return 'MoveL(P' + idx + ', speed=' + velMmS(vel) + ')'; },
        moveJ:   function (p, vel, idx) { return 'MoveJ(P' + idx + ', speed=' + velPct(vel) + ')'; },
        moveC:   function (pv, pt, vel, idx) { return 'MoveC(P' + (idx - 1) + ', P' + idx + ', speed=' + velMmS(vel) + ')'; },
        halt:    function ()     { return 'Stop()'; },
        dout:    function (n, v) { return 'SetDO(' + n + ', ' + (v ? 'true' : 'false') + ')'; },
        dinWait: function (n)    { return 'WaitDI(' + n + ', true)'; },
        aout:    function (n, v) { return 'SetAO(' + n + ', ' + parseFloat(v).toFixed(2) + ')'; },
        ainRead: function (n)    { return 'r = GetAI(' + n + ')'; },
        waitSec: function (t)    { return 'Wait(' + parseFloat(t).toFixed(1) + ')'; },
        tool:    function (n)    { return 'setTCP(' + n + ')'; },
        base:    function (n)    { return 'setUserFrame(' + n + ')'; },
        comment: function (t)    { return '// ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MoveL\(P(\d+)/, ptp: false },
        { rx: /MoveJ\(P(\d+)/, ptp: true  },
      ]);
    }
  };

  // ── igus Robot Control / iJC ─────────────────────────────────────────────
  IMPLS.igus = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'igus',
        header: function (t, b, vars) {
          var h = ['# igus iRC/iJC'];
          vars.forEach(function (v) {
            var typ = v.varType === 'INT' ? 'INT' : v.varType === 'REAL' ? 'REAL' : 'BOOL';
            h.push('VAR ' + v.name + ' : ' + typ + ' = ' + (v.val || (v.varType === 'BOOL' ? 'false' : v.varType === 'REAL' ? '1.0' : '1')));
          });
          if (t) h.push('SetTool(' + t + ')');
          if (b) h.push('SetBase(' + b + ')');
          return h;
        },
        footer: function () { return ['StopProgram()']; },
        moveL:   function (p, vel, idx) { return 'LinearMotion(P' + idx + ')'; },
        moveJ:   function (p, vel, idx) { return 'JointMotion(P' + idx + ')'; },
        halt:    function ()     { return 'StopProgram()'; },
        dout:    function (n, v) { return 'SetDOUT(DOUT' + n + ', ' + (v ? 'true' : 'false') + ')'; },
        dinWait: function (n)    { return 'WaitForSignal(DIN' + n + ', true)'; },
        aout:    function (n, v) { return 'SetAOUT(' + n + ', ' + parseFloat(v).toFixed(2) + ')'; },
        ainRead: function (n)    { return 'r = ReadAIN(' + n + ')'; },
        waitSec: function (t)    { return 'WaitTime(' + Math.round(parseFloat(t) * 1000) + ' ms)'; },
        tool:    function (n)    { return 'SetTool(' + n + ')'; },
        base:    function (n)    { return 'SetBase(' + n + ')'; },
        comment: function (t)    { return '# ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /LinearMotion\(P(\d+)\)/, ptp: false },
        { rx: /JointMotion\(P(\d+)\)/,  ptp: true  },
      ]);
    }
  };

  // ── Estun Robot Language / ER ────────────────────────────────────────────
  IMPLS.estun = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'estun',
        header: function (t, b, vars) {
          var h = ['PROGRAM MAIN_ESTUN'];
          vars.forEach(function (v) { h.push('  ' + v.varType + ' ' + v.name); });
          if (t) h.push('  TOOL ' + t);
          if (b) h.push('  USER ' + b);
          return h;
        },
        footer: function () { return ['END']; },
        moveL:   function (p, vel, idx) { return '  MOVL P' + String(idx).padStart(3, '0') + ' VL=' + velMmS(vel); },
        moveJ:   function (p, vel, idx) { return '  MOVJ P' + String(idx).padStart(3, '0') + ' VJ=' + velPct(vel); },
        moveC:   function (pv, pt, vel, idx) { return '  MOVC P' + String(idx - 1).padStart(3, '0') + ' P' + String(idx).padStart(3, '0') + ' VL=' + velMmS(vel); },
        halt:    function ()     { return '  PAUSE'; },
        dout:    function (n, v) { return '  DO[' + n + ']=' + (v ? 'ON' : 'OFF'); },
        dinWait: function (n)    { return '  WAIT DI[' + n + ']=ON'; },
        aout:    function (n, v) { return '  AO[' + n + ']=' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r = AI[' + n + ']'; },
        waitSec: function (t)    { return '  WAIT ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  TOOL ' + n; },
        base:    function (n)    { return '  USER ' + n; },
        comment: function (t)    { return '  ;' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVL\s+P\d+/, ptp: false },
        { rx: /MOVJ\s+P\d+/, ptp: true  },
      ]);
    }
  };

  // ── NEURA Teach / SDK ───────────────────────────────────────────────────
  IMPLS.neura = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'neura',
        header: function (t, b, vars) {
          var h = ['// NEURA SDK'];
          vars.forEach(function (v) {
            var typ = v.varType === 'INT' ? 'int' : v.varType === 'REAL' ? 'double' : 'bool';
            h.push(typ + ' ' + v.name + ' = ' + (v.val || (v.varType === 'BOOL' ? 'false' : v.varType === 'REAL' ? '1.0' : '1')) + ';');
          });
          if (t) h.push('robot.setTool("tool' + t + '");');
          if (b) h.push('robot.setBase("base' + b + '");');
          return h;
        },
        footer: function () { return ['robot.stop();']; },
        moveL:   function (p, vel, idx, pos) { return 'robot.moveL({' + parseFloat(pos.X).toFixed(3) + ', ' + parseFloat(pos.Y).toFixed(3) + ', ' + parseFloat(pos.Z).toFixed(3) + ', ' + parseFloat(pos.A).toFixed(3) + ', ' + parseFloat(pos.B).toFixed(3) + ', ' + parseFloat(pos.C).toFixed(3) + '});'; },
        moveJ:   function (p, vel, idx, pos) { return 'robot.moveJ({' + parseFloat(pos.X).toFixed(3) + ', ' + parseFloat(pos.Y).toFixed(3) + ', ' + parseFloat(pos.Z).toFixed(3) + ', ' + parseFloat(pos.A).toFixed(3) + ', ' + parseFloat(pos.B).toFixed(3) + ', ' + parseFloat(pos.C).toFixed(3) + '});'; },
        halt:    function ()     { return 'robot.stop();'; },
        dout:    function (n, v) { return 'robot.setDigitalOutput(' + n + ', ' + (v ? 'true' : 'false') + ');'; },
        dinWait: function (n)    { return 'robot.waitUntil(robot.digitalInput(' + n + '));'; },
        aout:    function (n, v) { return 'robot.setAnalogOutput(' + n + ', ' + parseFloat(v).toFixed(2) + ');'; },
        ainRead: function (n)    { return 'r = robot.analogInput(' + n + ');'; },
        waitSec: function (t)    { return 'robot.sleep(' + parseFloat(t).toFixed(1) + ');'; },
        tool:    function (n)    { return 'robot.setTool("tool' + n + '");'; },
        base:    function (n)    { return 'robot.setBase("base' + n + '");'; },
        comment: function (t)    { return '// ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /robot\.moveL\(\{(?<X>[\d.\-+]+),\s*(?<Y>[\d.\-+]+),\s*(?<Z>[\d.\-+]+)/, ptp: false },
        { rx: /robot\.moveJ\(\{(?<X>[\d.\-+]+),\s*(?<Y>[\d.\-+]+),\s*(?<Z>[\d.\-+]+)/, ptp: true  },
      ]);
    }
  };

  // ── MABI Steuerung ──────────────────────────────────────────────────────
  IMPLS.mabi = {
    _generate: function (pd) {
      return generate(pd, {
        _formatId: 'mabi',
        header: function (t, b, vars) {
          var h = ['// MABI Robot'];
          vars.forEach(function (v) {
            h.push(v.varType + ' ' + v.name + ' = ' + (v.val || (v.varType === 'BOOL' ? 'FALSE' : v.varType === 'REAL' ? '1.0' : '1')));
          });
          if (t) h.push('SetTool(' + t + ')');
          if (b) h.push('SetBase(' + b + ')');
          return h;
        },
        footer: function () { return ['Stop()']; },
        moveL:   function (p, vel, idx) { return 'MoveL(P' + idx + ')'; },
        moveJ:   function (p, vel, idx) { return 'MoveJ(P' + idx + ')'; },
        moveC:   function (pv, pt, vel, idx) { return 'MoveC(P' + (idx - 1) + ', P' + idx + ')'; },
        halt:    function ()     { return 'Stop()'; },
        dout:    function (n, v) { return 'SetDO(' + n + ', ' + (v ? 'TRUE' : 'FALSE') + ')'; },
        dinWait: function (n)    { return 'WaitDI(' + n + ', TRUE)'; },
        aout:    function (n, v) { return 'SetAO(' + n + ', ' + parseFloat(v).toFixed(2) + ')'; },
        ainRead: function (n)    { return 'r = ReadAI(' + n + ')'; },
        waitSec: function (t)    { return 'Wait(' + parseFloat(t).toFixed(1) + ')'; },
        tool:    function (n)    { return 'SetTool(' + n + ')'; },
        base:    function (n)    { return 'SetBase(' + n + ')'; },
        comment: function (t)    { return '// ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MoveL\(P(\d+)\)/, ptp: false },
        { rx: /MoveJ\(P(\d+)\)/, ptp: true  },
      ]);
    }
  };

  // ══════════════════════════════════════════════════════════════════════════
  // REGISTRATION
  // ══════════════════════════════════════════════════════════════════════════

  var META = [
    { id: 'abb',      label: 'ABB RAPID',        file: 'abb.png'      },
    { id: 'fanuc',    label: 'FANUC TP',          file: 'fanuc.png'    },
    { id: 'yaskawa',  label: 'Yaskawa INFORM',    file: 'yaskawa.png'  },
    { id: 'kawasaki', label: 'Kawasaki AS',       file: 'kawasaki.png' },
    { id: 'staubli',  label: 'Stäubli VAL3',      file: 'staubli.png'  },
    { id: 'ur',       label: 'Universal Robots',  file: 'universal.png'},
    { id: 'adept',    label: 'Adept V+',          file: 'adept.png'    },
    { id: 'omron',    label: 'Omron V+',          file: 'omron.png'    },
    { id: 'epson',    label: 'Epson SPEL+',       file: 'epson.png'    },
    { id: 'comau',    label: 'Comau PDL2',        file: 'comau.png'    },
    { id: 'aubo',     label: 'AUBO Script',       file: 'aubo.png'     },
    { id: 'dobot',    label: 'Dobot Script',      file: 'dobot.png'    },
    { id: 'denso',    label: 'Denso PACScript',   file: 'denso.png'    },
    { id: 'nachi',    label: 'Nachi FD',          file: 'nachi.png'    },
    { id: 'hanwha',   label: 'Hanwha HCR',        file: 'hanwha.png'   },
    { id: 'igus',     label: 'igus iRC',          file: 'igus.png'     },
    { id: 'estun',    label: 'Estun ER',          file: 'estun.png'    },
    { id: 'neura',    label: 'NEURA SDK',         file: 'neura.png'    },
    { id: 'mabi',     label: 'MABI Robot',        file: 'mabi.png'     },
  ];

  META.forEach(function (meta) {
    var impl = IMPLS[meta.id];
    if (!impl) return;
    FormatRegistry.register({
      id:    meta.id,
      label: meta.label,
      icon:  logo(meta.file),
      activate:   makeActivate(impl),
      deactivate: function () {},
      _generate:  impl._generate,
    });
  });


// ══════════════════════════════════════════════════════════════════════════
// FORMAT-BESCHREIBUNGEN (Tab 5 im Einstellungen-Popup)
// Quelle: CAD/CAM Reitz - Roboter-Befehlsvergleich (2026-05-05)
// ══════════════════════════════════════════════════════════════════════════
var FMT_DESCRIPTIONS = {
  kuka: {
    model: 'DIRECT_STREAM',
    note: 'Ausführung im SRC-Programm blockweise/zeilenweise. Positionen können zusätzlich in DAT-Dateien stehen; die Bewegungsfolge entsteht im SRC-Ablauf.',
    table: [
      ['PTP / Joint',   'PTP XP1'],
      ['Linear',        'LIN XP2'],
      ['Spline/SLIN',   'SPLINE / SLIN XP3 / ENDSPLINE'],
      ['Bogen/Kreis',   'CIRC XP3, XP4  (ARC als PP-Alias)'],
      ['Stop',          'HALT / BRAKE'],
      ['Digital I/O',   '$OUT[1]=TRUE / WAIT FOR $IN[1]'],
      ['Analog I/O',    'ANOUT ON 1 = 5.0 / r = $ANIN[1]'],
      ['Wait',          'WAIT SEC 1.0 / WAIT FOR $IN[1]'],
      ['Tool/Base',     '$TOOL = TOOL_DATA[1] / $BASE = BASE_DATA[1]'],
      ['Variablen',     'DECL INT i / DECL REAL r / DECL BOOL b'],
      ['CALL',          'MY_SUB(i, r, b)'],
    ],
    example: 'DEF PP_MAIN()\n DECL INT i\n DECL REAL r\n DECL BOOL b\n $TOOL=TOOL_DATA[1]\n $BASE=BASE_DATA[1]\n PTP XP1\n LIN XP2\n CIRC XP3, XP4\n $OUT[1]=TRUE\n WAIT SEC 0.5\n MY_SUB(i,r,b)\nEND',
    source: 'KUKA KRL Reference / KRL Syntax 8.x',
  },
  abb: {
    model: 'MIXED_MODULE',
    note: 'Daten/robtargets werden vor der PROC definiert; die Bewegung wird im PROC-Ablauf ausgeführt.',
    table: [
      ['PTP / Joint',   'MoveJ p1, v100, z10, t1\\WObj:=w1;'],
      ['Linear',        'MoveL p2, v100, z10, t1\\WObj:=w1;'],
      ['Spline',        'Kein SLIN-Standard; MoveL-Kette mit zone/Path'],
      ['Bogen/Kreis',   'MoveC p_mid, p_end, v100, z10, t1\\WObj:=w1;'],
      ['Stop',          'StopMove; / Stop;'],
      ['Digital I/O',   'SetDO do1, 1; / WaitDI di1, 1;'],
      ['Analog I/O',    'SetAO ao1, 5; / r := AInput(ai1);'],
      ['Wait',          'WaitTime 1; / WaitUntil b;'],
      ['Tool/Base',     'tooldata t1 / wobjdata w1'],
      ['Variablen',     'VAR num i; / VAR bool b;'],
      ['CALL',          'SubProc i, r, b;'],
    ],
    example: 'MODULE PP_MAIN\n VAR num i:=1;\n VAR bool b:=TRUE;\n PROC main()\n  MoveJ p1,v100,z10,t1\\WObj:=w1;\n  MoveL p2,v100,z10,t1\\WObj:=w1;\n  MoveC p3,p4,v100,z10,t1\\WObj:=w1;\n  SetDO do1,1;\n  WaitTime 0.5;\n ENDPROC\nENDMODULE',
    source: 'ABB RAPID Technical Reference Manual, search.abb.com',
  },
  fanuc: {
    model: 'POINT_SECTION_THEN_INSTRUCTIONS',
    note: 'TP-Dateien besitzen Programminstruktionen und Positionsbereich; Bewegungszeilen referenzieren P[]/PR[]. Die /POS-Sektion steht am Ende.',
    table: [
      ['PTP / Joint',   'J P[1] 100% FINE'],
      ['Linear',        'L P[2] 500mm/sec CNT50'],
      ['Spline',        'Kein SLIN-Standard; controller-optional'],
      ['Bogen/Kreis',   'C P[3] P[4] 500mm/sec FINE'],
      ['Stop',          'PAUSE / ABORT'],
      ['Digital I/O',   'DO[1]=ON / WAIT DI[1]=ON'],
      ['Analog I/O',    'AO[1]=50 / R[1]=AI[1]'],
      ['Wait',          'WAIT 1.00(sec) / WAIT DI[1]=ON'],
      ['Tool/Base',     'UTOOL_NUM=1 / UFRAME_NUM=1'],
      ['Variablen',     'R[n] numerisch / F[n] bool'],
      ['CALL',          'CALL SUBPROG(R[1])'],
    ],
    example: '/PROG PP_MAIN\n/MN\n 1: UTOOL_NUM=1 ;\n 2: UFRAME_NUM=1 ;\n 3:J P[1] 100% FINE ;\n 4:L P[2] 500mm/sec FINE ;\n 5:C P[3] P[4] 500mm/sec FINE ;\n 6: DO[1]=ON ;\n 7: WAIT 0.50(sec) ;\n/POS\n P[1]{...}\n/END',
    source: 'FANUC TP/KAREL Herstellerhandbuch (lizenzgebunden)',
  },
  yaskawa: {
    model: 'POINT_TABLE_THEN_INST',
    note: 'JBI-Datei enthält zuerst //POS mit Positionsdaten (C00000...), danach //INST mit NOP, MOVJ/MOVL/MOVC und END. Der PP muss zweiphasig ausgeben: Punkte sammeln, Referenzen erzeugen, Bewegungsjob schreiben.',
    table: [
      ['PTP / Joint',   'MOVJ C00000 VJ=80.00'],
      ['Linear',        'MOVL C00001 V=200.0 PL=0'],
      ['Spline',        'MOVS C00002 V=200.0'],
      ['Bogen/Kreis',   'MOVC C00003 V=200.0'],
      ['Stop',          'PAUSE / HOLD'],
      ['Digital I/O',   'DOUT OT#(1) ON / WAIT IN#(1)=ON'],
      ['Analog I/O',    'AOUT AO#(1) 5.0 / AIN je Controller'],
      ['Wait',          'TIMER T=0.50'],
      ['Tool/Base',     '///TOOL 1 im Header / ///USER n'],
      ['Variablen',     'Bxxx / Ixxx / Dxxx / Rxxx'],
      ['CALL',          'CALL JOB:SUBPROG ARGF\"1;2;3\"'],
    ],
    example: '/JOB\n//NAME PP_MAIN\n//POS\n///NPOS 3,0,0,0,0,0\n///TOOL 1\nC00000=...\nC00001=...\n//INST\nNOP\nMOVJ C00000 VJ=80.00\nMOVL C00001 V=200.0 PL=0\nDOUT OT#(1) ON\nTIMER T=0.50\nEND',
    source: 'Yaskawa Motoman Knowledge Center: YRC1000/DX200 INFORM LANGUAGE',
  },
  kawasaki: {
    model: 'DIRECT_STREAM',
    note: 'AS-Programm führt Befehle sequenziell aus; Punkte können vorher als Transformationen definiert werden.',
    table: [
      ['PTP / Joint',   'JMOVE p1'],
      ['Linear',        'LMOVE p2'],
      ['Spline',        'Kein direktes SLIN; LMOVE-Kette/CP-Option'],
      ['Bogen/Kreis',   'C1MOVE p_mid / C2MOVE p_end'],
      ['Stop',          'HALT / BRAKE'],
      ['Digital I/O',   'SIGNAL 1 / IF SIG(1001) THEN...'],
      ['Analog I/O',    'AOUT/AIN je I/O-Option'],
      ['Wait',          'WAIT SIG(1001) / TWAIT 0.5'],
      ['Tool/Base',     'TOOL tool1 / BASE base1'],
      ['Variablen',     'i = 1 / r = 1.0 / b = TRUE'],
      ['CALL',          'CALL sub(i)'],
    ],
    example: '.PROGRAM pp_main()\n TOOL tool1\n BASE base1\n JMOVE p1\n LMOVE p2\n C1MOVE p3\n C2MOVE p4\n SIGNAL 1\n TWAIT 0.5\n CALL sub(i)\n.END',
    source: 'Kawasaki AS Language Reference Manual, F/E/C Series',
  },
  staubli: {
    model: 'SCRIPTED_API',
    note: 'Bewegungsbefehle stehen im Programmablauf; Punkte, Tool und Frame sind typisierte Datenobjekte der VAL3-Applikation.',
    table: [
      ['PTP / Joint',   'movej(p1, tTool, mNomSpeed)'],
      ['Linear',        'movel(p2, tTool, mNomSpeed)'],
      ['Spline',        'Kein SLIN-Standard; movel-Kette/Blending'],
      ['Bogen/Kreis',   'movec(pMid, pEnd, tTool, mNomSpeed)'],
      ['Stop',          'stopMove() / halt'],
      ['Digital I/O',   'dioOut1 = true / b = dioIn1'],
      ['Analog I/O',    'aioOut1 = 5.0 / r = aioIn1'],
      ['Wait',          'delay(0.5) / waitEndMove()'],
      ['Tool/Base',     'tool tTool / frame fBase'],
      ['Variablen',     'num i / bool b'],
      ['CALL',          'sub(i, r, b)'],
    ],
    example: 'program pp_main()\nbegin\n num i\n bool b\n movej(p1,tTool,mNomSpeed)\n movel(p2,tTool,mNomSpeed)\n movec(p3,p4,tTool,mNomSpeed)\n dioOut1 = true\n delay(0.5)\nend',
    source: 'Stäubli VAL3 Reference Manual / Trainingsunterlagen',
  },
  ur: {
    model: 'SCRIPTED_DIRECT',
    note: 'URScript wird als Funktionsblock ausgeführt; Posen können inline, als Variablen oder extern erzeugt werden.',
    table: [
      ['PTP / Joint',   'movej(q_or_pose, a=1.2, v=0.25)'],
      ['Linear',        'movel(pose, a=1.2, v=0.25)'],
      ['Spline',        'Kein SLIN; ggf. movep/Blending'],
      ['Bogen/Kreis',   'movec(pose_via, pose_to, a=1.2, v=0.25)'],
      ['Stop',          'stopj(2.0) / stopl(1.0) / halt'],
      ['Digital I/O',   'set_standard_digital_out(0, True) / get_standard_digital_in(0)'],
      ['Analog I/O',    'set_standard_analog_out(0, 0.5) / get_standard_analog_in(0)'],
      ['Wait',          'sleep(0.5)'],
      ['Tool/Base',     'set_tcp(p[...]) / pose_trans(...)'],
      ['Variablen',     'i = 1 / r = 1.0 / b = True'],
      ['CALL',          'sub(i, r, b)'],
    ],
    example: 'def pp_main():\n i = 1\n set_tcp(p[0,0,0.2,0,0,0])\n movej(p[0.5,0,0.4,3.14,0,0])\n movel(p[0.6,0,0.4,3.14,0,0])\n set_standard_digital_out(0, True)\n sleep(0.5)\nend',
    source: 'Universal Robots Script Manual SW5.x',
  },
  adept: {
    model: 'DIRECT_STREAM',
    note: 'V+/Adept-Programme werden sequenziell ausgeführt; Punkte können als Daten/Speicherpositionen getrennt verwaltet werden.',
    table: [
      ['PTP / Joint',   'MOVE p1'],
      ['Linear',        'MOVES p2'],
      ['Spline',        'Kein SLIN; MOVES-Kette/Continuous Path'],
      ['Bogen/Kreis',   'MOVEC pMid, pEnd'],
      ['Stop',          'HALT / BRAKE / ESTOP'],
      ['Digital I/O',   'SIGNAL 1 / IF SIG(1) THEN...'],
      ['Analog I/O',    'AIO/AOUT je Controller-Option'],
      ['Wait',          'WAIT SIG(1) / TIMER'],
      ['Tool/Base',     'TOOL tool1 / BASE base1'],
      ['Variablen',     'AUTO i, r, b'],
      ['CALL',          'CALL sub(i)'],
    ],
    example: '.PROGRAM pp_main()\n AUTO i\n TOOL tool1\n BASE base1\n MOVE p1\n MOVES p2\n MOVEC p3,p4\n SIGNAL 1\n WAIT 0.5\n CALL sub(i)\n.END',
    source: 'Adept/Omron V+ Language Reference Guide',
  },
  omron: {
    model: 'DIRECT_STREAM',
    note: 'Omron/Adept V+ ist je nach Controller und eV+/V+ Variante leicht unterschiedlich.',
    table: [
      ['PTP / Joint',   'JMOVE p1'],
      ['Linear',        'MOVES p2'],
      ['Spline',        'Kein SLIN; MOVES-Kette/CP'],
      ['Bogen/Kreis',   'MOVEC pMid, pEnd'],
      ['Stop',          'HALT / BRAKE / ESTOP'],
      ['Digital I/O',   'SIGNAL 1 / WAIT SIG(1)'],
      ['Analog I/O',    'AOUT / AIN je Controller'],
      ['Wait',          'WAIT / TWAIT'],
      ['Tool/Base',     'TOOL tool1 / BASE base1'],
      ['Variablen',     'LOCAL i, r, b'],
      ['CALL',          'CALL sub(i)'],
    ],
    example: '.PROGRAM pp_main()\n LOCAL i\n TOOL tool1\n BASE base1\n JMOVE p1\n MOVES p2\n SIGNAL 1\n TWAIT 0.5\n CALL sub(i)\n.END',
    source: 'Omron V+ Keyword Reference Manual / eV+ Language Reference',
  },
  epson: {
    model: 'DIRECT_STREAM',
    note: 'SPEL+-Programm mit Funktionskopf; Bewegungen im Funktionsablauf. Punkte im Projekt/Controller gespeichert.',
    table: [
      ['PTP / Joint',   'Go P1 / Jump P1'],
      ['Linear',        'Move P2'],
      ['Spline',        'Kein SLIN; Move-Kette/CP'],
      ['Bogen/Kreis',   'Arc P3, P4 / Arc3 P2, P3, P4'],
      ['Stop',          'Halt / Quit'],
      ['Digital I/O',   'On 1 / Off 1 / If Sw(1)=On'],
      ['Analog I/O',    'AIn(1) / AOut 1, value'],
      ['Wait',          'Wait 0.5 / Wait Sw(1)=On'],
      ['Tool/Base',     'Tool 1 / Local 1'],
      ['Variablen',     'Integer i / Real r / Boolean b'],
      ['CALL',          'Call sub(i, r, b)'],
    ],
    example: 'Function main\n Integer i\n Tool 1\n Local 1\n Go P1\n Move P2\n Arc P3, P4\n On 1\n Wait 0.5\n Call sub(i,r,b)\nFend',
    source: 'Epson RC+ SPEL+ Language Reference',
  },
  comau: {
    model: 'DIRECT/MIXED',
    note: 'PDL2 ist blockstrukturiert; Positions- und Variablendaten können vor BEGIN stehen, Ausführung im BEGIN/END-Ablauf.',
    table: [
      ['PTP / Joint',   'MOVE JOINT TO p1'],
      ['Linear',        'MOVE LINEAR TO p2 WITH $SPD_OPT:=SPD_MM_SEC'],
      ['Spline',        'Kein SLIN-Standard; CP-Funktionen'],
      ['Bogen/Kreis',   'MOVE CIRCULAR TO pEnd VIA pMid'],
      ['Stop',          'PAUSE / ABORT'],
      ['Digital I/O',   '$DOUT[1] := TRUE / WAIT FOR $DIN[1]'],
      ['Analog I/O',    '$AOUT[1] := 5.0 / r := $AIN[1]'],
      ['Wait',          'DELAY 0.5 / WAIT FOR $DIN[1]'],
      ['Tool/Base',     '$TOOL := tool1 / $BASE := base1'],
      ['Variablen',     'VAR i : INTEGER / r : REAL / b : BOOLEAN'],
      ['CALL',          'CALL SUBPROG(i, r, b)'],
    ],
    example: 'PROGRAM PP_MAIN\nVAR\n i : INTEGER\nBEGIN\n $TOOL := tool1\n MOVE JOINT TO p1\n MOVE LINEAR TO p2\n $DOUT[1] := TRUE\n DELAY 0.5\n CALL SUBPROG(i,r,b)\nEND PP_MAIN',
    source: 'Comau PDL2 Programming Language / C5G controller documentation',
  },
  aubo: {
    model: 'SCRIPTED_API',
    note: 'Script/Lua-ähnliche Struktur; konkrete Funktionsnamen hängen von Controller- und SDK-Version ab.',
    table: [
      ['PTP / Joint',   'move_joint(joint_or_pose, a, v)'],
      ['Linear',        'move_line(pose, a, v)'],
      ['Spline',        'move_track(...) / Segmentkette'],
      ['Bogen/Kreis',   'move_track(..., ARC) je API'],
      ['Stop',          'robot_slow_stop() / robot_fast_stop()'],
      ['Digital I/O',   'set_robot_io_status("DO",...) / get_robot_io_status("DI",...)'],
      ['Analog I/O',    'set_robot_io_status("AO",...) / get_robot_io_status("AI",...)'],
      ['Wait',          'sleep(0.5)'],
      ['Tool/Base',     'set_tool_kinematics_param(...) / User/Base über API'],
      ['Variablen',     'local i = 1 / local r = 1.0 / local b = true'],
      ['CALL',          'sub(i, r, b)'],
    ],
    example: 'function pp_main()\n local i = 1\n set_tool_kinematics_param(tool1)\n move_joint(p1, 1.0, 0.5)\n move_line(p2, 1.0, 0.2)\n set_robot_io_status("DO",1,true)\n sleep(0.5)\nend',
    source: 'AUBO Teach Pendant Scripting Interface Manual',
  },
  dobot: {
    model: 'SCRIPTED_DIRECT',
    note: 'DobotScript/Lua-ähnlich; Befehlssatz je Modell/Controller validieren (Magician/MG400/CR/Nova).',
    table: [
      ['PTP / Joint',   'MovJ(P1)'],
      ['Linear',        'MovL(P2)'],
      ['Spline',        'Kein SLIN-Standard'],
      ['Bogen/Kreis',   'Arc(P3, P4) / Circle(P3, P4, count)'],
      ['Stop',          'Stop() / PauseScript()'],
      ['Digital I/O',   'DO(1, 1) / b = DI(1)'],
      ['Analog I/O',    'AO(1, 5.0) / r = AI(1)'],
      ['Wait',          'Wait(500) je Version / Sleep(0.5)'],
      ['Tool/Base',     'Tool(1) / User(1)'],
      ['Variablen',     'local i = 1 / local r = 1.0'],
      ['CALL',          'sub(i, r, b)'],
    ],
    example: 'function main()\n Tool(1)\n User(1)\n MovJ(P1)\n MovL(P2)\n Arc(P3, P4)\n DO(1,1)\n Wait(500)\nend',
    source: 'Dobot CR/Magician manuals and DobotScript/API references',
  },
  denso: {
    model: 'DIRECT_STREAM',
    note: 'Programmablauf mit Bewegungsbefehlen; Punkte/Variablen können separat im Projekt definiert sein.',
    table: [
      ['PTP / Joint',   'MOVE P, P1'],
      ['Linear',        'MOVE L, P2'],
      ['Spline',        'Kein SLIN-Standard; lineare Segmente/Interpolation'],
      ['Bogen/Kreis',   'MOVE C, P3, P4 / ARC je Controller'],
      ['Stop',          'STOP / PAUSE'],
      ['Digital I/O',   'SET IO[1] / WAIT IO[2]'],
      ['Analog I/O',    'AO[1] = 5.0 / r = AI[1]'],
      ['Wait',          'WAIT 0.5 / WAIT IO[2]'],
      ['Tool/Base',     'TOOL 1 / WORK 1'],
      ['Variablen',     'DIM i AS INTEGER / DIM r AS SINGLE'],
      ['CALL',          'CALL SUBPROG(i, r, b)'],
    ],
    example: 'PROGRAM PP_MAIN\nDIM i AS INTEGER\nTAKEARM\nTOOL 1\nMOVE P, P1\nMOVE L, P2\nSET IO[1]\nWAIT 0.5\nCALL SUBPROG(i,r,b)\nGIVEARM\nEND',
    source: 'DENSO PACScript / RC8 manuals and training material',
  },
  nachi: {
    model: 'DIRECT_STREAM',
    note: 'Bewegungen stehen in Job-Reihenfolge; genaue Syntax hängt von FD/CFD/AX und Systemsoftware ab.',
    table: [
      ['PTP / Joint',   'MOVEX A, P1, S=50'],
      ['Linear',        'MOVEX L, P2, S=200'],
      ['Spline',        'MOVEX S / Spline-Option je Controller'],
      ['Bogen/Kreis',   'MOVEX C, P3, P4, S=200'],
      ['Stop',          'STOP / PAUSE'],
      ['Digital I/O',   'OUT[1]=ON / IF IN[1]=ON THEN'],
      ['Analog I/O',    'AO[1]=5.0 / r=AI[1]'],
      ['Wait',          'WAIT 0.5 / WAIT IN[1]=ON'],
      ['Tool/Base',     'TOOL 1 / BASE 1'],
      ['Variablen',     'INT i / REAL r / BOOL b'],
      ['CALL',          'CALL SUBPROG(i, r, b)'],
    ],
    example: 'PROGRAM PP_MAIN\nINT i\nTOOL 1\nBASE 1\nMOVEX A, P1, S=50\nMOVEX L, P2, S=200\nOUT[1]=ON\nWAIT 0.5\nCALL SUBPROG(i,r,b)\nEND',
    source: 'Nachi FD Controller Robot Language documentation',
  },
  hanwha: {
    model: 'SCRIPTED/TEACH_TEMPLATE',
    note: 'HCR-Software bietet Teach-/Script-/API-Ebene; konkrete Syntax je Controllerstand prüfen.',
    table: [
      ['PTP / Joint',   'MoveJ(p1) / movej(p1)'],
      ['Linear',        'MoveL(p2) / movel(p2)'],
      ['Spline',        'Segmentkette/Blending je API'],
      ['Bogen/Kreis',   'MoveC(pMid,pEnd) falls verfügbar'],
      ['Stop',          'Stop/Halt-Funktion je API'],
      ['Digital I/O',   'SetDO(1, True) / GetDI(1)'],
      ['Analog I/O',    'SetAO(1, 5.0) / GetAI(1)'],
      ['Wait',          'Wait(0.5) / sleep(0.5)'],
      ['Tool/Base',     'setTCP(1) / setUserFrame(1)'],
      ['Variablen',     'int i / double r / bool b'],
      ['CALL',          'sub(i, r, b)'],
    ],
    example: 'def pp_main():\n SetTool(1)\n SetBase(1)\n MoveJ(p1)\n MoveL(p2)\n SetDO(1, True)\n Wait(0.5)\nend',
    source: 'Hanwha HCR manuals / Task-Builder; versionsabhängig',
  },
  igus: {
    model: 'PROJECT_FORMAT',
    note: 'iRC arbeitet stark projekt-/GUI-orientiert. Für den PP sollte ein XML/Programmbaustein-Modell statt reiner Textzeilen verwendet werden.',
    table: [
      ['PTP / Joint',   '<Move type="Joint" target="P1"/>'],
      ['Linear',        '<Move type="Linear" target="P2"/>'],
      ['Spline',        'Nicht einheitlich; ggf. Pfadsegmentliste'],
      ['Bogen/Kreis',   'Nicht einheitlich; falls vorhanden Kreis-/Splinebaustein'],
      ['Stop',          '<Stop/>'],
      ['Digital I/O',   '<SetDO channel="1" value="true"/>'],
      ['Analog I/O',    'Feldbus-/I/O-Modul abhängig'],
      ['Wait',          '<Wait time="0.5"/>'],
      ['Tool/Base',     'Tool/TCP im Projekt / Base/Frame im Projekt'],
      ['Variablen',     '<Var name="i" type="INT" value="1"/>'],
      ['CALL',          '<Call program="SUB" arg="i"/>'],
    ],
    example: '<Program name="PP_MAIN">\n <Var name="i" type="INT" value="1"/>\n <SetTool id="1"/>\n <Move type="Joint" target="P1"/>\n <Move type="Linear" target="P2"/>\n <SetDO channel="1" value="true"/>\n <Wait time="0.5"/>\n</Program>',
    source: 'igus Robot Control / iJC user guides',
  },
  estun: {
    model: 'INFORM-LIKE_TEMPLATE',
    note: 'Bei vielen Estun-Controllern werden MOVJ/MOVL/MOVC-ähnliche Befehle verwendet; genaue Syntax und Header je Controller prüfen.',
    table: [
      ['PTP / Joint',   'MOVJ P1, VJ=50'],
      ['Linear',        'MOVL P2, V=200'],
      ['Spline',        'MOVS / Spline-Option je Controller'],
      ['Bogen/Kreis',   'MOVC P3, P4, V=200'],
      ['Stop',          'STOP / PAUSE'],
      ['Digital I/O',   'DO[1]=ON / b = DI[1]'],
      ['Analog I/O',    'AO[1]=5.0 / r = AI[1]'],
      ['Wait',          'WAIT 0.5 / WAIT DI[1]=ON'],
      ['Tool/Base',     'TOOL 1 / USER/BASE 1'],
      ['Variablen',     'INT i / REAL r / BOOL b'],
      ['CALL',          'CALL SUBPROG(i, r, b)'],
    ],
    example: 'PROGRAM PP_MAIN\nINT i\nTOOL 1\nMOVJ P1, VJ=50\nMOVL P2, V=200\nDO[1]=ON\nWAIT 0.5\nCALL SUBPROG(i,r,b)\nEND',
    source: 'ESTUN Robot manuals; controller- und softwareabhängig',
  },
  neura: {
    model: 'SDK/APP_TEMPLATE',
    note: 'NEURA ist stärker API-/Teach-orientiert. Für den PP ist ein abstrakter Script-/API-Emitter sinnvoll; finale Syntax nur mit konkreter SDK-Version verbindlich.',
    table: [
      ['PTP / Joint',   'robot.moveJ(p1) / move_joint(p1)'],
      ['Linear',        'robot.moveL(p2) / move_linear(p2)'],
      ['Spline',        'trajectory/path API je SDK'],
      ['Bogen/Kreis',   'robot.moveC(pMid,pEnd) falls SDK-Funktion'],
      ['Stop',          'robot.stop() / stop_motion()'],
      ['Digital I/O',   'robot.setDigitalOutput(1, True) / robot.digitalInput(1)'],
      ['Analog I/O',    'robot.setAnalogOutput(1, 5.0) / robot.analogInput(1)'],
      ['Wait',          'robot.sleep(0.5) / wait(condition)'],
      ['Tool/Base',     'robot.setTool(tool1) / robot.setBase(base1)'],
      ['Variablen',     'int i = 1 / double r / bool b'],
      ['CALL',          'sub(i, r, b)'],
    ],
    example: 'def pp_main():\n robot.setTool(tool1)\n robot.moveJ(p1)\n robot.moveL(p2)\n robot.setDigitalOutput(1, True)\n robot.sleep(0.5)\nend',
    source: 'NEURA Teach/SDK: öffentliche Detailsyntax versionsabhängig',
  },
  mabi: {
    model: 'VENDOR_TEMPLATE',
    note: 'Öffentliche Syntax ist nicht ausreichend belastbar. Der PP sollte als konfigurierbares Template mit separatem Header, Footer, Punktliste und Bewegungsinstruktionen aufgebaut werden.',
    table: [
      ['PTP / Joint',   'PTP/JMOVE P1 (nach Controller-Mapping)'],
      ['Linear',        'LIN/LMOVE P2 (nach Controller-Mapping)'],
      ['Spline',        'Nach Handbuch'],
      ['Bogen/Kreis',   'ARC/CIRC P3, P4 nach Handbuch'],
      ['Stop',          'STOP/HALT nach Handbuch'],
      ['Digital I/O',   'SET_DO(1,TRUE) / GET_DI(1)'],
      ['Analog I/O',    'SET_AO(1,5.0) / GET_AI(1)'],
      ['Wait',          'WAIT 0.5 / WAIT_DI'],
      ['Tool/Base',     'SET_TOOL(1) / SET_BASE(1)'],
      ['Variablen',     'INT i / REAL r / BOOL b'],
      ['CALL',          'CALL SUBPROG(i,r,b)'],
    ],
    example: 'PROGRAM PP_MAIN\nINT i\nSET_TOOL(1)\nSET_BASE(1)\nMoveJ(P1)\nMoveL(P2)\nSET_DO(1,TRUE)\nWAIT 0.5\nCALL SUBPROG(i,r,b)\nEND',
    source: 'MABI: öffentliche Detailsyntax nicht ausreichend belastbar',
  },
};

// Tab 5: Beschreibung rendern
function fmtHfRenderDesc(id) {
  var el = document.getElementById('fmt-pane-desc');
  if (!el) return;
  var d = FMT_DESCRIPTIONS[id];
  if (!d) { el.innerHTML = '<div style="color:var(--txt3);padding:16px">Keine Beschreibung verfügbar.</div>'; return; }

  var rows = d.table.map(function(r) {
    return '<tr><td style="padding:4px 10px 4px 0;color:var(--txt2);white-space:nowrap">' + r[0] +
           '</td><td style="padding:4px 0;color:#d4d4d4;font-family:monospace;font-size:.88em">' + r[1] + '</td></tr>';
  }).join('');

  el.innerHTML =
    '<div style="display:flex;align-items:center;gap:10px;margin-bottom:10px">' +
      '<span style="background:var(--bg3);color:var(--acc);border:1px solid var(--acc);border-radius:3px;padding:2px 8px;font-size:.8em;font-weight:700">' + d.model + '</span>' +
    '</div>' +
    '<div style="color:var(--txt2);font-size:.88em;margin-bottom:12px;line-height:1.5">' + d.note + '</div>' +
    '<table style="width:100%;border-collapse:collapse;margin-bottom:14px">' + rows + '</table>' +
    '<div style="color:var(--txt2);font-size:.82em;font-weight:600;margin-bottom:6px">Mini-Beispiel</div>' +
    '<pre style="background:#1e1e1e;color:#d4d4d4;border:1px solid #474747;border-radius:3px;padding:10px 12px;font-size:.85em;line-height:1.5;overflow-x:auto;margin:0 0 10px">' +
      d.example.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/\n/g,'\n') +
    '</pre>' +
    '<div style="color:var(--txt3);font-size:.75em">Quelle: ' + d.source + '</div>';
}

})();

// ══════════════════════════════════════════════════════════════════════════
// PROGRAMMKOPF / PROGRAMMFUSS — Settings UI + localStorage
// ══════════════════════════════════════════════════════════════════════════

// ══════════════════════════════════════════════════════════════════════════
// EINSTELLUNGEN — Programmkopf/-fuß + Befehlsvorlagen je Format
// Quelle: CAD/CAM Reitz - Roboter-Befehlsvergleich fuer Postprozessoren (2026-05-05)
// Platzhalter: {N} Pos-Index, {X}{Y}{Z}{A}{B}{C} Koordinaten,
//   {VN} Via-Pos-Index, {VX}{VY}{VZ}{VA}{VB}{VC} Via-Koordinaten,
//   {VEL_MMS} mm/s, {VEL_PCT} %, {VEL_MS} m/s,
//   {TOOL} {BASE} Nummern, {CH} I/O-Kanal,
//   {VAL} Digital-Wert, {VAL_F} Analog-Wert, {T} Wartezeit s,
//   {TYPE} {NAME} {INITVAL} Variable, {PROG} {ARGS} CALL
// ══════════════════════════════════════════════════════════════════════════

var FMT_HF_DEFAULTS = {
  kuka:     { header: 'DEF PP_MAIN()\n ;INIT / BAS optional\n BAS(#INITMOV,0)', footer: 'END' },
  abb:      { header: "MODULE PP_MAIN\n PERS tooldata t1 := [TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,0.001],[1,0,0,0],0,0,0]];\n PERS wobjdata w1 := [FALSE,TRUE,'',[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];\n PROC main()", footer: 'ENDPROC\nENDMODULE' },
  fanuc:    { header: '/PROG PP_MAIN\n/ATTR\nOWNER = MNEDITOR;\n/MN', footer: '/END' },
  yaskawa:  { header: '/JOB\n//NAME PP_MAIN', footer: 'END' },
  kawasaki: { header: '.PROGRAM pp_main()', footer: '.END' },
  staubli:  { header: 'program pp_main()\nbegin', footer: 'end' },
  ur:       { header: 'def pp_main():', footer: 'end' },
  adept:    { header: '.PROGRAM pp_main()', footer: '.END' },
  omron:    { header: '.PROGRAM pp_main()', footer: '.END' },
  epson:    { header: 'Function main', footer: 'Fend' },
  comau:    { header: 'PROGRAM PP_MAIN', footer: 'END PP_MAIN' },
  aubo:     { header: 'function pp_main()', footer: 'end' },
  dobot:    { header: 'function main()', footer: 'end' },
  denso:    { header: 'PROGRAM PP_MAIN\nTAKEARM\nSPEED 50', footer: 'GIVEARM\nEND' },
  nachi:    { header: 'PROGRAM PP_MAIN', footer: 'END' },
  hanwha:   { header: 'def pp_main():', footer: 'end' },
  igus:     { header: '<Program name="PP_MAIN">', footer: '</Program>' },
  estun:    { header: 'PROGRAM PP_MAIN', footer: 'END' },
  neura:    { header: 'def pp_main():', footer: 'end' },
  mabi:     { header: 'PROGRAM PP_MAIN ; nach MABI-Handbuch anpassen', footer: 'END' },
};

var FMT_CMD_DEFAULTS = {
  kuka: {
    moveJ:   'PTP {X {X},Y {Y},Z {Z},A {A},B {B},C {C}}',
    moveL:   'LIN {X {X},Y {Y},Z {Z},A {A},B {B},C {C}}',
    moveC:   'CIRC {X {VX},Y {VY},Z {VZ},A {VA},B {VB},C {VC}}, {X {X},Y {Y},Z {Z},A {A},B {B},C {C}}',
    halt:    'HALT',
    dout:    '$OUT[{CH}]={VAL}',
    din:     'WAIT FOR $IN[{CH}]',
    aout:    '$ANOUT[{CH}]={VAL_F}',
    ain:     'r = $ANIN[{CH}]',
    wait:    'WAIT SEC {T}',
    tool:    '$TOOL=TOOL_DATA[{N}]',
    base:    '$BASE=BASE_DATA[{N}]',
    varInt:  'DECL INT {NAME}={INITVAL}',
    varReal: 'DECL REAL {NAME}={INITVAL}',
    varBool: 'DECL BOOL {NAME}={INITVAL}',
    call:    '{PROG}({ARGS})',
  },
  abb: {
    moveJ:   '    MoveJ p{N}, v{VEL_MMS}, z10, tool{TOOL}\\WObj:=wobj{BASE};',
    moveL:   '    MoveL p{N}, v{VEL_MMS}, fine, tool{TOOL}\\WObj:=wobj{BASE};',
    moveC:   '    MoveC p{VN}, p{N}, v{VEL_MMS}, fine, tool{TOOL}\\WObj:=wobj{BASE};',
    halt:    '    Stop;',
    dout:    '    SetDO do{CH}, {VAL};',
    din:     '    WaitDI di{CH}, 1;',
    aout:    '    SetAO ao{CH}, {VAL_F};',
    ain:     '    r := AInput(ai{CH});',
    wait:    '    WaitTime {T};',
    tool:    '    ! Tool: tool{N}',
    base:    '    ! WObj: wobj{N}',
    varInt:  '  VAR num {NAME} := {INITVAL};',
    varReal: '  VAR num {NAME} := {INITVAL};',
    varBool: '  VAR bool {NAME} := {INITVAL};',
    call:    '    {PROG} {ARGS};',
  },
  fanuc: {
    moveJ:   ' {LN}:J P[{N}] {VEL_PCT}% FINE ;',
    moveL:   ' {LN}:L P[{N}] {VEL_MMS}mm/sec FINE ;',
    moveC:   ' {LN}:C P[{VN}]\n   P[{N}] {VEL_MMS}mm/sec FINE ;',
    halt:    ' {LN}: PAUSE ;',
    dout:    ' {LN}: DO[{CH}]={VAL} ;',
    din:     ' {LN}: WAIT DI[{CH}]=ON ;',
    aout:    ' {LN}: AO[{CH}]={VAL_F} ;',
    ain:     ' {LN}: R[1]=AI[{CH}] ;',
    wait:    ' {LN}: WAIT {T}(sec) ;',
    tool:    ' {LN}: UTOOL_NUM={N} ;',
    base:    ' {LN}: UFRAME_NUM={N} ;',
    varInt:  ' {LN}: R[{N}]={INITVAL} ;',
    varReal: ' {LN}: R[{N}]={INITVAL} ;',
    varBool: ' {LN}: F[{N}]=(OFF) ;',
    call:    ' {LN}: CALL {PROG}({ARGS}) ;',
  },
  yaskawa: {
    moveJ:   'MOVJ {CNAME} VJ={VEL_PCT}.00',
    moveL:   'MOVL {CNAME} V={VEL_MMS}.0 PL=0',
    moveC:   'MOVC {VCNAME} V={VEL_MMS}.0\nMOVC {CNAME} V={VEL_MMS}.0',
    halt:    'PAUSE',
    dout:    'DOUT OT#({CH}) {VAL}',
    din:     'WAIT IN#({CH})=ON',
    aout:    'AOUT AO#({CH}) {VAL_F}',
    ain:     'AIN AI#({CH}) R000',
    wait:    'TIMER T={T}',
    tool:    'TOOL {N}',
    base:    '\' BASE: {N}',
    varInt:  'SET I000 {INITVAL}',
    varReal: 'SET R000 {INITVAL}',
    varBool: 'SET B000 {INITVAL}',
    call:    'CALL JOB:{PROG} ARGF"{ARGS}"',
  },
  kawasaki: {
    moveJ:   '  JMOVE p{N}',
    moveL:   '  LMOVE p{N}',
    moveC:   '  C1MOVE p{VN}\n  C2MOVE p{N}',
    halt:    '  HALT',
    dout:    '  SIGNAL {CH}',
    din:     '  WAIT SIG({CH})',
    aout:    '  AOUT {CH}, {VAL_F}',
    ain:     '  r = AIN({CH})',
    wait:    '  TWAIT {T}',
    tool:    '  TOOL tool{N}',
    base:    '  BASE base{N}',
    varInt:  '  {NAME} = {INITVAL}',
    varReal: '  {NAME} = {INITVAL}',
    varBool: '  {NAME} = {INITVAL}',
    call:    '  CALL {PROG}({ARGS})',
  },
  staubli: {
    moveJ:   '  movej(p{N}, tTool, mNomSpeed)',
    moveL:   '  movel(p{N}, tTool, mNomSpeed)',
    moveC:   '  movec(p{VN}, p{N}, tTool, mNomSpeed)',
    halt:    '  stopMove()',
    dout:    '  dout{CH} := {VAL}',
    din:     '  wait(din{CH} == true)',
    aout:    '  aout{CH} := {VAL_F}',
    ain:     '  r := ain{CH}',
    wait:    '  delay({T})',
    tool:    '  ! Tool: tTool{N}',
    base:    '  ! Frame: fBase{N}',
    varInt:  '  num {NAME} := {INITVAL}',
    varReal: '  num {NAME} := {INITVAL}',
    varBool: '  bool {NAME} := {INITVAL}',
    call:    '  {PROG}({ARGS})',
  },
  ur: {
    moveJ:   '  movej(p[{X},{Y},{Z},0,0,0], a=1.2, v={VEL_MS})',
    moveL:   '  movel(p[{X},{Y},{Z},0,0,0], a=1.2, v={VEL_MS})',
    moveC:   '  movec(p[{VX},{VY},{VZ},0,0,0], p[{X},{Y},{Z},0,0,0], a=1.2, v={VEL_MS})',
    halt:    '  stopl(1.0)',
    dout:    '  set_standard_digital_out({CH}, {VAL})',
    din:     '  while not get_standard_digital_in({CH}):\n    sleep(0.01)\n  end',
    aout:    '  set_standard_analog_out({CH}, {VAL_F})',
    ain:     '  r = get_standard_analog_in({CH})',
    wait:    '  sleep({T})',
    tool:    '  set_tcp(p[0,0,0.120,0,0,0])  # tool{N}',
    base:    '  # base: {N}',
    varInt:  '  {NAME} = {INITVAL}',
    varReal: '  {NAME} = {INITVAL}',
    varBool: '  {NAME} = {INITVAL}',
    call:    '  {PROG}({ARGS})',
  },
  adept: {
    moveJ:   '  MOVE p{N}',
    moveL:   '  MOVES p{N}',
    moveC:   '  MOVEC p{VN}, p{N}',
    halt:    '  HALT',
    dout:    '  SIGNAL {CH}',
    din:     '  WAIT SIG({CH})',
    aout:    '  AOUT {CH} = {VAL_F}',
    ain:     '  r = AIN({CH})',
    wait:    '  WAIT {T}',
    tool:    '  TOOL tool{N}',
    base:    '  BASE base{N}',
    varInt:  '  LOCAL {NAME}\n  {NAME} = {INITVAL}',
    varReal: '  LOCAL {NAME}\n  {NAME} = {INITVAL}',
    varBool: '  LOCAL {NAME}\n  {NAME} = {INITVAL}',
    call:    '  CALL {PROG}({ARGS})',
  },
  omron: {
    moveJ:   '  JMOVE p{N}',
    moveL:   '  MOVES p{N}',
    moveC:   '  MOVEC p{VN}, p{N}',
    halt:    '  HALT',
    dout:    '  SIGNAL {CH}',
    din:     '  WAIT SIG({CH})',
    aout:    '  AOUT {CH} = {VAL_F}',
    ain:     '  r = AIN({CH})',
    wait:    '  TWAIT {T}',
    tool:    '  TOOL tool{N}',
    base:    '  BASE base{N}',
    varInt:  '  LOCAL {NAME}\n  {NAME} = {INITVAL}',
    varReal: '  LOCAL {NAME}\n  {NAME} = {INITVAL}',
    varBool: '  LOCAL {NAME}\n  {NAME} = {INITVAL}',
    call:    '  CALL {PROG}({ARGS})',
  },
  epson: {
    moveJ:   '  Go P{N}',
    moveL:   '  Move P{N}',
    moveC:   '  Arc P{VN}, P{N}',
    halt:    '  Halt',
    dout:    '  On {CH}',
    din:     '  Wait Sw({CH})=On',
    aout:    '  AOut {CH}, {VAL_F}',
    ain:     '  r = AIn({CH})',
    wait:    '  Wait {T}',
    tool:    '  Tool {N}',
    base:    '  Local {N}',
    varInt:  '  Integer {NAME}',
    varReal: '  Real {NAME}',
    varBool: '  Boolean {NAME}',
    call:    '  Call {PROG}({ARGS})',
  },
  comau: {
    moveJ:   '  MOVE JOINT TO p{N}',
    moveL:   '  MOVE LINEAR TO p{N} WITH $SPD_OPT:=SPD_MM_SEC,$SPD_LIN:={VEL_MMS}',
    moveC:   '  MOVE CIRCULAR TO p{N} VIA p{VN}',
    halt:    '  PAUSE',
    dout:    '  $DOUT[{CH}] := {VAL}',
    din:     '  WAIT FOR $DIN[{CH}] = TRUE',
    aout:    '  $AOUT[{CH}] := {VAL_F}',
    ain:     '  r := $AIN[{CH}]',
    wait:    '  DELAY {T}',
    tool:    '  $TOOL := tool{N}',
    base:    '  $BASE := base{N}',
    varInt:  '  {NAME} : INTEGER',
    varReal: '  {NAME} : REAL',
    varBool: '  {NAME} : BOOLEAN',
    call:    '  CALL {PROG}({ARGS})',
  },
  aubo: {
    moveJ:   'move_joint({{{X},{Y},{Z},0,0,0}}, true)',
    moveL:   'move_line({{{X},{Y},{Z},0,0,0}}, true)',
    moveC:   'add_waypoint(p{VN})\nadd_waypoint(p{N})\nmove_track("arc", true)',
    halt:    'robot_slow_stop()',
    dout:    'set_robot_io_status("DO",{CH},{VAL})',
    din:     'while get_robot_io_status("DI",{CH}) == false do\n  sleep(0.01)\nend',
    aout:    'set_robot_io_status("AO",{CH},{VAL_F})',
    ain:     'r = get_robot_io_status("AI",{CH})',
    wait:    'sleep({T})',
    tool:    'set_tool_kinematics_param(tool{N})',
    base:    '-- base: {N}',
    varInt:  'local {NAME} = {INITVAL}',
    varReal: 'local {NAME} = {INITVAL}',
    varBool: 'local {NAME} = {INITVAL}',
    call:    '{PROG}({ARGS})',
  },
  dobot: {
    moveJ:   'MovJ({X},{Y},{Z},{A},{B},{C})',
    moveL:   'MovL({X},{Y},{Z},{A},{B},{C})',
    moveC:   'Arc({VX},{VY},{VZ},{VA},{VB},{VC}, {X},{Y},{Z},{A},{B},{C})',
    halt:    'Stop()',
    dout:    'DO({CH}, {VAL})',
    din:     'while DI({CH}) == 0 do\n  Sleep(10)\nend',
    aout:    'AO({CH}, {VAL_F})',
    ain:     'r = AI({CH})',
    wait:    'Wait({T_MS})',
    tool:    'Tool({N})',
    base:    'User({N})',
    varInt:  'local {NAME} = {INITVAL}',
    varReal: 'local {NAME} = {INITVAL}',
    varBool: 'local {NAME} = {INITVAL}',
    call:    '{PROG}({ARGS})',
  },
  denso: {
    moveJ:   '  MOVE P, P{N}',
    moveL:   '  MOVE L, P{N}',
    moveC:   '  MOVE C, P{VN}, P{N}',
    halt:    '  STOP',
    dout:    '  SET IO[{CH}]',
    din:     '  WAIT IO[{CH}]',
    aout:    '  AO[{CH}] = {VAL_F}',
    ain:     '  r = AI[{CH}]',
    wait:    '  WAIT {T}',
    tool:    '  TOOL {N}',
    base:    '  WORK {N}',
    varInt:  '  DIM {NAME} AS INTEGER',
    varReal: '  DIM {NAME} AS SINGLE',
    varBool: '  DIM {NAME} AS BOOLEAN',
    call:    '  CALL {PROG}({ARGS})',
  },
  nachi: {
    moveJ:   '  MOVEX A, P{N}, S={VEL_PCT}',
    moveL:   '  MOVEX L, P{N}, S={VEL_MMS}',
    moveC:   '  MOVEX C, P{VN}, P{N}, S={VEL_MMS}',
    halt:    '  STOP',
    dout:    '  OUT[{CH}]={VAL}',
    din:     '  WAITI IN[{CH}]=ON',
    aout:    '  AOUT[{CH}]={VAL_F}',
    ain:     '  r = AIN[{CH}]',
    wait:    '  WAIT {T}',
    tool:    '  TOOL {N}',
    base:    '  BASE {N}',
    varInt:  '  INT {NAME}',
    varReal: '  REAL {NAME}',
    varBool: '  BOOL {NAME}',
    call:    '  CALL {PROG}({ARGS})',
  },
  hanwha: {
    moveJ:   'MoveJ(P{N}, speed={VEL_PCT})',
    moveL:   'MoveL(P{N}, speed={VEL_MMS})',
    moveC:   'MoveC(P{VN}, P{N}, speed={VEL_MMS})',
    halt:    'Stop()',
    dout:    'SetDO({CH}, {VAL})',
    din:     'WaitDI({CH}, true)',
    aout:    'SetAO({CH}, {VAL_F})',
    ain:     'r = GetAI({CH})',
    wait:    'Wait({T})',
    tool:    'setTCP({N})',
    base:    'setUserFrame({N})',
    varInt:  'int {NAME} = {INITVAL}',
    varReal: 'double {NAME} = {INITVAL}',
    varBool: 'bool {NAME} = {INITVAL}',
    call:    '{PROG}({ARGS})',
  },
  igus: {
    moveJ:   'JointMotion(P{N})',
    moveL:   'LinearMotion(P{N})',
    moveC:   '; arc not standard in iRC',
    halt:    'StopProgram()',
    dout:    'SetDOUT(DOUT{CH}, {VAL})',
    din:     'WaitForSignal(DIN{CH}, true)',
    aout:    'SetAOUT({CH}, {VAL_F})',
    ain:     'r = ReadAIN({CH})',
    wait:    'WaitTime({T_MS} ms)',
    tool:    'SetTool({N})',
    base:    'SetBase({N})',
    varInt:  'VAR {NAME} : INT = {INITVAL}',
    varReal: 'VAR {NAME} : REAL = {INITVAL}',
    varBool: 'VAR {NAME} : BOOL = {INITVAL}',
    call:    '<Call program="{PROG}" arg="{ARGS}"/>',
  },
  estun: {
    moveJ:   '  MOVJ P{N}, VJ={VEL_PCT}',
    moveL:   '  MOVL P{N}, V={VEL_MMS}',
    moveC:   '  MOVC P{VN}, P{N}, V={VEL_MMS}',
    halt:    '  PAUSE',
    dout:    '  DO[{CH}]={VAL}',
    din:     '  WAIT DI[{CH}]=ON',
    aout:    '  AO[{CH}]={VAL_F}',
    ain:     '  r = AI[{CH}]',
    wait:    '  WAIT {T}',
    tool:    '  TOOL {N}',
    base:    '  USER {N}',
    varInt:  '  INT {NAME}',
    varReal: '  REAL {NAME}',
    varBool: '  BOOL {NAME}',
    call:    '  CALL {PROG}({ARGS})',
  },
  neura: {
    moveJ:   'robot.moveJ({X}, {Y}, {Z}, {A}, {B}, {C});',
    moveL:   'robot.moveL({X}, {Y}, {Z}, {A}, {B}, {C});',
    moveC:   'robot.moveC(p{VN}, p{N});',
    halt:    'robot.stop();',
    dout:    'robot.setDigitalOutput({CH}, {VAL});',
    din:     'robot.waitUntil(robot.digitalInput({CH}));',
    aout:    'robot.setAnalogOutput({CH}, {VAL_F});',
    ain:     'r = robot.analogInput({CH});',
    wait:    'robot.sleep({T});',
    tool:    'robot.setTool("tool{N}");',
    base:    'robot.setBase("base{N}");',
    varInt:  'int {NAME} = {INITVAL};',
    varReal: 'double {NAME} = {INITVAL};',
    varBool: 'bool {NAME} = {INITVAL};',
    call:    '{PROG}({ARGS});',
  },
  mabi: {
    moveJ:   'MoveJ(P{N})',
    moveL:   'MoveL(P{N})',
    moveC:   'MoveC(P{VN}, P{N})',
    halt:    'Stop()',
    dout:    'SetDO({CH}, {VAL})',
    din:     'WaitDI({CH}, TRUE)',
    aout:    'SetAO({CH}, {VAL_F})',
    ain:     'r = ReadAI({CH})',
    wait:    'Wait({T})',
    tool:    'SetTool({N})',
    base:    'SetBase({N})',
    varInt:  'INT {NAME} = {INITVAL}',
    varReal: 'REAL {NAME} = {INITVAL}',
    varBool: 'BOOL {NAME} = {INITVAL}',
    call:    'CALL {PROG}({ARGS})',
  },
};

// ── Template-Substitution ─────────────────────────────────────────────────
function subTpl(tpl, vars) {
  return tpl.replace(/\{([A-Z0-9_]+)\}/g, function(_, k) {
    return (vars[k] !== undefined && vars[k] !== null) ? vars[k] : '{' + k + '}';
  });
}

// Vorlage laden: localStorage → FMT_CMD_DEFAULTS → null
function applyTpl(formatId, key, vars) {
  var tpl = null;
  try {
    var stored = localStorage.getItem('robsimul_cmd_' + formatId);
    if (stored) { var cmds = JSON.parse(stored); if (cmds[key] !== undefined) tpl = cmds[key]; }
  } catch(e) {}
  if (tpl === null) {
    var def = FMT_CMD_DEFAULTS[formatId];
    if (def && def[key] !== undefined) tpl = def[key];
  }
  return tpl !== null ? subTpl(tpl, vars) : null;
}

// ── Speicher-Helfer ───────────────────────────────────────────────────────
var _fmtHfCurrentId = null;

function fmtHfLoad(id) {
  try {
    var stored = localStorage.getItem('robsimul_hf_' + id);
    if (stored) return JSON.parse(stored);
  } catch(e) {}
  return FMT_HF_DEFAULTS[id] || { header: '', footer: '' };
}

function fmtCmdLoad(id) {
  try {
    var stored = localStorage.getItem('robsimul_cmd_' + id);
    if (stored) return JSON.parse(stored);
  } catch(e) {}
  return FMT_CMD_DEFAULTS[id] || {};
}

function fmtHfSaveKey(id, header, footer) {
  try { localStorage.setItem('robsimul_hf_' + id, JSON.stringify({ header: header, footer: footer })); } catch(e) {}
}

function fmtCmdSaveKey(id, cmds) {
  try { localStorage.setItem('robsimul_cmd_' + id, JSON.stringify(cmds)); } catch(e) {}
}

function fmtHfRemoveKey(id) {
  try { localStorage.removeItem('robsimul_hf_' + id); } catch(e) {}
}

function fmtCmdRemoveKey(id) {
  try { localStorage.removeItem('robsimul_cmd_' + id); } catch(e) {}
}

// ── Drag-Logik ────────────────────────────────────────────────────────────
var _fmtHfDragInit = false;
function _initFmtHfDrag() {
  if (_fmtHfDragInit) return;
  _fmtHfDragInit = true;
  var popup = document.getElementById('fmt-hf-popup');
  var handle = document.getElementById('fmt-hf-drag');
  if (!popup || !handle) return;
  var dx = 0, dy = 0, dragging = false;
  handle.addEventListener('mousedown', function(e) {
    if (e.button !== 0) return;
    dragging = true;
    var r = popup.getBoundingClientRect();
    dx = e.clientX - r.left;
    dy = e.clientY - r.top;
    popup.style.right = 'auto';
    e.preventDefault();
  });
  document.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    popup.style.left = (e.clientX - dx) + 'px';
    popup.style.top  = (e.clientY - dy) + 'px';
  });
  document.addEventListener('mouseup', function() { dragging = false; });
}

// ── Tab-Wechsel im Popup ──────────────────────────────────────────────────
var _fmtHfTab = 'hf';
function fmtHfSwitchTab(tab) {
  _fmtHfTab = tab;
  ['hf','move','io','vars','desc'].forEach(function(t) {
    var btn = document.getElementById('fmt-tab-' + t);
    var pane = document.getElementById('fmt-pane-' + t);
    if (btn)  btn.style.borderBottom  = (t === tab) ? '2px solid var(--acc)' : '2px solid transparent';
    if (btn)  btn.style.color         = (t === tab) ? 'var(--acc)' : 'var(--txt2)';
    if (pane) pane.style.display      = (t === tab) ? 'block' : 'none';
  });
}

// ── Popup öffnen ──────────────────────────────────────────────────────────
function fmtHfOpenEditor() {
  var activeId = FormatRegistry.getActiveId ? FormatRegistry.getActiveId() : null;
  if (!activeId || activeId === 'kuka-form') {
    alert('Bitte zuerst ein Ausgabeformat wählen (nicht Formular).');
    return;
  }
  _fmtHfCurrentId = activeId;
  var fmt = FormatRegistry._allFormats().find(function(f) { return f.id === activeId; });
  var title = document.getElementById('fmt-hf-popup-title');
  if (title) title.textContent = (fmt ? fmt.label : activeId) + ' — Einstellungen';

  // Kopf / Fuß
  var hf = fmtHfLoad(activeId);
  var hEl = document.getElementById('fmt-hf-header');
  var fEl = document.getElementById('fmt-hf-footer');
  if (hEl) hEl.value = hf.header || '';
  if (fEl) fEl.value = hf.footer || '';

  // Befehlsvorlagen
  var cmds = fmtCmdLoad(activeId);
  var KEYS = ['moveJ','moveL','moveC','halt','dout','din','aout','ain','wait','tool','base','varInt','varReal','varBool','call'];
  KEYS.forEach(function(k) {
    var el = document.getElementById('fmt-cmd-' + k);
    if (el) el.value = (cmds[k] !== undefined) ? cmds[k] : ((FMT_CMD_DEFAULTS[activeId] || {})[k] || '');
  });

  // Ersten Tab aktivieren
  fmtHfSwitchTab('hf');
  // Beschreibung für dieses Format vorab rendern
  if (typeof fmtHfRenderDesc === 'function') fmtHfRenderDesc(activeId);

  // Popup positionieren
  var popup = document.getElementById('fmt-hf-popup');
  if (!popup) return;
  var btn = document.getElementById('fmt-edit-btn');
  if (btn) {
    var r = btn.getBoundingClientRect();
    popup.style.top  = Math.min(r.bottom + 6, window.innerHeight - 520) + 'px';
    popup.style.left = Math.max(8, r.right - 620) + 'px';
  }
  popup.style.display = 'block';
  _initFmtHfDrag();
}

// ── Speichern ─────────────────────────────────────────────────────────────
function fmtHfSave() {
  if (!_fmtHfCurrentId) return;
  // Kopf / Fuß
  var h = (document.getElementById('fmt-hf-header') || {}).value || '';
  var f = (document.getElementById('fmt-hf-footer') || {}).value || '';
  fmtHfSaveKey(_fmtHfCurrentId, h, f);
  // Befehlsvorlagen
  var cmds = {};
  var KEYS = ['moveJ','moveL','moveC','halt','dout','din','aout','ain','wait','tool','base','varInt','varReal','varBool','call'];
  KEYS.forEach(function(k) {
    var el = document.getElementById('fmt-cmd-' + k);
    if (el) cmds[k] = el.value;
  });
  fmtCmdSaveKey(_fmtHfCurrentId, cmds);
  document.getElementById('fmt-hf-popup').style.display = 'none';
  // Output neu generieren
  var fmt = FormatRegistry._allFormats && FormatRegistry._allFormats().find(function(fmt) { return fmt.id === _fmtHfCurrentId; });
  var ci  = document.getElementById('code-input');
  if (fmt && fmt._generate && ci && typeof parsedData !== 'undefined') {
    ci.value = fmt._generate(parsedData);
    if (typeof rebuildGutter === 'function') rebuildGutter();
  }
}

// ── Auf Standard zurücksetzen ─────────────────────────────────────────────
function fmtHfReset() {
  if (!_fmtHfCurrentId) return;
  fmtHfRemoveKey(_fmtHfCurrentId);
  fmtCmdRemoveKey(_fmtHfCurrentId);
  var def = FMT_HF_DEFAULTS[_fmtHfCurrentId] || { header: '', footer: '' };
  var hEl = document.getElementById('fmt-hf-header');
  var fEl = document.getElementById('fmt-hf-footer');
  if (hEl) hEl.value = def.header || '';
  if (fEl) fEl.value = def.footer || '';
  var KEYS = ['moveJ','moveL','moveC','halt','dout','din','aout','ain','wait','tool','base','varInt','varReal','varBool','call'];
  var defCmd = FMT_CMD_DEFAULTS[_fmtHfCurrentId] || {};
  KEYS.forEach(function(k) {
    var el = document.getElementById('fmt-cmd-' + k);
    if (el) el.value = defCmd[k] || '';
  });
}
