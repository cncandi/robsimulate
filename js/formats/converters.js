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
  function generate(parsedData, cfg) {
    if (!parsedData) return '';
    var positions = parsedData.positions || [];

    // Tool/Base aus KRL-Code lesen
    var toolN = 1, baseN = 1, vars = [];
    var code = (typeof document !== 'undefined' && document.getElementById('code-input'))
      ? document.getElementById('code-input').value : '';
    code.split(/\r?\n/).forEach(function (line) {
      var t = line.trim();
      var tm = t.match(/^\$TOOL\s*=\s*TOOL_DATA\[(\d+)\]/i);
      if (tm) toolN = parseInt(tm[1]);
      var bm = t.match(/^\$BASE\s*=\s*BASE_DATA\[(\d+)\]/i);
      if (bm) baseN = parseInt(bm[1]);
      var vm = t.match(/^DECL\s+(INT|REAL|BOOL)\s+(\w+)(?:\s*=\s*(.+))?/i);
      if (vm) vars.push({ varType: vm[1].toUpperCase(), name: vm[2], val: (vm[3] || '').trim() });
    });

    var lines = [];
    var push = function (l) { if (l != null && l !== '') lines.push(l); };

    // Programmkopf: Settings (localStorage) überschreiben Default
    var hfId = cfg._formatId;
    var hfStored = (hfId && typeof fmtHfLoad === 'function') ? fmtHfLoad(hfId) : null;
    var useStoredHeader = hfStored && (hfStored.header !== (FMT_HF_DEFAULTS[hfId] || {}).header || !FMT_HF_DEFAULTS[hfId]);
    if (useStoredHeader) {
      hfStored.header.split('\n').forEach(push);
    } else if (cfg.header) {
      cfg.header(toolN, baseN, vars).forEach(push);
    }

    positions.forEach(function (pos, i) {
      var vel  = pos.velCP || 0.167;
      var idx  = i + 1;
      var p    = fmtP(pos);
      var typ  = (pos.type || 'LIN').toUpperCase();

      if (typ === 'CIRC_AUX') return; // Wird zusammen mit CIRC ausgegeben

      if (typ === 'CIRC') {
        var via = (i > 0 && positions[i - 1].type === 'CIRC_AUX') ? positions[i - 1] : null;
        if (via && cfg.moveC) push(cfg.moveC(fmtP(via), p, vel, idx));
        else if (cfg.moveL)   push(cfg.moveL(p, vel, idx, pos));
      } else if (typ === 'LIN' || typ === 'SLIN') {
        if (cfg.moveL) push(cfg.moveL(p, vel, idx, pos));
      } else {
        if (cfg.moveJ) push(cfg.moveJ(p, vel, idx, pos));
      }
    });

    if (cfg.ptpSection && positions.length) {
      var ptpLines = cfg.ptpSection(positions);
      if (ptpLines && ptpLines.length) ptpLines.forEach(push);
    }

    // Programmfuß: Settings (localStorage) überschreiben Default
    var useStoredFooter = hfStored && (hfStored.footer !== (FMT_HF_DEFAULTS[hfId] || {}).footer || !FMT_HF_DEFAULTS[hfId]);
    if (useStoredFooter) {
      hfStored.footer.split('\n').forEach(push);
    } else if (cfg.footer) {
      cfg.footer().forEach(push);
    }
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

})();

// ══════════════════════════════════════════════════════════════════════════
// PROGRAMMKOPF / PROGRAMMFUSS — Settings UI + localStorage
// ══════════════════════════════════════════════════════════════════════════

var FMT_HF_DEFAULTS = {
  kuka:     { header: 'DEF Main()\n; Automatisch generiert von RobSimul', footer: 'END' },
  abb:      { header: 'MODULE Main_Prog\nPROC main()', footer: 'ENDPROC\nENDMODULE' },
  fanuc:    { header: '/PROG MAIN_PROG\n/MN', footer: '/END' },
  yaskawa:  { header: '/JOB\n//NAME MAIN_PROG\nNOP', footer: 'END' },
  kawasaki: { header: '.PROGRAM main()', footer: '.END' },
  staubli:  { header: 'begin', footer: 'end' },
  ur:       { header: 'def main():', footer: 'end' },
  adept:    { header: '.PROGRAM main()', footer: '.END' },
  omron:    { header: '.PROGRAM main()', footer: '.END' },
  epson:    { header: 'Function Main', footer: 'Fend' },
  comau:    { header: 'PROGRAM main\nBEGIN', footer: 'END main' },
  aubo:     { header: '-- AUBO Script', footer: 'robot_slow_stop()' },
  dobot:    { header: '# Dobot CR Script', footer: 'PauseScript()' },
  denso:    { header: 'PROGRAM mini_denso', footer: 'END' },
  nachi:    { header: 'PROGRAM MAIN_NACHI', footer: 'END' },
  hanwha:   { header: '// Hanwha HCR', footer: 'Stop()' },
  igus:     { header: '# igus iRC/iJC', footer: 'StopProgram()' },
  estun:    { header: 'PROGRAM MAIN_ESTUN', footer: 'END' },
  neura:    { header: '// NEURA SDK', footer: 'robot.stop();' },
  mabi:     { header: '// MABI Robot', footer: 'Stop()' },
};

var _fmtHfCurrentId = null;

function fmtHfLoad(id) {
  var key = 'robsimul_hf_' + id;
  try {
    var stored = localStorage.getItem(key);
    if (stored) return JSON.parse(stored);
  } catch(e) {}
  return FMT_HF_DEFAULTS[id] || { header: '', footer: '' };
}

function fmtHfSaveKey(id, header, footer) {
  try {
    localStorage.setItem('robsimul_hf_' + id, JSON.stringify({ header: header, footer: footer }));
  } catch(e) {}
}

function fmtHfRemoveKey(id) {
  try { localStorage.removeItem('robsimul_hf_' + id); } catch(e) {}
}

// Drag-Logik wird beim ersten Öffnen initialisiert
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

// Popup für aktives Format öffnen
function fmtHfOpenEditor() {
  var activeId = FormatRegistry.getActiveId ? FormatRegistry.getActiveId() : null;
  if (!activeId || activeId === 'kuka-form') {
    alert('Bitte zuerst ein Ausgabeformat wählen (nicht Formular).');
    return;
  }
  _fmtHfCurrentId = activeId;
  var fmt = FormatRegistry._allFormats().find(function(f){ return f.id === activeId; });
  var title = document.getElementById('fmt-hf-popup-title');
  if (title) title.textContent = (fmt ? fmt.label : activeId) + ' — Kopf / Fuß';
  var hf = fmtHfLoad(activeId);
  var hEl = document.getElementById('fmt-hf-header');
  var fEl = document.getElementById('fmt-hf-footer');
  if (hEl) hEl.value = hf.header;
  if (fEl) fEl.value = hf.footer;
  // Popup neben ✎ Button positionieren
  var popup = document.getElementById('fmt-hf-popup');
  if (!popup) return;
  var btn = document.getElementById('fmt-edit-btn');
  if (btn) {
    var r = btn.getBoundingClientRect();
    popup.style.top  = (r.bottom + 6) + 'px';
    popup.style.left = Math.max(8, r.right - 340) + 'px';
  }
  popup.style.display = 'block';
  _initFmtHfDrag();
}

// Speichern + Code-Output sofort neu generieren
function fmtHfSave() {
  if (!_fmtHfCurrentId) return;
  var h = (document.getElementById('fmt-hf-header') || {}).value || '';
  var f = (document.getElementById('fmt-hf-footer') || {}).value || '';
  fmtHfSaveKey(_fmtHfCurrentId, h, f);
  document.getElementById('fmt-hf-popup').style.display = 'none';
  // Code-Output neu generieren (aktives Format)
  var fmt = FormatRegistry._allFormats && FormatRegistry._allFormats().find(function(f){ return f.id === _fmtHfCurrentId; });
  var ci  = document.getElementById('code-input');
  if (fmt && fmt._generate && ci && typeof parsedData !== 'undefined') {
    ci.value = fmt._generate(parsedData);
    if (typeof rebuildGutter === 'function') rebuildGutter();
  }
}

// Auf Standard zurücksetzen
function fmtHfReset() {
  if (!_fmtHfCurrentId) return;
  fmtHfRemoveKey(_fmtHfCurrentId);
  var def = FMT_HF_DEFAULTS[_fmtHfCurrentId] || { header: '', footer: '' };
  var hEl = document.getElementById('fmt-hf-header');
  var fEl = document.getElementById('fmt-hf-footer');
  if (hEl) hEl.value = def.header;
  if (fEl) fEl.value = def.footer;
}
