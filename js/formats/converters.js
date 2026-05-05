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
  // cfg: { header, footer, moveL, moveJ, moveC, halt, brake,
  //        dout, dinWait, aout, ainRead, waitSec, waitFor,
  //        tool, base, velLine, comment, varDecl, ptpSection }
  function generate(parsedData, cfg) {
    if (!parsedData) return '';
    var steps     = parsedData.steps     || [];
    var positions = parsedData.positions || [];

    // Collect tool/base/vars from steps
    var toolN = 1, baseN = 1, vars = [];
    steps.forEach(function (s) {
      if (s.type === 'tool') toolN = s.n;
      if (s.type === 'base') baseN = s.n;
      if (s.type === 'var')  vars.push(s);
    });

    var lines = [];
    var push  = function (l) { if (l != null) lines.push(l); };

    // Header
    (cfg.header ? cfg.header(toolN, baseN, vars) : []).forEach(push);

    var curVel = 0.167;
    steps.forEach(function (s) {
      switch (s.type) {
        case 'comment': if (cfg.comment) push(cfg.comment(s.text || '')); break;
        case 'velcp':
          curVel = s.v || 0.167;
          if (cfg.velLine) push(cfg.velLine(curVel));
          break;
        case 'tool': if (cfg.tool) push(cfg.tool(s.n)); break;
        case 'base': if (cfg.base) push(cfg.base(s.n)); break;
        case 'var':  if (cfg.varDecl) push(cfg.varDecl(s.varType, s.name, s.val)); break;
        case 'move': {
          var pos = positions[s.posIdx]; if (!pos) break;
          var p = fmtP(pos), idx = (s.posIdx || 0) + 1;
          if (s.moveType === 'LIN' || s.moveType === 'SLIN') push(cfg.moveL(p, curVel, idx, pos));
          else push(cfg.moveJ(p, curVel, idx, pos));
          break;
        }
        case 'circ': {
          if (cfg.moveC) {
            var pVia = positions[s.viaIdx], pTo = positions[s.posIdx];
            if (pVia && pTo) push(cfg.moveC(fmtP(pVia), fmtP(pTo), curVel, (s.posIdx || 0) + 1));
          }
          break;
        }
        case 'halt':   if (cfg.halt)    push(cfg.halt());       break;
        case 'brake':  if (cfg.brake)   push(cfg.brake());
                       else if (cfg.halt) push(cfg.halt());     break;
        case 'dout':   if (cfg.dout)    push(cfg.dout(s.n, s.v === 'TRUE' || s.v === '1' || s.v === 'ON')); break;
        case 'din':    if (cfg.dinWait) push(cfg.dinWait(s.n)); break;
        case 'aout':   if (cfg.aout)    push(cfg.aout(s.n, s.v));   break;
        case 'ain':    if (cfg.ainRead) push(cfg.ainRead(s.n));      break;
        case 'wait':   if (cfg.waitSec) push(cfg.waitSec(s.t));      break;
        case 'waitFor':
          if (cfg.waitFor) push(cfg.waitFor(s.cond || ''));
          else if (cfg.dinWait) push(cfg.dinWait(1));
          break;
      }
    });

    // Optional position table (for formats using point names)
    if (cfg.ptpSection && positions.length) {
      var ptpLines = cfg.ptpSection(positions);
      if (ptpLines && ptpLines.length) ptpLines.forEach(push);
    }

    (cfg.footer ? cfg.footer() : []).forEach(push);
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

  // ── Shared activate / deactivate for all code-view formats ──────────────
  function makeActivate(id) {
    return function () {
      var ci = document.getElementById('code-input');
      var gt = document.getElementById('gutter');
      var fv = document.getElementById('krl-form-view');
      if (ci) ci.style.display = '';
      if (gt) gt.style.display = '';
      if (fv) fv.style.display = 'none';
      var fmt = FormatRegistry._implById(id);
      if (fmt && fmt._generate && typeof parsedData !== 'undefined') {
        ci.value = fmt._generate(parsedData);
        if (typeof rebuildGutter === 'function') rebuildGutter();
      }
    };
  }

  // ══════════════════════════════════════════════════════════════════════════
  // FORMAT IMPLEMENTATIONS
  // ══════════════════════════════════════════════════════════════════════════

  var IMPLS = {};

  // ── ABB RAPID / IRC5 / OmniCore ─────────────────────────────────────────
  IMPLS.abb = {
    _generate: function (pd) {
      return generate(pd, {
        header: function (t, b, vars) {
          var h = ['MODULE Main_Prog'];
          vars.forEach(function (v) {
            var typ = (v.varType === 'BOOL') ? 'bool' : 'num';
            var val = v.val || (v.varType === 'BOOL' ? 'FALSE' : v.varType === 'REAL' ? '0.0' : '0');
            h.push('VAR ' + typ + ' ' + v.name + ' := ' + val + ';');
          });
          h.push('PROC main()');
          return h;
        },
        footer: function () { return ['ENDPROC', 'ENDMODULE']; },
        moveL:   function (p, vel) { return '  MoveL [[' + p.X + ',' + p.Y + ',' + p.Z + '],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v' + velMmS(vel) + ', fine, tool1\\WObj:=wobj1;'; },
        moveJ:   function (p, vel) { return '  MoveJ [[' + p.X + ',' + p.Y + ',' + p.Z + '],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v' + velMmS(vel) + ', z10, tool1\\WObj:=wobj1;'; },
        moveC:   function (pv, pt, vel) { return '  MoveC [[' + pv.X + ',' + pv.Y + ',' + pv.Z + '],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],\n        [[' + pt.X + ',' + pt.Y + ',' + pt.Z + '],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v' + velMmS(vel) + ', fine, tool1\\WObj:=wobj1;'; },
        halt:    function ()        { return '  Stop;'; },
        dout:    function (n, v)    { return '  SetDO do' + n + ', ' + (v ? '1' : '0') + ';'; },
        dinWait: function (n)       { return '  WaitDI di' + n + ', 1;'; },
        aout:    function (n, v)    { return '  SetAO ao' + n + ', ' + parseFloat(v).toFixed(2) + ';'; },
        ainRead: function (n)       { return '  r := AInput(ai' + n + ');'; },
        waitSec: function (t)       { return '  WaitTime ' + parseFloat(t).toFixed(1) + ';'; },
        tool:    function (n)       { return '  ! Tool: tool' + n; },
        base:    function (n)       { return '  ! WObj: wobj' + n; },
        comment: function (t)       { return '  ! ' + t; },
        varDecl: function (tp, nm, v) { var t2 = tp === 'BOOL' ? 'bool' : 'num'; return 'VAR ' + t2 + ' ' + nm + ' := ' + (v || '0') + ';'; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MoveL\s+\[\[(?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: false },
        { rx: /MoveJ\s+\[\[(?<X>[\d.\-+]+),(?<Y>[\d.\-+]+),(?<Z>[\d.\-+]+)/, ptp: true  },
      ]);
    }
  };

  // ── FANUC TP / KAREL ────────────────────────────────────────────────────
  IMPLS.fanuc = {
    _generate: function (pd) {
      var ln = 1;
      return generate(pd, {
        header: function (t, b, vars) {
          var h = ['/PROG MAIN_PROG', '/MN'];
          if (t) h.push(' ' + (ln++) + ': UTOOL_NUM=' + t + ' ;');
          if (b) h.push(' ' + (ln++) + ': UFRAME_NUM=' + b + ' ;');
          vars.forEach(function (v) {
            if (v.varType === 'BOOL') h.push(' ' + (ln++) + ': F[1]=(OFF) ;');
            else h.push(' ' + (ln++) + ': R[' + (ln - 1) + ']=' + (v.val || '0') + ' ;');
          });
          return h;
        },
        footer: function () { return ['/END']; },
        moveL:   function (p, vel, idx) { return ' ' + (ln++) + ':L P[' + idx + '] ' + velMmS(vel) + 'mm/sec FINE ;'; },
        moveJ:   function (p, vel, idx) { return ' ' + (ln++) + ':J P[' + idx + '] ' + velPct(vel) + '% FINE ;'; },
        moveC:   function (pv, pt, vel, idx) { return ' ' + (ln++) + ':C P[' + idx + ']\n   P[' + (idx + 1) + '] ' + velMmS(vel) + 'mm/sec FINE ;'; },
        halt:    function ()        { return ' ' + (ln++) + ': PAUSE ;'; },
        dout:    function (n, v)    { return ' ' + (ln++) + ': DO[' + n + ']=' + (v ? 'ON' : 'OFF') + ' ;'; },
        dinWait: function (n)       { return ' ' + (ln++) + ': WAIT DI[' + n + ']=ON ;'; },
        aout:    function (n, v)    { return ' ' + (ln++) + ': AO[' + n + ']=' + parseFloat(v).toFixed(2) + ' ;'; },
        ainRead: function (n)       { return ' ' + (ln++) + ': R[1]=AI[' + n + '] ;'; },
        waitSec: function (t)       { return ' ' + (ln++) + ': WAIT ' + parseFloat(t).toFixed(1) + ' ;'; },
        tool:    function (n)       { return ' ' + (ln++) + ': UTOOL_NUM=' + n + ' ;'; },
        base:    function (n)       { return ' ' + (ln++) + ': UFRAME_NUM=' + n + ' ;'; },
        comment: function (t)       { return ' ' + (ln++) + ': ! ' + t + ' ;'; },
        ptpSection: function (positions) {
          var h = ['/POS'];
          positions.forEach(function (pos, i) {
            h.push('P[' + (i + 1) + ']{');
            h.push('   GP1:');
            h.push('    UF : 1, UT : 1,        CONFIG : \'N U T, 0, 0, 0\',');
            h.push('    X =' + parseFloat(pos.X).toFixed(3) + '  mm, Y =' + parseFloat(pos.Y).toFixed(3) + '  mm, Z =' + parseFloat(pos.Z).toFixed(3) + '  mm,');
            h.push('    W =' + parseFloat(pos.A).toFixed(3) + '  deg, P =' + parseFloat(pos.B).toFixed(3) + '  deg, R =' + parseFloat(pos.C).toFixed(3) + '  deg');
            h.push('};');
          });
          return h;
        },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /:L\s+P\[/, ptp: false },
        { rx: /:J\s+P\[/, ptp: true  },
      ]);
    }
  };

  // ── Yaskawa INFORM / DX,YRC ─────────────────────────────────────────────
  IMPLS.yaskawa = {
    _generate: function (pd) {
      return generate(pd, {
        header: function (t, b, vars) {
          var h = ['/JOB', '//NAME MAIN_PROG', 'NOP'];
          vars.forEach(function (v) {
            var pfx = v.varType === 'INT' ? 'I' : v.varType === 'REAL' ? 'R' : 'B';
            h.push('SET ' + pfx + '000 ' + (v.val || '0'));
          });
          if (t) h.push('TOOL ' + t);
          return h;
        },
        footer: function () { return ['END']; },
        moveL:   function (p, vel) { return 'MOVL C00000 V=' + velMmS(vel) + '.0'; },
        moveJ:   function (p, vel) { return 'MOVJ C00000 VJ=' + velPct(vel) + '.00'; },
        moveC:   function (pv, pt, vel) { return 'MOVC C00000 V=' + velMmS(vel) + '.0'; },
        halt:    function ()     { return 'PAUSE'; },
        dout:    function (n, v) { return 'DOUT OT#(' + n + ') ' + (v ? 'ON' : 'OFF'); },
        dinWait: function (n)    { return 'WAIT IN#(' + n + ')=ON'; },
        aout:    function (n, v) { return 'AOUT AO#(' + n + ') ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return 'AIN AI#(' + n + ') R000'; },
        waitSec: function (t)    { return 'TIMER T=' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return 'TOOL ' + n; },
        comment: function (t)    { return "'" + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVL\s+C\d+/, ptp: false },
        { rx: /MOVJ\s+C\d+/, ptp: true  },
        { rx: /MOVC\s+C\d+/, ptp: false },
      ]);
    }
  };

  // ── Kawasaki AS Language ─────────────────────────────────────────────────
  IMPLS.kawasaki = {
    _generate: function (pd) {
      return generate(pd, {
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
  IMPLS.comau = {
    _generate: function (pd) {
      return generate(pd, {
        header: function (t, b, vars) {
          var h = ['PROGRAM main'];
          if (vars.length) {
            h.push('VAR');
            vars.forEach(function (v) {
              h.push('  ' + v.name + ' : ' + (v.varType === 'INT' ? 'INTEGER' : v.varType === 'REAL' ? 'REAL' : 'BOOLEAN'));
            });
          }
          h.push('BEGIN');
          if (t) h.push('  $TOOL := tool' + t);
          if (b) h.push('  $BASE := base' + b);
          return h;
        },
        footer: function () { return ['END main']; },
        moveL:   function (p, vel, idx) { return '  MOVE LINEAR TO p' + idx; },
        moveJ:   function (p, vel, idx) { return '  MOVE JOINT TO p' + idx; },
        moveC:   function (pv, pt, vel, idx) { return '  MOVE CIRCULAR TO p' + idx + ' VIA p' + (idx - 1); },
        halt:    function ()     { return '  PAUSE'; },
        dout:    function (n, v) { return '  $DOUT[' + n + '] := ' + (v ? 'TRUE' : 'FALSE'); },
        dinWait: function (n)    { return '  WAIT FOR $DIN[' + n + '] = TRUE'; },
        aout:    function (n, v) { return '  $AOUT[' + n + '] := ' + parseFloat(v).toFixed(2); },
        ainRead: function (n)    { return '  r := $AIN[' + n + ']'; },
        waitSec: function (t)    { return '  DELAY ' + parseFloat(t).toFixed(1); },
        tool:    function (n)    { return '  $TOOL := tool' + n; },
        base:    function (n)    { return '  $BASE := base' + n; },
        comment: function (t)    { return '  -- ' + t; },
      });
    },
    _parse: function (text) {
      return parsePositions(text, [
        { rx: /MOVE LINEAR TO/, ptp: false },
        { rx: /MOVE JOINT TO/,  ptp: true  },
      ]);
    }
  };

  // ── AUBO Script / Teach Pendant ─────────────────────────────────────────
  IMPLS.aubo = {
    _generate: function (pd) {
      return generate(pd, {
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

  // Lookup für activate()
  FormatRegistry._implById = function (id) { return IMPLS[id]; };

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
      activate:   makeActivate(meta.id),
      deactivate: function () {},
      _generate:  impl._generate,
      _parse:     impl._parse,
    });
  });

  // KUKA logo auf PNG umstellen (Icon im bestehenden Eintrag überschreiben)
  // wird nach DOMContentLoaded via registry.setIcon aufgerufen falls nötig

})();
