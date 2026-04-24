'use strict';
// ═══════════════════════════════════════════════════════════════
// KRL Monaco Editor — vollständige Integration
// Ersetzt das textarea code-input durch Monaco
// ═══════════════════════════════════════════════════════════════

var _krlEditor = null;
var _monacoInstance = null;
var _monacoReady = false;
var _pendingValue = null;

// ── KRL Sprach-Definition (portiert aus krlLanguage.ts) ────────
function registerKRLLanguage(monaco) {
  monaco.languages.register({ id:'krl', extensions:['.src','.dat','.sub'], aliases:['KRL','krl'] });

  monaco.languages.setMonarchTokensProvider('krl', {
    ignoreCase: true,
    defaultToken: '',
    tokenPostfix: '.krl',
    keywords: [
      'DEF','END','DEFFCT','ENDFCT','DEFDAT','ENDDAT',
      'DECL','GLOBAL','PUBLIC','IMPORT','EXPORT',
      'IF','THEN','ELSE','ELSEIF','ENDIF',
      'FOR','TO','STEP','ENDFOR',
      'WHILE','ENDWHILE','REPEAT','UNTIL','LOOP','ENDLOOP',
      'SWITCH','CASE','DEFAULT','ENDSWITCH',
      'GOTO','HALT','RETURN','EXIT','RESUME','BRAKE','CONTINUE',
      'WAIT','TRIGGER','INTERRUPT','WHEN','DISTANCE','PATH','ONSTART',
      'FOLD','ENDFOLD','INI','BAS','TRUE','FALSE',
      'SIGNAL','CHANNEL','PORT','ANIN','ANOUT','DIGIN','PULSE','STROBE',
      'ON','OFF','NOT','AND','OR','EXOR','B_NOT','B_AND','B_OR','B_EXOR','ASYNC',
      'SEC','DELAY',
    ],
    motionCommands: ['PTP','LIN','CIRC','SPTP','SLIN','SCIRC','PTP_REL','LIN_REL','CIRC_REL'],
    types: ['INT','REAL','BOOL','CHAR','E6POS','E6AXIS','FRAME','POS','AXIS','STRUC','ENUM','LOAD','INERTIA'],
    systemVars: [
      '$VEL','$ACC','$APO','$ORI_TYPE','$CIRC_TYPE',
      '$VEL.CP','$VEL.ORI1','$VEL.ORI2','$ACC.CP','$ACC.ORI1','$ACC.ORI2',
      '$APO.CDIS','$APO.CVEL','$APO.CORI','$TOOL','$BASE','$NULLFRAME',
      '$VEL_AXIS','$ACC_AXIS','$POS_ACT','$AXIS_ACT',
      '$IN','$OUT','$ANIN','$ANOUT','$FLAG','$CYCFLAG','$TIMER','$ADVANCE',
      '$MOVE_ENABLE','$DRIVES_ON','$MODE_OP','$PRO_STATE','$STOPMESS',
    ],
    builtins: [
      'ABS','ACOS','ASIN','ATAN2','COS','SIN','TAN',
      'SQRT','EXP','LN','POT','MAX','MIN','MOD','DIV',
      'TRUNC','ROUND','STRLEN','SET_CHAR','GET_CHAR','CTOI','ITOC',
    ],
    operators: ['=','==','<>','<','>','<=','>=','+','-','*','/'],
    symbols: /[=<>:+\-*/,;.[\](){}]/,
    tokenizer: {
      root: [
        [/;FOLD\b.*$/, 'comment.fold'],
        [/;ENDFOLD\b.*$/, 'comment.fold'],
        [/;.*$/, 'comment'],
        [/&[A-Z_]+.*$/, 'comment.preprocessor'],
        [/\d+\.\d*([eE][+-]?\d+)?/, 'number.float'],
        [/\d+[eE][+-]?\d+/, 'number.float'],
        [/\d+/, 'number'],
        [/"[^"]*"/, 'string'],
        [/[A-Z_$][A-Z0-9_.]*/, {
          cases: {
            '@motionCommands': 'keyword.motion',
            '@keywords':       'keyword',
            '@types':          'type',
            '@systemVars':     'variable.system',
            '@builtins':       'keyword.function',
            '@default':        'identifier',
          }
        }],
        [/@symbols/, 'operator'],
        [/[ \t\r\n]+/, 'white'],
        [/{/, 'delimiter.curly'],
        [/}/, 'delimiter.curly'],
        [/\[/, 'delimiter.square'],
        [/\]/, 'delimiter.square'],
        [/\(/, 'delimiter.parenthesis'],
        [/\)/, 'delimiter.parenthesis'],
        [/#[A-Z_]+/, 'constant'],
      ]
    }
  });

  // ── KRL Dark Theme ────────────────────────────────────────────
  monaco.editor.defineTheme('krl-dark', {
    base: 'vs-dark',
    inherit: true,
    rules: [
      { token: 'keyword.motion',        foreground: 'C586C0', fontStyle: 'bold' },
      { token: 'keyword',               foreground: '569CD6', fontStyle: 'bold' },
      { token: 'type',                  foreground: '4EC9B0' },
      { token: 'variable.system',       foreground: '9CDCFE' },
      { token: 'keyword.function',      foreground: 'DCDCAA' },
      { token: 'number',                foreground: 'B5CEA8' },
      { token: 'number.float',          foreground: 'B5CEA8' },
      { token: 'string',                foreground: 'CE9178' },
      { token: 'comment',               foreground: '6A9955', fontStyle: 'italic' },
      { token: 'comment.fold',          foreground: 'D4A017', fontStyle: 'italic' },
      { token: 'comment.preprocessor',  foreground: 'A8A8A8', fontStyle: 'italic' },
      { token: 'constant',              foreground: '86C691' },
      { token: 'identifier',            foreground: 'D4D4D4' },
      { token: 'operator',              foreground: 'D4D4D4' },
      { token: 'delimiter.curly',       foreground: 'FFD700' },
    ],
    colors: {
      'editor.background':              '#0a1422',
      'editor.foreground':              '#D4D4D4',
      'editor.lineHighlightBackground': '#0d1f35',
      'editor.selectionBackground':     '#264F78',
      'editorLineNumber.foreground':    '#3a5a7a',
      'editorLineNumber.activeForeground': '#9ecfea',
      'editorCursor.foreground':        '#F05A28',
      'editor.inactiveSelectionBackground': '#3a3d41',
      'editorGutter.background':        '#0a1422',
      'editorIndentGuide.background':   '#1a2a3a',
    }
  });

  // ── Autocomplete ─────────────────────────────────────────────
  monaco.languages.registerCompletionItemProvider('krl', {
    provideCompletionItems: function(model, position) {
      var word = model.getWordUntilPosition(position);
      var range = {
        startLineNumber: position.lineNumber,
        endLineNumber:   position.lineNumber,
        startColumn:     word.startColumn,
        endColumn:       word.endColumn,
      };
      var suggestions = [
        // Motion snippets
        { label:'PTP', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'PTP {A1 0.0,A2 -90.0,A3 90.0,A4 0.0,A5 0.0,A6 0.0}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Point-to-Point Bewegung', range:range },
        { label:'LIN', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'LIN {X ${1:1200},Y ${2:0},Z ${3:800},A ${4:0},B ${5:90},C ${6:0},S 2,T 35}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Lineare Bewegung', range:range },
        { label:'CIRC', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'CIRC {X ${1:1200},Y ${2:200},Z ${3:800},A 0,B 90,C 0},{X ${4:1400},Y 0,Z ${3:800},A 0,B 90,C 0}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Kreisbewegung', range:range },
        // Control flow
        { label:'IF', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'IF ${1:Bedingung} THEN\n\t${0}\nENDIF',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Bedingte Ausführung', range:range },
        { label:'FOR', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'FOR ${1:i}=1 TO ${2:10}\n\t${0}\nENDFOR',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Zählschleife', range:range },
        { label:'WHILE', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'WHILE ${1:Bedingung}\n\t${0}\nENDWHILE',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Schleife', range:range },
        { label:'FOLD', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:';FOLD ${1:Beschreibung}\n\t${0}\n;ENDFOLD',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'FOLD Block', range:range },
        { label:'DEF', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'DEF ${1:ProgrammName}()\n\tINI\n\t${0}\nEND',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Programmdefinition', range:range },
        { label:'BAS', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'BAS(#INITMOV,0)',
          detail:'KUKA BAS Initialisierung', range:range },
        { label:'WAIT SEC', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'WAIT SEC ${1:0.5}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Warten in Sekunden', range:range },
        { label:'WAIT FOR', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'WAIT FOR ${1:$IN[1]==TRUE}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Warten auf Bedingung', range:range },
        { label:'$VEL.CP', kind:monaco.languages.CompletionItemKind.Property,
          insertText:'$VEL.CP=${1:0.5}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Bahngeschwindigkeit [m/s]', range:range },
        { label:'$OUT', kind:monaco.languages.CompletionItemKind.Property,
          insertText:'$OUT[${1:1}]=${2:TRUE}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Digital Output', range:range },
        { label:'$IN', kind:monaco.languages.CompletionItemKind.Property,
          insertText:'$IN[${1:1}]',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Digital Input', range:range },
        { label:'INTERRUPT', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'INTERRUPT DECL ${1:1} WHEN ${2:$STOPMESS==TRUE} DO ${3:IR_STOPM()}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Interrupt deklarieren', range:range },
        { label:'DECL E6POS', kind:monaco.languages.CompletionItemKind.Snippet,
          insertText:'DECL E6POS ${1:XP1}={X ${2:0},Y ${3:0},Z ${4:0},A ${5:0},B ${6:90},C ${7:0},S 0,T 0}',
          insertTextRules:monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet,
          detail:'Kartesische Position', range:range },
      ];
      return { suggestions: suggestions };
    }
  });

  // ── Hover Provider ────────────────────────────────────────────
  monaco.languages.registerHoverProvider('krl', {
    provideHover: function(model, position) {
      var word = model.getWordAtPosition(position);
      if (!word) return null;
      var hover = {
        'PTP':  '**PTP** — Point-to-Point Bewegung (achsinterpoliert)\n\nSchnellste Bewegungsart, Weg nicht vorhersagbar.\n\n`PTP {A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}`',
        'LIN':  '**LIN** — Lineare Bewegung\n\nTCP bewegt sich geradlinig.\n\n`$VEL.CP = 0.5  ; [m/s]`\n`LIN {X 1200, Y 0, Z 800, A 0, B 90, C 0, S 2, T 35}`',
        'CIRC': '**CIRC** — Kreisbewegung\n\n`CIRC {XHilfspunkt}, {XZielpunkt}`',
        'SPTP': '**SPTP** — Synchrone PTP (Spline-Gruppe)',
        'SLIN': '**SLIN** — Synchrone LIN (Spline-Gruppe)',
        'SCIRC':'**SCIRC** — Synchrone CIRC (Spline-Gruppe)',
        'DEF':  '**DEF** — Programmstart\n\n`DEF ProgrammName()`\n`  INI`\n`  ...`\n`END`',
        'DEFFCT':'**DEFFCT** — Funktion mit Rückgabewert\n\n`DEFFCT INT FuncName(INT Param)`\n`  ...`\n`  RETURN Wert`\n`ENDFCT`',
        'WAIT': '**WAIT** — Warten\n\n`WAIT SEC 0.5` — Sekunden\n`WAIT FOR $IN[1]==TRUE` — auf Bedingung',
        'BAS':  '**BAS** — KUKA Basisinitialisierung\n\n`BAS(#INITMOV, 0)` — Standard',
        'INI':  '**INI** — Initialisierungs-FOLD (implizit)',
        'INTERRUPT': '**INTERRUPT** — Interrupt-Behandlung\n\n`INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM()`\n`INTERRUPT ON 3`',
        'TRIGGER': '**TRIGGER** — Schaltpunkt relativ zur Bewegung\n\n`TRIGGER WHEN DISTANCE=0 DELAY=0 DO $OUT[1]=TRUE`',
        '$VEL': '**$VEL** — Geschwindigkeitsvorgaben\n\n`$VEL.CP` — Bahngeschwindigkeit [m/s]\n`$VEL.ORI1`, `$VEL.ORI2` — Orientierungsgeschwindigkeit [°/s]',
        '$ACC': '**$ACC** — Beschleunigungsvorgaben\n\n`$ACC.CP` — Bahnbeschleunigung [m/s²]',
        '$TOOL': '**$TOOL** — Aktuelles Werkzeugkoordinatensystem\n\n`$TOOL = TOOL_DATA[1]`',
        '$BASE': '**$BASE** — Aktuelles Basiskoordinatensystem\n\n`$BASE = BASE_DATA[1]`',
        '$OUT': '**$OUT[n]** — Digitaler Ausgang\n\n`$OUT[1] = TRUE  ; Ausgang 1 setzen`',
        '$IN':  '**$IN[n]** — Digitaler Eingang\n\n`WAIT FOR $IN[1]==TRUE`',
        '$ANOUT': '**$ANOUT[n]** — Analoger Ausgang [V]',
        '$ANIN':  '**$ANIN[n]** — Analoger Eingang [V]',
      };
      var key = word.word.toUpperCase();
      var content = hover[key];
      if (!content) return null;
      return {
        range: { startLineNumber: position.lineNumber, endLineNumber: position.lineNumber,
                 startColumn: word.startColumn, endColumn: word.endColumn },
        contents: [{ value: content }]
      };
    }
  });
}

// ── Monaco initialisieren ─────────────────────────────────────
function initMonacoEditor(containerId, initialValue, onChange, onCursorChange) {
  if (typeof require === 'undefined') {
    console.error('Monaco AMD loader nicht vorhanden');
    return;
  }

  require.config({
    paths: { vs: 'https://cdnjs.cloudflare.com/ajax/libs/monaco-editor/0.45.0/min/vs' }
  });

  require(['vs/editor/editor.main'], function(monaco) {
    _monacoInstance = monaco;
    registerKRLLanguage(monaco);

    var container = document.getElementById(containerId);
    if (!container) { console.error('Container nicht gefunden:', containerId); return; }

    _krlEditor = monaco.editor.create(container, {
      value:              initialValue || '',
      language:           'krl',
      theme:              'krl-dark',
      fontSize:           17,
      lineNumbers:        'on',
      glyphMargin:        true,
      folding:            true,
      wordWrap:           'off',
      automaticLayout:    true,
      minimap:            { enabled: false },
      scrollBeyondLastLine: false,
      renderLineHighlight: 'all',
      cursorStyle:        'line',
      tabSize:            2,
      insertSpaces:       true,
      fontFamily:         "'Consolas', 'Courier New', monospace",
      lineHeight:         24,
      padding:            { top: 4, bottom: 4 },
      scrollbar: {
        verticalScrollbarSize: 8,
        horizontalScrollbarSize: 8,
      },
      contextmenu:        true,
    });

    // Pending value anwenden
    if (_pendingValue !== null) {
      _krlEditor.setValue(_pendingValue);
      _pendingValue = null;
    }

    // Breakpoints: Gutter-Click
    _krlEditor.onMouseDown(function(e) {
      if (e.target.type === monaco.editor.MouseTargetType.GUTTER_GLYPH_MARGIN ||
          e.target.type === monaco.editor.MouseTargetType.GUTTER_LINE_NUMBERS) {
        var line = e.target.position.lineNumber;
        toggleMonacoBreakpoint(line);
      }
    });

    // Content-Änderung
    _krlEditor.onDidChangeModelContent(function() {
      if (onChange) onChange(_krlEditor.getValue());
    });

    // Cursor-Änderung
    _krlEditor.onDidChangeCursorPosition(function(e) {
      if (onCursorChange) onCursorChange(e.position.lineNumber, e.position.column);
    });

    _monacoReady = true;
    console.log('Monaco KRL Editor bereit');
    if (typeof onMonacoReady === 'function') onMonacoReady();
  });
}

// ── Breakpoints ───────────────────────────────────────────────
var _monacoBreakpoints = {}; // { lineNumber: decorationId }

function toggleMonacoBreakpoint(line) {
  if (!_krlEditor || !_monacoInstance) return;
  var monaco = _monacoInstance;
  if (_monacoBreakpoints[line]) {
    _krlEditor.deltaDecorations([_monacoBreakpoints[line]], []);
    delete _monacoBreakpoints[line];
  } else {
    var ids = _krlEditor.deltaDecorations([], [{
      range: new monaco.Range(line, 1, line, 1),
      options: {
        isWholeLine: true,
        className: 'krl-breakpoint-line',
        glyphMarginClassName: 'krl-breakpoint-glyph',
        stickiness: monaco.editor.TrackedRangeStickiness.NeverGrowsWhenTypingAtEdges,
      }
    }]);
    _monacoBreakpoints[line] = ids[0];
  }
}

function getMonacoBreakpointLines() {
  return Object.keys(_monacoBreakpoints).map(Number);
}

function hasMonacoBreakpoint(line) {
  return !!_monacoBreakpoints[line];
}

// ── Active line highlight ─────────────────────────────────────
var _activeLineDec = [];
function setMonacoActiveLine(line) {
  if (!_krlEditor || !_monacoInstance) return;
  var monaco = _monacoInstance;
  _activeLineDec = _krlEditor.deltaDecorations(_activeLineDec, line ? [{
    range: new monaco.Range(line, 1, line, 1),
    options: {
      isWholeLine: true,
      className: 'krl-active-line',
      glyphMarginClassName: 'krl-active-glyph',
    }
  }] : []);
  if (line) _krlEditor.revealLineInCenter(line);
}

// ── Public API ────────────────────────────────────────────────
function monacoGetValue()      { return _krlEditor ? _krlEditor.getValue() : ''; }
function monacoSetValue(v)     {
  if (_krlEditor) { _krlEditor.setValue(v); }
  else { _pendingValue = v; }
}
function monacoFontSize(sz)    {
  if (_krlEditor) _krlEditor.updateOptions({ fontSize: sz, lineHeight: Math.round(sz * 1.6) });
}
function monacoRevealLine(n)   { if (_krlEditor) _krlEditor.revealLineInCenter(n); }
function monacoLayout()        { if (_krlEditor) _krlEditor.layout(); }
