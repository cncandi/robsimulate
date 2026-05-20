// ══════════════════════════════════════════════════════
// FORMAT: ABB Binary  —  ABB 3DP Print_N ZIP
// ══════════════════════════════════════════════════════

FormatRegistry.register({
  id:    'abb-binary',
  label: 'ABB Binary',
  icon:  '<img src="logos/abb.svg" style="height:16px;vertical-align:middle;margin-right:4px" onerror="this.style.display=\'none\'">',

  activate: function () {
    // Editor ausblenden, Hinweis einblenden
    var ci = document.getElementById('code-input');
    if (ci) ci.style.display = 'none';
    var hint = document.getElementById('abb-binary-hint');
    if (!hint) {
      hint = document.createElement('div');
      hint.id = 'abb-binary-hint';
      hint.style.cssText = 'padding:18px 14px;color:var(--txt2);font-size:.85em;line-height:1.7';
      hint.innerHTML =
        '<b style="color:var(--acc)">ABB Binary</b> — Binäres ABB 3DP Druckpfad-Format<br>' +
        '<span style="color:var(--txt3)">Print_N Dateien in ZIP (232 Bytes/Block · XYZ + Quaternion + Speed)</span><br><br>' +
        '📂 <b>Laden:</b> ABB ZIP wählen → wird automatisch dekodiert<br>' +
        '💾 <b>Speichern:</b> Aktuelle Positionen → Print_N ZIP';
      var editorWrap = ci ? ci.parentNode : null;
      if (editorWrap) editorWrap.insertBefore(hint, ci);
    }
    hint.style.display = 'block';

    // Programm-Load-Button auf ZIP umstellen
    var progIn = document.getElementById('prog-file-in');
    if (progIn) progIn.accept = '.zip';
  },

  deactivate: function () {
    var ci = document.getElementById('code-input');
    if (ci) ci.style.display = '';
    var hint = document.getElementById('abb-binary-hint');
    if (hint) hint.style.display = 'none';
    var progIn = document.getElementById('prog-file-in');
    if (progIn) progIn.accept = '.src,.krl,.txt,.zip';
  }
});

// ── saveProgram / loadProgram überschreiben wenn ABB Binary aktiv ──

(function () {
  var _origSave = null;
  var _origLoad = null;

  function _hookIfNeeded() {
    if (_origSave) return;
    _origSave = window.saveProgram;
    _origLoad = window.loadProgram;

    window.saveProgram = function () {
      if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'abb-binary') {
        if (typeof _3dp_saveZip === 'function') _3dp_saveZip();
      } else {
        _origSave();
      }
    };

    window.loadProgram = function () {
      if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'abb-binary') {
        var inp = document.getElementById('prog-file-in');
        inp.accept = '.zip';
        inp.onchange = function (e) {
          var file = e.target.files[0]; if (!file) return;
          if (typeof _3dp_loadZip === 'function') _3dp_loadZip(file);
          inp.value = '';
        };
        inp.click();
      } else {
        _origLoad();
      }
    };
  }

  // Hook sobald DOM bereit
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', _hookIfNeeded);
  } else {
    _hookIfNeeded();
  }
})();

// fmtSaveFile / fmtLoadFile ebenfalls für ABB Binary patchen
(function () {
  var _origFmtSave = null;
  var _origFmtLoad = null;

  function _hookFmt() {
    if (_origFmtSave) return;
    if (typeof fmtSaveFile === 'undefined') return;
    _origFmtSave = window.fmtSaveFile;
    _origFmtLoad = window.fmtLoadFile;

    window.fmtSaveFile = function () {
      if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'abb-binary') {
        if (typeof _3dp_saveZip === 'function') _3dp_saveZip();
      } else {
        _origFmtSave();
      }
    };

    window.fmtLoadFile = function () {
      if (typeof FormatRegistry !== 'undefined' && FormatRegistry.getActiveId() === 'abb-binary') {
        var inp = document.getElementById('fmt-load-input');
        if (!inp) return;
        inp.accept = '.zip';
        inp.onchange = function () {
          var file = inp.files[0]; if (!file) return;
          if (typeof _3dp_loadZip === 'function') _3dp_loadZip(file);
          inp.value = '';
        };
        inp.click();
      } else {
        _origFmtLoad();
      }
    };
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', _hookFmt);
  } else {
    setTimeout(_hookFmt, 500);
  }
})();
