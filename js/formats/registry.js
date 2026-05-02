// ══════════════════════════════════════════════════════
// FORMAT REGISTRY  — Post-Prozessor Verwaltung
// ══════════════════════════════════════════════════════
var FormatRegistry = (function () {
  var _formats  = [];   // [{ id, label, activate, deactivate }]
  var _activeId = null;

  function _find(id) {
    for (var i = 0; i < _formats.length; i++)
      if (_formats[i].id === id) return _formats[i];
    return null;
  }

  return {
    register: function (fmt) {
      // Doppelte Registrierung verhindern
      if (!_find(fmt.id)) _formats.push(fmt);
    },

    setActive: function (id) {
      if (id === _activeId) return;
      var prev = _find(_activeId);
      if (prev && prev.deactivate) prev.deactivate();
      _activeId = id;
      var next = _find(id);
      if (next && next.activate) next.activate();
      // Dropdown synchronisieren
      var sel = document.getElementById('format-select');
      if (sel && sel.value !== id) sel.value = id;
      // Label aktualisieren
      var lbl = document.getElementById('editor-lang-label');
      if (lbl && next) lbl.textContent = next.label;
    },

    getActiveId: function () { return _activeId; },

    // Dropdown mit allen registrierten Formaten befüllen
    buildDropdown: function (selectEl) {
      selectEl.innerHTML = '';
      _formats.forEach(function (f) {
        var opt = document.createElement('option');
        opt.value = f.id;
        opt.textContent = f.label;
        if (f.id === _activeId) opt.selected = true;
        selectEl.appendChild(opt);
      });
    }
  };
})();
