// ══════════════════════════════════════════════════════
// FORMAT REGISTRY  — Post-Prozessor Verwaltung
// ══════════════════════════════════════════════════════
var FormatRegistry = (function () {
  var _formats  = [];   // [{ id, label, icon, activate, deactivate }]
  var _activeId = null;

  function _find(id) {
    for (var i = 0; i < _formats.length; i++)
      if (_formats[i].id === id) return _formats[i];
    return null;
  }

  function _renderBtn() {
    var btn = document.getElementById('fmt-btn');
    if (!btn) return;
    var f = _find(_activeId);
    if (!f) return;
    btn.innerHTML = (f.icon || '') + '<span class="fmt-label">' + f.label + '</span>';
  }

  function _buildMenu() {
    var menu = document.getElementById('fmt-menu');
    if (!menu) return;
    menu.innerHTML = '';
    _formats.forEach(function(f) {
      var item = document.createElement('div');
      item.className = 'fmt-item' + (f.id === _activeId ? ' active' : '');
      item.innerHTML = (f.icon || '') + '<span>' + f.label + '</span>';
      item.onclick = function() {
        FormatRegistry.setActive(f.id);
        menu.style.display = 'none';
      };
      menu.appendChild(item);
    });
  }

  return {
    register: function (fmt) {
      if (!_find(fmt.id)) _formats.push(fmt);
    },

    setActive: function (id) {
      if (id === _activeId) return;
      var prev = _find(_activeId);

      // Aktuelles Format parsen → parsedData aktualisieren
      // Damit hat das nächste Format immer frische Daten zum Generieren
      var ci = document.getElementById('code-input');
      if (prev && prev._parse && ci && ci.value.trim()) {
        try {
          var parsed = prev._parse(ci.value);
          if (parsed && parsed.positions && parsed.positions.length) {
            if (typeof parsedData !== 'undefined') {
              parsedData.positions = parsed.positions;
              parsedData.steps     = parsed.steps || parsedData.steps || [];
            }
          }
        } catch (e) { /* Parse-Fehler ignorieren */ }
      }
      // Wenn Formular aktiv war → parsedData ist bereits aktuell (kein Parse nötig)

      if (prev && prev.deactivate) prev.deactivate();
      _activeId = id;
      var next = _find(id);
      if (next && next.activate) next.activate();
      _renderBtn();
      var lbl = document.getElementById('editor-lang-label');
      if (lbl && next) lbl.textContent = next.label;
    },

    getActiveId: function () { return _activeId; },

    // Dropdown aufbauen (Legacy-Kompatibilität, wird nicht mehr für <select> verwendet)
    buildDropdown: function (selectEl) {
      if (!selectEl) return;
      selectEl.innerHTML = '';
      _formats.forEach(function (f) {
        var opt = document.createElement('option');
        opt.value = f.id; opt.textContent = f.label;
        if (f.id === _activeId) opt.selected = true;
        selectEl.appendChild(opt);
      });
    },

    // Custom-Button Dropdown initialisieren
    initButton: function () {
      var btn  = document.getElementById('fmt-btn');
      var menu = document.getElementById('fmt-menu');
      if (!btn || !menu) return;
      _renderBtn();
      btn.onclick = function(e) {
        e.stopPropagation();
        _buildMenu();
        menu.style.display = menu.style.display === 'block' ? 'none' : 'block';
      };
      document.addEventListener('click', function() { menu.style.display = 'none'; });
    }
  };
})();
