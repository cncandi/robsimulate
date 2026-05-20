// ══════════════════════════════════════════════════════
// FORMAT: ABB Binary  —  ABB 3DP Print_N ZIP
// ══════════════════════════════════════════════════════

FormatRegistry.register({
  id:    'abb-binary',
  label: 'ABB Binary',
  icon:  '<img src="logos/abb.png" style="height:16px;vertical-align:middle;margin-right:4px" onerror="this.style.display=\'none\'">',

  activate: function () {
    var ci = document.getElementById('code-input');
    if (ci) ci.style.display = 'none';
    var hint = document.getElementById('abb-binary-hint');
    if (!hint) {
      hint = document.createElement('div');
      hint.id = 'abb-binary-hint';
      hint.style.cssText = 'padding:18px 14px;color:var(--txt2);font-size:.85em;line-height:1.9';
      hint.innerHTML =
        '<b style="color:var(--acc)">ABB Binary</b> — Binäres ABB 3DP Druckpfad-Format<br>' +
        '<span style="color:var(--txt3)">Print_N Dateien in ZIP · 232 Bytes/Block · XYZ + Quaternion + Speed</span><br><br>' +
        '📂 <b>Laden:</b> ABB ZIP wählen → wird automatisch dekodiert und simuliert<br>' +
        '💾 <b>Speichern:</b> Aktuelle Positionen → Print_N Binär-ZIP';
      if (ci && ci.parentNode) ci.parentNode.insertBefore(hint, ci);
    }
    hint.style.display = 'block';
  },

  deactivate: function () {
    var ci = document.getElementById('code-input');
    if (ci) ci.style.display = '';
    var hint = document.getElementById('abb-binary-hint');
    if (hint) hint.style.display = 'none';
  }
});
