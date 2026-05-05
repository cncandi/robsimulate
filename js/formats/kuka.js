// ══════════════════════════════════════════════════════
// FORMAT: KUKA KRL  —  Ausgabe aus parsedData
// ══════════════════════════════════════════════════════
FormatRegistry.register({
  id:    'kuka',
  label: 'KUKA KRL',
  icon:  '<img src="logos/kuka.png" height="14" style="vertical-align:middle;margin-right:6px;max-width:56px;object-fit:contain">',

  activate: function () {
    var ci = document.getElementById('code-input');
    var gt = document.getElementById('gutter');
    var fv = document.getElementById('krl-form-view');
    var fw = document.getElementById('fv-form-wrap');
    if (ci) ci.style.display = '';
    if (gt) gt.style.display = '';
    if (fv) fv.style.display = 'none';
    if (fw) fw.style.display = 'none';
    // Code aus parsedData neu generieren (Formular ist Quelle)
    if (typeof generateKRL === 'function' && typeof parsedData !== 'undefined') {
      ci.value = generateKRL(parsedData);
      if (typeof rebuildGutter === 'function') rebuildGutter();
    }
  },

  deactivate: function () {}
});
