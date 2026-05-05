// ══════════════════════════════════════════════════════
// FORMAT: KUKA KRL
// ══════════════════════════════════════════════════════
FormatRegistry.register({
  id:    'kuka',
  label: 'KUKA KRL',
  icon: '<img src="logos/kuka.png" height="14" style="vertical-align:middle;margin-right:6px;max-width:56px;object-fit:contain">',

  activate: function () {
    document.getElementById('code-input').style.display = '';
    document.getElementById('gutter').style.display     = '';
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'none';
  },

  deactivate: function () {}
});
