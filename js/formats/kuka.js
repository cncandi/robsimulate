// ══════════════════════════════════════════════════════
// FORMAT: KUKA KRL
// ══════════════════════════════════════════════════════
FormatRegistry.register({
  id:    'kuka',
  label: 'KUKA KRL',

  activate: function () {
    document.getElementById('code-input').style.display        = '';
    document.getElementById('gutter').style.display            = '';
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'none';
  },

  deactivate: function () {}
});
