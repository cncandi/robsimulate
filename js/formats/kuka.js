// ══════════════════════════════════════════════════════
// FORMAT: KUKA KRL
// ══════════════════════════════════════════════════════
FormatRegistry.register({
  id:    'kuka',
  label: 'KUKA KRL',
  icon:  '<svg viewBox="0 0 52 16" width="40" height="13" style="vertical-align:middle;margin-right:5px">'
       + '<text x="1" y="13" font-family="Arial Black,Arial,sans-serif" font-weight="900" font-size="14"'
       + ' fill="#e8400c" letter-spacing="1">KUKA</text></svg>',

  activate: function () {
    document.getElementById('code-input').style.display = '';
    document.getElementById('gutter').style.display     = '';
    var fv = document.getElementById('krl-form-view');
    if (fv) fv.style.display = 'none';
  },

  deactivate: function () {}
});
