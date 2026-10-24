<?php
// Kundenportal: Konten, Anmeldung (E-Mail-Link, Passkey), Testphase/Freischaltung je App, Freigabe fuer geschuetzte Apps.
require_once __DIR__ . '/lib.php';
require_once __DIR__ . '/webauthn.php';

header('Cache-Control: no-store');
header('X-Content-Type-Options: nosniff');
ensure_dirs();

const COOKIE = 'cnc_s';
const WA_COOKIE = 'cnc_wa';

// ─── Hilfen ─────────────────────────────────────────────────────────────────
function base_path() { return rtrim(str_replace('\\', '/', dirname($_SERVER['SCRIPT_NAME'] ?? '/')), '/') . '/'; }
function rp_id() { return preg_replace('/^www\./', '', preg_replace('/:\d+$/', '', $_SERVER['HTTP_HOST'] ?? 'localhost')); }
function origin() { return (is_https() ? 'https' : 'http') . '://' . ($_SERVER['HTTP_HOST'] ?? 'localhost'); }
function set_cookie($name, $val, $exp) {
  setcookie($name, $val, ['expires' => $exp, 'path' => base_path(), 'secure' => is_https(), 'httponly' => true, 'samesite' => 'Lax']);
}
function norm_email($e) { $e = mb_strtolower(trim((string)$e)); return filter_var($e, FILTER_VALIDATE_EMAIL) ? $e : null; }
function lang_of($b) { return ($b['lang'] ?? '') === 'en' ? 'en' : 'de'; }
function today() { return gmdate('Y-m-d'); }
function redirect($url) { header('Location: ' . $url, true, 302); exit; }
function safe_next($n) { return (is_string($n) && preg_match('/^(portal\.php\?fn=sso&[A-Za-z0-9_.~%&=:\/?-]*|#[A-Za-z0-9_=&%.-]*)$/', $n)) ? $n : '#account'; }

function customers() { return load('customers', []); }
function find_customer($pred) { foreach (customers() as $c) if ($pred($c)) return $c; return null; }
function update_customer($id, $fn) {
  return with_lock(function () use ($id, $fn) {
    $all = load('customers', []);
    foreach ($all as $i => $c) if ($c['id'] === $id) { $all[$i] = $fn($c); save('customers', $all); return $all[$i]; }
    return null;
  });
}
function new_customer($email, $name = '', $company = '') {
  return ['id' => uuid4(), 'email' => $email, 'name' => mb_substr($name, 0, 120), 'company' => mb_substr($company, 0, 120),
          'created_at' => gmdate('c'), 'last_login' => null, 'blocked' => false, 'note' => '', 'passkeys' => [], 'access' => []];
}
function apps_by_id() { $r = []; foreach (load('apps', []) as $a) $r[$a['id']] = $a; return $r; }

// Zugriffsstatus eines Kunden fuer eine App
function access_state($c, $app) {
  if (empty($app['protected'])) return ['state' => 'free', 'allowed' => true];
  $acc = $c['access'][$app['id']] ?? [];
  if (!empty($c['blocked']) || !empty($acc['blocked'])) return ['state' => 'blocked', 'allowed' => false];
  $paid = max((string)($acc['paid_until'] ?? ''), (string)($acc['manual_until'] ?? ''));
  if ($paid !== '' && $paid >= today()) return ['state' => 'active', 'allowed' => true, 'until' => $paid];
  if (empty($acc['trial_start'])) return ['state' => 'trial_available', 'allowed' => true, 'days_left' => TRIAL_DAYS];
  $end = strtotime($acc['trial_start']) + TRIAL_DAYS * 86400;
  if (time() < $end) return ['state' => 'trial', 'allowed' => true, 'until' => gmdate('c', $end), 'days_left' => (int)ceil(($end - time()) / 86400)];
  return ['state' => 'expired', 'allowed' => false, 'until' => gmdate('c', $end), 'expired_paid' => $paid ?: null];
}
function access_list($c) {
  $r = [];
  foreach (apps_by_id() as $id => $a) if (!empty($a['protected']) && !empty($a['visible'])) $r[$id] = access_state($c, $a);
  return $r;
}

// ─── Sitzung ────────────────────────────────────────────────────────────────
function session_customer() {
  $t = $_COOKIE[COOKIE] ?? '';
  if (!is_string($t) || strlen($t) < 20) return null;
  $s = load('sessions', [])[hash('sha256', $t)] ?? null;
  if (!$s || $s['e'] < time()) return null;
  $c = find_customer(fn($x) => $x['id'] === $s['c']);
  return ($c && empty($c['blocked'])) ? $c : null;
}
function start_session($cid) {
  $t = b64u_enc(random_bytes(32)); $exp = time() + SESSION_DAYS * 86400;
  with_lock(function () use ($t, $cid, $exp) {
    $all = array_filter(load('sessions', []), fn($s) => $s['e'] > time());
    $all[hash('sha256', $t)] = ['c' => $cid, 'e' => $exp];
    save('sessions', $all);
  });
  update_customer($cid, function ($c) { $c['last_login'] = gmdate('c'); return $c; });
  set_cookie(COOKIE, $t, $exp);
}
function end_session() {
  $t = $_COOKIE[COOKIE] ?? '';
  if (is_string($t) && $t !== '') with_lock(function () use ($t) { $all = load('sessions', []); unset($all[hash('sha256', $t)]); save('sessions', $all); });
  set_cookie(COOKIE, '', time() - 3600);
}

// Challenge fuer WebAuthn, an ein Kurzzeit-Cookie gebunden
function wa_new_challenge($cid = null) {
  $k = b64u_enc(random_bytes(18)); $ch = random_bytes(32);
  with_lock(function () use ($k, $ch, $cid) {
    $all = array_filter(load('challenges', []), fn($x) => $x['e'] > time());
    $all[$k] = ['ch' => b64u_enc($ch), 'c' => $cid, 'e' => time() + 300];
    save('challenges', $all);
  });
  set_cookie(WA_COOKIE, $k, time() + 300);
  return $ch;
}
function wa_take_challenge() {
  $k = $_COOKIE[WA_COOKIE] ?? '';
  return with_lock(function () use ($k) {
    $all = load('challenges', []);
    $x = $all[$k] ?? null; unset($all[$k]); save('challenges', $all);
    return ($x && $x['e'] > time()) ? $x : null;
  });
}

// ─── E-Mail-Link ────────────────────────────────────────────────────────────
function send_login_link($c, $lang, $next, $invite = false) {
  $t = b64u_enc(random_bytes(32));
  with_lock(function () use ($t, $c, $next) {
    $all = array_filter(load('links', []), fn($x) => $x['e'] > time());
    $all[hash('sha256', $t)] = ['c' => $c['id'], 'e' => time() + ($next === '__invite' ? 7 * 86400 : LINK_MINUTES * 60), 'n' => $next];
    save('links', $all);
  });
  $url = base_url() . '/portal.php?fn=link&t=' . $t;
  if ($lang === 'en') {
    $subj = $invite ? 'Your access to cnc-technik.de Apps' : 'Your sign-in link for cnc-technik.de Apps';
    $txt = ($invite ? "An account has been created for you at cnc-technik.de Apps.\n\n" : "") . "Click the link below to sign in:\n\n$url\n\nThe link is valid " . ($invite ? "for 7 days" : "for " . LINK_MINUTES . " minutes") . " and can only be used once.\nIf you did not request this, you can ignore this email.\n\nCAD/CAM Systeme Datentechnik Reitz GmbH & Co. KG\nhttps://cnc-technik.de";
  } else {
    $subj = $invite ? 'Ihr Zugang zu cnc-technik.de Apps' : 'Ihr Anmelde-Link für cnc-technik.de Apps';
    $txt = ($invite ? "für Sie wurde ein Konto bei cnc-technik.de Apps angelegt.\n\n" : "") . "Mit diesem Link melden Sie sich an:\n\n$url\n\nDer Link ist " . ($invite ? "7 Tage" : LINK_MINUTES . " Minuten") . " gültig und nur einmal verwendbar.\nFalls Sie keine Anmeldung angefordert haben, ignorieren Sie diese E-Mail.\n\nCAD/CAM Systeme Datentechnik Reitz GmbH & Co. KG\nhttps://cnc-technik.de";
    $txt = "Guten Tag,\n\n" . $txt;
  }
  return send_mail($c['email'], $subj, $txt);
}

function public_customer($c) {
  return ['email' => $c['email'], 'name' => $c['name'], 'company' => $c['company'],
          'passkeys' => array_map(fn($p) => ['id' => $p['id'], 'name' => $p['name'], 'created' => $p['created'], 'last_used' => $p['last_used'] ?? null], $c['passkeys'])];
}

// ─── GET-Aufrufe (Links, Weiterleitungen) ───────────────────────────────────
$fn = $_GET['fn'] ?? '';

if ($fn === 'link') {
  $t = $_GET['t'] ?? '';
  $x = with_lock(function () use ($t) {
    $all = load('links', []); $h = hash('sha256', (string)$t);
    $x = $all[$h] ?? null; unset($all[$h]); save('links', $all);
    return ($x && $x['e'] > time()) ? $x : null;
  });
  if (!$x) redirect(base_url() . '/#login=expired');
  $c = find_customer(fn($c) => $c['id'] === $x['c']);
  if (!$c || !empty($c['blocked'])) redirect(base_url() . '/#login=blocked');
  start_session($c['id']);
  $next = ($x['n'] ?? '') === '__invite' ? '#account=welcome' : safe_next($x['n'] ?? '');
  redirect(base_url() . '/' . $next);
}

if ($fn === 'sso') {
  // Aufruf durch das Pruefskript einer geschuetzten App: ?fn=sso&app=<id>&return=<URL in der App>
  $apps = apps_by_id();
  $app = $apps[$_GET['app'] ?? ''] ?? null;
  $ret = (string)($_GET['return'] ?? '');
  if (!$app || empty($app['protected']) || empty($app['gate_secret'])) { http_response_code(404); exit('App nicht gefunden.'); }
  $o = fn($u) => strtolower((parse_url($u, PHP_URL_SCHEME) ?? '') . '://' . (parse_url($u, PHP_URL_HOST) ?? '') . (parse_url($u, PHP_URL_PORT) ? ':' . parse_url($u, PHP_URL_PORT) : ''));
  if ($ret === '' || $o($ret) !== $o($app['url'])) $ret = $app['url'];
  $c = session_customer();
  if (!$c) redirect(base_url() . '/#login&next=' . rawurlencode('portal.php?fn=sso&app=' . $app['id'] . '&return=' . rawurlencode($ret)));
  $st = access_state($c, $app);
  if (!$st['allowed']) redirect(base_url() . '/#app=' . $app['id']);
  if ($st['state'] === 'trial_available') {
    $c = update_customer($c['id'], function ($c) use ($app) { if (empty($c['access'][$app['id']]['trial_start'])) $c['access'][$app['id']]['trial_start'] = gmdate('c'); return $c; });
    $st = access_state($c, $app);
  }
  // Freigabe-Ende: Freischaltung gilt einschliesslich des UTC-Kalendertags, Testphase bis zum genauen Zeitpunkt
  if ($st['state'] === 'active') { $until = $st['until']; $expUnix = strtotime($until . ' 00:00:00 UTC') + 86400; }
  else { $expUnix = strtotime($st['until']); $until = gmdate('c', $expUnix); }
  $tok = sign_token(['c' => $c['id'], 'm' => $c['email'], 'a' => $app['id'], 'e' => time() + 120,
                     'accessState' => $st['state'] === 'active' ? 'active' : 'trial', 'accessUntil' => $until, 'accessExpiresUnix' => $expUnix], $app['gate_secret']);
  redirect($ret . (strpos($ret, '?') === false ? '?' : '&') . 'cnc_token=' . rawurlencode($tok));
}

// ─── POST-Aufrufe (JSON) ────────────────────────────────────────────────────
if ($_SERVER['REQUEST_METHOD'] !== 'POST') out(['message' => 'bad request'], 400);
$b = json_decode(file_get_contents('php://input') ?: '{}', true);
if (!is_array($b)) $b = [];

switch ($fn) {

  case 'me': {
    $c = session_customer();
    out(['customer' => $c ? public_customer($c) : null, 'access' => $c ? access_list($c) : null, 'trial_days' => TRIAL_DAYS]);
  }

  case 'register':
  case 'request_link': {
    $email = norm_email($b['email'] ?? '');
    if (!$email) out(['error' => 'bad_email']);
    if (!rate_ok('ip:' . client_ip(), 20, 3600) || !rate_ok('mail:' . $email, 5, 3600)) out(['error' => 'rate_limited']);
    $c = find_customer(fn($c) => $c['email'] === $email);
    if (!$c && $fn === 'register') {
      $c = with_lock(function () use ($email, $b) {
        $all = load('customers', []);
        foreach ($all as $x) if ($x['email'] === $email) return $x;
        $n = new_customer($email, trim((string)($b['name'] ?? '')), trim((string)($b['company'] ?? '')));
        $all[] = $n; save('customers', $all); return $n;
      });
    }
    // Antwort unabhaengig davon, ob das Konto existiert (keine Auskunft ueber registrierte Adressen)
    if ($c && empty($c['blocked'])) {
      if (!send_login_link($c, lang_of($b), safe_next($b['next'] ?? ''))) out(['error' => 'mail_failed']);
    }
    out(['ok' => true]);
  }

  case 'logout': end_session(); out(['ok' => true]);

  case 'profile': {
    $c = session_customer() ?: out(['error' => 'not_logged_in']);
    $c = update_customer($c['id'], function ($c) use ($b) {
      $c['name'] = mb_substr(trim((string)($b['name'] ?? $c['name'])), 0, 120);
      $c['company'] = mb_substr(trim((string)($b['company'] ?? $c['company'])), 0, 120);
      return $c;
    });
    out(['ok' => true, 'customer' => public_customer($c)]);
  }

  case 'start_trial': {   // Testphase manuell starten (z. B. Klick auf "Öffnen")
    $c = session_customer() ?: out(['error' => 'not_logged_in']);
    $app = apps_by_id()[$b['app'] ?? ''] ?? out(['error' => 'not_found']);
    if (access_state($c, $app)['state'] === 'trial_available')
      $c = update_customer($c['id'], function ($c) use ($app) { $c['access'][$app['id']]['trial_start'] = gmdate('c'); return $c; });
    out(['ok' => true, 'access' => access_state($c, $app)]);
  }

  // ── Passkeys ──
  case 'passkey_register_options': {
    $c = session_customer() ?: out(['error' => 'not_logged_in']);
    $ch = wa_new_challenge($c['id']);
    out(['publicKey' => [
      'challenge' => b64u_enc($ch),
      'rp' => ['id' => rp_id(), 'name' => 'cnc-technik.de Apps'],
      'user' => ['id' => b64u_enc(hex2bin(str_replace('-', '', $c['id']))), 'name' => $c['email'], 'displayName' => $c['name'] ?: $c['email']],
      'pubKeyCredParams' => [['type' => 'public-key', 'alg' => -7], ['type' => 'public-key', 'alg' => -257]],
      'authenticatorSelection' => ['residentKey' => 'required', 'requireResidentKey' => true, 'userVerification' => 'preferred'],
      'excludeCredentials' => array_map(fn($p) => ['type' => 'public-key', 'id' => $p['id']], $c['passkeys']),
      'attestation' => 'none', 'timeout' => 120000,
    ]]);
  }
  case 'passkey_register': {
    $c = session_customer() ?: out(['error' => 'not_logged_in']);
    $x = wa_take_challenge();
    if (!$x || $x['c'] !== $c['id']) out(['error' => 'challenge_expired']);
    try { $cred = wa_verify_register(b64u_dec($b['clientDataJSON'] ?? ''), b64u_dec($b['attestationObject'] ?? ''), b64u_dec($x['ch']), origin(), rp_id()); }
    catch (Exception $e) { out(['error' => 'passkey_invalid', 'detail' => $e->getMessage()]); }
    if (find_customer(fn($o) => (bool)array_filter($o['passkeys'], fn($p) => $p['id'] === $cred['id']))) out(['error' => 'passkey_exists']);
    $name = mb_substr(trim((string)($b['name'] ?? '')), 0, 60) ?: 'Passkey';
    update_customer($c['id'], function ($c) use ($cred, $name) {
      $c['passkeys'][] = ['id' => $cred['id'], 'pem' => $cred['pem'], 'counter' => $cred['counter'], 'name' => $name, 'created' => gmdate('c'), 'last_used' => null];
      return $c;
    });
    out(['ok' => true]);
  }
  case 'passkey_delete': {
    $c = session_customer() ?: out(['error' => 'not_logged_in']);
    update_customer($c['id'], function ($c) use ($b) { $c['passkeys'] = array_values(array_filter($c['passkeys'], fn($p) => $p['id'] !== ($b['id'] ?? ''))); return $c; });
    out(['ok' => true]);
  }
  case 'passkey_login_options': {
    if (!rate_ok('wa:' . client_ip(), 30, 600)) out(['error' => 'rate_limited']);
    $ch = wa_new_challenge();
    out(['publicKey' => ['challenge' => b64u_enc($ch), 'rpId' => rp_id(), 'userVerification' => 'preferred', 'allowCredentials' => [], 'timeout' => 120000]]);
  }
  case 'passkey_login': {
    $x = wa_take_challenge();
    if (!$x) out(['error' => 'challenge_expired']);
    $id = (string)($b['id'] ?? '');
    $c = find_customer(fn($o) => (bool)array_filter($o['passkeys'], fn($p) => $p['id'] === $id));
    if (!$c) out(['error' => 'passkey_unknown']);
    if (!empty($c['blocked'])) out(['error' => 'blocked']);
    $cred = array_values(array_filter($c['passkeys'], fn($p) => $p['id'] === $id))[0];
    try { $cnt = wa_verify_login(b64u_dec($b['clientDataJSON'] ?? ''), b64u_dec($b['authenticatorData'] ?? ''), b64u_dec($b['signature'] ?? ''), $cred, b64u_dec($x['ch']), origin(), rp_id()); }
    catch (Exception $e) { out(['error' => 'passkey_invalid', 'detail' => $e->getMessage()]); }
    update_customer($c['id'], function ($c) use ($id, $cnt) {
      foreach ($c['passkeys'] as &$p) if ($p['id'] === $id) { $p['counter'] = $cnt; $p['last_used'] = gmdate('c'); }
      return $c;
    });
    start_session($c['id']);
    out(['ok' => true]);
  }

  // ── Backend (Passwort) ──
  case 'admin_customers': {
    if ($e = admin_error()) out(['error' => $e]);
    $apps = array_values(array_map(fn($a) => ['id' => $a['id'], 'title' => $a['title_de'] ?: $a['title_en'], 'protected' => !empty($a['protected']), 'url' => $a['url']],
                                   array_filter(load('apps', []), fn($a) => !empty($a['protected']))));
    $byId = apps_by_id();
    $list = array_map(function ($c) use ($byId) {
      $st = [];
      foreach ($byId as $id => $a) if (!empty($a['protected'])) $st[$id] = access_state($c, $a) + ['raw' => $c['access'][$id] ?? new stdClass()];
      return ['id' => $c['id'], 'email' => $c['email'], 'name' => $c['name'], 'company' => $c['company'], 'created_at' => $c['created_at'],
              'last_login' => $c['last_login'], 'blocked' => !empty($c['blocked']), 'note' => $c['note'] ?? '', 'passkeys' => count($c['passkeys']), 'access' => $st];
    }, customers());
    usort($list, fn($a, $b) => strcmp($b['created_at'], $a['created_at']));
    out(['customers' => $list, 'apps' => $apps, 'trial_days' => TRIAL_DAYS]);
  }
  case 'admin_customer_save': {
    if ($e = admin_error()) out(['error' => $e]);
    $in = is_array($b['customer'] ?? null) ? $b['customer'] : [];
    $email = norm_email($in['email'] ?? '');
    if (!$email) out(['error' => 'bad_email']);
    $res = with_lock(function () use ($in, $email, $b) {
      $all = load('customers', []); $idx = null;
      foreach ($all as $i => $c) { if ($c['id'] === ($in['id'] ?? '')) $idx = $i; elseif ($c['email'] === $email) return ['error' => 'email_exists']; }
      $c = $idx === null ? new_customer($email) : $all[$idx];
      $c['email'] = $email;
      foreach (['name', 'company', 'note'] as $k) if (isset($in[$k])) $c[$k] = mb_substr(trim((string)$in[$k]), 0, $k === 'note' ? 2000 : 120);
      if (isset($in['blocked'])) $c['blocked'] = (bool)$in['blocked'];
      foreach ((is_array($b['access'] ?? null) ? $b['access'] : []) as $appId => $acc) {
        if (!valid_id($appId) || !is_array($acc)) continue;
        $cur = $c['access'][$appId] ?? [];
        if (array_key_exists('manual_until', $acc)) $cur['manual_until'] = (is_string($acc['manual_until']) && preg_match('/^\d{4}-\d{2}-\d{2}$/', $acc['manual_until'])) ? $acc['manual_until'] : null;
        if (array_key_exists('blocked', $acc)) $cur['blocked'] = (bool)$acc['blocked'];
        if (!empty($acc['trial_reset'])) $cur['trial_start'] = null;
        $c['access'][$appId] = $cur;
      }
      if ($idx === null) $all[] = $c; else $all[$idx] = $c;
      save('customers', $all);
      if (!empty($c['blocked'])) save('sessions', array_filter(load('sessions', []), fn($s) => $s['c'] !== $c['id']));
      return ['ok' => true, 'id' => $c['id'], 'new' => $idx === null];
    });
    if (!empty($res['new']) && !empty($b['invite'])) {
      $c = find_customer(fn($c) => $c['id'] === $res['id']);
      if (!send_login_link($c, lang_of($b), '__invite', true)) $res['mail'] = 'failed';
    }
    out($res);
  }
  case 'admin_customer_invite': {
    if ($e = admin_error()) out(['error' => $e]);
    $c = find_customer(fn($c) => $c['id'] === ($b['id'] ?? '')) ?: out(['error' => 'not_found']);
    out(send_login_link($c, lang_of($b), '__invite', true) ? ['ok' => true] : ['error' => 'mail_failed']);
  }
  case 'admin_customer_delete': {
    if ($e = admin_error()) out(['error' => $e]);
    $id = $b['id'] ?? '';
    with_lock(function () use ($id) {
      save('customers', array_values(array_filter(load('customers', []), fn($c) => $c['id'] !== $id)));
      save('sessions', array_filter(load('sessions', []), fn($s) => $s['c'] !== $id));
    });
    out(['ok' => true]);
  }
  case 'admin_gate_script': {
    if ($e = admin_error()) out(['error' => $e]);
    $app = apps_by_id()[$b['app'] ?? ''] ?? out(['error' => 'not_found']);
    if (empty($app['protected']) || empty($app['gate_secret'])) out(['error' => 'not_protected']);
    $tpl = file_get_contents(__DIR__ . '/gate-template.txt');
    out(['script' => strtr($tpl, ['%%APP_ID%%' => $app['id'], '%%SECRET%%' => $app['gate_secret'],
         '%%PORTAL%%' => base_url() . '/portal.php', '%%TITLE%%' => str_replace(["\n", "'", '*/'], ' ', $app['title_de']), '%%HOURS%%' => (string)GATE_HOURS])]);
  }

  default: out(['message' => 'bad request'], 400);
}
