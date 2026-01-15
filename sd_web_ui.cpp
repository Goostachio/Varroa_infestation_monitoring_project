#include "sd_web_ui.h"
#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>

static WebServer server(80);

static constexpr const char* AP_SSID = "ESP32-SD";
static constexpr const char* AP_PASS = ""; // open AP

static inline void no_cache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

static void json_chunk_begin() {
  no_cache();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");   // starts chunked response
}
static void json_chunk_end() {
  server.sendContent(""); 
}

static const char* mime_for(const char* path) {
  if (!path) return "application/octet-stream";
  const char* dot = strrchr(path, '.');
  if (!dot) return "application/octet-stream";
  if (!strcasecmp(dot, ".bmp")) return "image/bmp";
  if (!strcasecmp(dot, ".jpg") || !strcasecmp(dot, ".jpeg")) return "image/jpeg";
  if (!strcasecmp(dot, ".png")) return "image/png";
  return "application/octet-stream";
}

static bool is_image(const char* name) {
  if (!name) return false;
  const char* dot = strrchr(name, '.');
  if (!dot) return false;
  return !strcasecmp(dot, ".bmp") || !strcasecmp(dot, ".jpg") || !strcasecmp(dot, ".jpeg") || !strcasecmp(dot, ".png");
}

static bool safe_path(const String& p) {
  if (!p.length() || p[0] != '/') return false;
  if (p.indexOf("..") >= 0) return false;
  // only allow these
  return p.startsWith("/overlays/") || p.startsWith("/bee_overlays/");
}

static void handle_health() {
  no_cache();
  server.send(200, "text/plain", "ok");
}


static void handle_state_get() {
  no_cache();

  const uint32_t bees  = g_total_bees;
  const uint32_t mites = g_total_mites;
  const double avg_w = (bees > 0) ? (100.0 * (double)mites / (double)bees) : 0.0;

  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"infer\":%s,\"save\":%s,\"bees\":%lu,\"mites\":%lu,\"avg_weighted\":%.2f}",
           g_infer_enabled ? "true" : "false",
           g_save_enabled  ? "true" : "false",
           (unsigned long)bees,
           (unsigned long)mites,
           avg_w);

  server.send(200, "application/json", buf);
}

static void handle_state_post() {
  // infer=0/1 save=0/1 (query or form body)
  if (server.hasArg("infer")) g_infer_enabled = (server.arg("infer") != "0");
  if (server.hasArg("save"))  g_save_enabled  = (server.arg("save")  != "0");
  handle_state_get();
}

static const char* root_to_base(const String& root) {
  if (root == "overlays") return "/overlays";
  if (root == "bee_overlays") return "/bee_overlays";
  if (root == "bee") return "/bee_overlays";
  return nullptr;
}

static void handle_boots() {
  const char* base = root_to_base(server.hasArg("root") ? server.arg("root") : "");
  if (!base) {
    no_cache();
    server.send(400, "application/json", "{\"error\":\"bad root\"}");
    return;
  }

  // If base doesn't exist, return empty list quickly
  if (!SD_MMC.exists(base)) {
    no_cache();
    server.send(200, "application/json", "[]");
    return;
  }

  File dir = SD_MMC.open(base);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    no_cache();
    server.send(500, "application/json", "{\"error\":\"open dir failed\"}");
    return;
  }

  no_cache();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");  // start chunked

  // IMPORTANT: use sendContent for chunked responses
  server.sendContent("[");
  bool first = true;

  while (true) {
    File e = dir.openNextFile();
    if (!e) break;

    if (e.isDirectory()) {
      const char* nm = e.name();
      const char* bn = nm ? strrchr(nm, '/') : nullptr;
      bn = bn ? (bn + 1) : (nm ? nm : "");

      if (bn[0] && !strncmp(bn, "boot_", 5)) {
        if (!first) server.sendContent(",");
        first = false;

        server.sendContent("\"");
        server.sendContent(bn);
        server.sendContent("\"");
      }
    }

    e.close();
    delay(0); // feed WDT
  }

  server.sendContent("]");
  dir.close();

  // CRITICAL: finalize chunked transfer so fetch() doesn't "Load failed"
  server.sendContent("");
}

static void handle_images() {
  const char* base = root_to_base(server.hasArg("root") ? server.arg("root") : "");
  const String boot = server.hasArg("boot") ? server.arg("boot") : "";
  const String sub  = server.hasArg("sub")  ? server.arg("sub")  : "";

  if (!base || !boot.length()) {
    no_cache();
    server.send(400, "application/json", "{\"error\":\"missing root/boot\"}");
    return;
  }

  String dirPath;
  if (!strcmp(base, "/overlays")) {
    const String chosen = sub.length() ? sub : "mite";
    dirPath = String(base) + "/" + boot + "/" + chosen;
  } else {
    dirPath = String(base) + "/" + boot;
  }

  if (!SD_MMC.exists(dirPath)) {
    no_cache();
    server.send(200, "application/json", "[]");
    return;
  }

  File dir = SD_MMC.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    no_cache();
    server.send(500, "application/json", "{\"error\":\"open dir failed\"}");
    return;
  }

  json_chunk_begin();
  server.sendContent("[");
  bool first = true;

  while (true) {
    File e = dir.openNextFile();
    if (!e) break;

    if (!e.isDirectory()) {
      const char* nm = e.name();
      const char* bn = nm ? strrchr(nm, '/') : nullptr;
      bn = bn ? (bn + 1) : (nm ? nm : "");

      if (is_image(bn)) {
        if (!first) server.sendContent(",");
        first = false;

        server.sendContent("{\"name\":\"");
        server.sendContent(bn);
        server.sendContent("\",\"path\":\"");
        server.sendContent(dirPath.c_str());
        server.sendContent("/");
        server.sendContent(bn);
        server.sendContent("\"}");
      }
    }

    e.close();
    delay(0);
  }

  server.sendContent("]");
  dir.close();
  json_chunk_end();
}

static String normalize_path(String p) {
  // Handle servers/browsers that keep %2F in query args
  p.replace("%2F", "/");
  p.replace("%2f", "/");
  p.replace("\\", "/");
  return p;
}

static void handle_sd_file() {
  String p = server.hasArg("path") ? server.arg("path") : "";
  p = normalize_path(p);

  if (!safe_path(p)) {
    no_cache();
    server.send(403, "text/plain", "forbidden");
    return;
  }

  File f = SD_MMC.open(p, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    no_cache();
    server.send(404, "text/plain", "not found");
    return;
  }

  const char* mime = mime_for(p.c_str());
  const size_t total = f.size();

  no_cache();
  server.sendHeader("Content-Disposition", "inline");  // force display in <img>
  server.setContentLength(total);
  server.send(200, mime, ""); // headers only, body streamed below

  WiFiClient c = server.client();
  c.setNoDelay(true);
  static uint8_t buf[8192];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (!n) break;
    size_t w = c.write(buf, n);
    if (w != n) break;
    delay(0);
  }

  f.close();
}


static void handle_root() {
  static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>SD Viewer</title>
<style>
  body{font-family:system-ui;margin:0}
  header{padding:12px 14px;border-bottom:1px solid #eee;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  button{padding:10px 14px}
  select{padding:8px}
  .pill{padding:6px 10px;border:1px solid #ccc;border-radius:999px}
  .wrap{display:grid;grid-template-columns:340px 1fr;min-height:calc(100vh - 58px)}
  .left{border-right:1px solid #eee;display:flex;flex-direction:column}
  .left .controls{padding:12px;border-bottom:1px solid #eee;display:flex;flex-direction:column;gap:10px}
  .list{overflow:auto;flex:1}
  .item{padding:10px 12px;border-bottom:1px solid #f2f2f2;cursor:pointer;display:flex;gap:8px;align-items:center}
  .item:hover{background:#fafafa}
  .item.active{background:#eef6ff}
  .tag{font-size:12px;color:#666;border:1px solid #ddd;border-radius:999px;padding:2px 8px;white-space:nowrap}
  .name{font-size:13px;color:#111;word-break:break-all}

  /* RIGHT: two-column layout, stats sidebar on the far right */
  .right{display:grid;grid-template-columns:1fr 280px;grid-template-rows:auto 1fr;min-width:0}
  .viewerTop{grid-column:1 / 3;padding:12px;border-bottom:1px solid #eee;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  .path{font-size:12px;color:#666;word-break:break-all}
  .viewer{grid-column:1 / 2;padding:12px;display:flex;justify-content:center;align-items:flex-start;flex:1;background:#fafafa;min-width:0}
  img{max-width:100%;max-height:calc(100vh - 160px);border-radius:14px;border:1px solid #ddd;background:#fff}
  .hint{color:#666;font-size:13px}

  .stats{grid-column:2 / 3;padding:12px;border-left:1px solid #eee;background:#fff;overflow:auto}
  .statCard{border:1px solid #eee;border-radius:14px;padding:12px;margin-bottom:10px}
  .statLabel{font-size:12px;color:#666;margin-bottom:6px}
  .statValue{font-size:22px;font-weight:700;color:#111}
  .statSub{font-size:12px;color:#666;margin-top:4px}
</style>
</head><body>

<header>
  <button id="start">Start infer+save</button>
  <button id="stop">Stop (browse SD)</button>
  <button id="refresh">Refresh</button>
  <span class="pill" id="state">state: ...</span>
</header>

<div class="wrap">
  <div class="left">
    <div class="controls">
      <div style="display:flex;gap:10px;flex-wrap:wrap;align-items:center">
        <span class="tag">Root</span>
        <select id="rootSel">
          <option value="bee_overlays">bee_overlays</option>
          <option value="overlays">overlays</option>
        </select>

        <span class="tag">Boot</span>
        <select id="bootSel"></select>

        <span class="tag" id="subTag" style="display:none">Sub</span>
        <select id="subSel" style="display:none">
          <option value="mite">mite</option>
          <option value="no_mite">no_mite</option>
        </select>
      </div>

      <div class="hint" id="counts">files: 0</div>
    </div>

    <div class="list" id="fileList"></div>
  </div>

  <div class="right">
    <div class="viewerTop">
      <span class="tag" id="viewerTag">preview</span>
      <span class="path" id="viewerPath">Click a file on the left.</span>
    </div>

    <div class="viewer" id="viewerArea">
      <div class="hint">No image selected.</div>
    </div>

    <!-- Stats sidebar -->
    <aside class="stats">
      <div class="statCard">
        <div class="statLabel">Summed bees (this boot)</div>
        <div class="statValue" id="statBees">0</div>
        <div class="statSub">Total detections accumulated</div>
      </div>

      <div class="statCard">
        <div class="statLabel">Summed varroa (this boot)</div>
        <div class="statValue" id="statMites">0</div>
        <div class="statSub">Total mites detected</div>
      </div>

      <div class="statCard">
        <div class="statLabel">Weighted average</div>
        <div class="statValue" id="statAvg">0.00%</div>
        <div class="statSub">100 * mites / bees</div>
      </div>
    </aside>
  </div>
</div>

<script>
const qs = id => document.getElementById(id);

function sdUrl(path){
  const enc = encodeURIComponent(path).replace(/%2F/gi, "/");
  return "/sd?path=" + enc + "&t=" + Date.now();
}

async function jget(url){
  const r = await fetch(url, {cache:"no-store"});
  if(!r.ok) throw new Error("HTTP "+r.status);
  return await r.json();
}

async function postState(infer, save){
  const body = new URLSearchParams({infer: infer?"1":"0", save: save?"1":"0"});
  const r = await fetch("/api/state", {
    method:"POST",
    headers:{"Content-Type":"application/x-www-form-urlencoded"},
    body,
    cache:"no-store"
  });
  if(!r.ok) throw new Error("HTTP "+r.status);
  return await r.json();
}

function fmtPct(x){
  const n = Number(x);
  if(!Number.isFinite(n)) return "0.00%";
  return n.toFixed(2) + "%";
}

async function loadState(){
  const st = await jget("/api/state?t="+Date.now());
  qs("state").textContent = `state: infer=${st.infer} save=${st.save}`;

  // Stats
  qs("statBees").textContent  = String(st.bees ?? 0);
  qs("statMites").textContent = String(st.mites ?? 0);
  qs("statAvg").textContent   = fmtPct(st.avg_weighted ?? 0);

  return st;
}

async function loadBoots(){
  const root = qs("rootSel").value;
  const boots = await jget(`/api/boots?root=${encodeURIComponent(root)}&t=${Date.now()}`);
  const s = qs("bootSel");
  const prev = s.value;
  s.innerHTML = "";
  for(const b of boots){
    const o=document.createElement("option");
    o.value=b; o.textContent=b;
    s.appendChild(o);
  }
  if(prev && boots.includes(prev)) s.value = prev;
  else if(boots.length) s.value = boots[0];
  return boots;
}

let g_items = [];
let g_activePath = "";

function renderList(items){
  const list = qs("fileList");
  list.innerHTML = "";
  qs("counts").textContent = `files: ${items.length}`;

  for(const it of items){
    const row = document.createElement("div");
    row.className = "item" + (it.path === g_activePath ? " active" : "");
    row.onclick = () => selectItem(it);

    const tag = document.createElement("div");
    tag.className = "tag";
    tag.textContent = "img";

    const name = document.createElement("div");
    name.className = "name";
    name.textContent = it.path;

    row.appendChild(tag);
    row.appendChild(name);
    list.appendChild(row);
  }
}

let g_imgLoading = false;

function selectItem(it){
  g_activePath = it.path;
  renderList(g_items);

  qs("viewerPath").textContent = it.path;
  const area = qs("viewerArea");
  area.innerHTML = "";

  const img = document.createElement("img");
  img.alt = it.path;

  g_imgLoading = true;
  img.onload = () => { g_imgLoading = false; };
  img.onerror = () => {
    g_imgLoading = false;
    area.innerHTML = `<div class="hint">Failed to load image.<br>${it.path}</div>`;
  };

  img.src = sdUrl(it.path);
  area.appendChild(img);
}

async function loadImagesList(){
  const root = qs("rootSel").value;
  const boot = qs("bootSel").value;
  const sub  = qs("subSel").value;

  if(!boot){
    g_items = [];
    g_activePath = "";
    renderList([]);
    qs("viewerPath").textContent = "No boot folders found.";
    qs("viewerArea").innerHTML = `<div class="hint">No image selected.</div>`;
    return;
  }

  let url = `/api/images?root=${encodeURIComponent(root)}&boot=${encodeURIComponent(boot)}&t=${Date.now()}`;
  if(root === "overlays") url += `&sub=${encodeURIComponent(sub)}`;

  const items = await jget(url);
  g_items = items;
  if(g_activePath && !items.some(x => x.path === g_activePath)) g_activePath = "";
  renderList(items);
}

function syncSubUi(){
  const isVar = (qs("rootSel").value === "overlays");
  qs("subSel").style.display = isVar ? "" : "none";
  qs("subTag").style.display = isVar ? "" : "none";
}

async function refreshAll(){
  await loadState();
  syncSubUi();
  await loadBoots();
  await loadImagesList();
}

qs("start").onclick = async()=>{ await postState(true,true); await refreshAll(); };
qs("stop").onclick  = async()=>{ await postState(false,false); await refreshAll(); };
qs("refresh").onclick = async()=>{ await refreshAll(); };

qs("rootSel").onchange = async()=>{ g_activePath=""; syncSubUi(); await loadBoots(); await loadImagesList(); };
qs("bootSel").onchange = async()=>{ g_activePath=""; await loadImagesList(); };
qs("subSel").onchange  = async()=>{ g_activePath=""; await loadImagesList(); };

setInterval(async()=>{
  try{
    const st = await loadState();
    if(!st.infer && !st.save && !g_imgLoading){
      await loadImagesList();
    }
  }catch{}
}, 8000);

refreshAll();
</script>
</body></html>
)HTML";

  no_cache();
  server.send(200, "text/html", PAGE);
}

void sd_web_ui_begin() {
  static bool started = false;
  if (started) return;
  started = true;

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

  Serial.printf("[WEB] AP %s IP=%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, handle_root);
  server.on("/api/health", HTTP_GET, handle_health);
  server.on("/api/state", HTTP_GET, handle_state_get);
  server.on("/api/state", HTTP_POST, handle_state_post);
  server.on("/api/boots", HTTP_GET, handle_boots);
  server.on("/api/images", HTTP_GET, handle_images);
  server.on("/sd", HTTP_GET, handle_sd_file);
  server.onNotFound([](){
    no_cache();
    server.send(404, "text/plain", "not found");
  });

  server.begin();
  Serial.println("[WEB] server started");
}

void sd_web_ui_loop() {
  server.handleClient();
  delay(0);
}
