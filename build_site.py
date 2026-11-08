#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
웹 배포용 정적 사이트 빌드
============================

`--viewer` 로 만든 자기 완결형 HTML 은 three.js(약 590 kB)와 지구 텍스처(약 230 kB)를 파일마다 내장한다.
혼자 열어 보기에는 편하지만 여러 개를 웹에 올리면 같은 자산이 중복되고 첫 로딩이 느리다.

이 스크립트는 공유 자산을 한 번만 올리고 시행마다 궤적 자료만 따로 두는 정적 사이트를 만든다.
Vercel, Netlify, GitHub Pages 처럼 정적 파일을 그대로 서빙하는 곳이면 어디든 그대로 올라간다.

    site/
      index.html             첫 화면: 연구 소개, 핵심 결과(대상별 성공률 곡선), 리플레이 목록
      runs/<이름>.html        뷰어 (공유 자산을 참조, 다른 시나리오로 가는 길이 위에 붙는다)
      data/<이름>.js          궤적 자료
      assets/three.min.js     공유
      assets/earth.jpg        공유
      assets/thumbs/<이름>.jpg 리플레이 썸네일 (--thumbs 로 찍는다)
      vercel.json             캐시 헤더 설정

입력은 두 가지를 받는다.
    - `--trajectory` 로 저장한 JSON
    - `--viewer` 로 만든 자기 완결형 HTML (안에 박힌 궤적을 도로 꺼내 쓴다. 재시뮬레이션이 필요 없다)

핵심 결과 절은 격자 실험 요약(results/results_summary.csv)에서 그린다. 파일이 없으면 그 절만 빠진다.

실행
    conda activate debris-sim
    python build_site.py                          # demo/*.html 로 site/ 생성
    python build_site.py --thumbs                 # 썸네일도 새로 찍는다 (Edge 또는 Chrome 필요)
    python build_site.py --inputs a.json b.html   # 파일을 직접 지정
    python build_site.py --serve                  # 빌드 후 로컬에서 확인 (http://localhost:8000)

썸네일은 리플레이를 바꿨을 때만 다시 찍으면 된다. 빌드는 site/ 를 지우지 않으므로 한 번 찍은 썸네일이 남는다.

배포
    site/ 를 커밋해 main 에 푸시하면 GitHub Actions 가 GitHub Pages 로 올린다 (.github/workflows/pages.yml).
    npx vercel --cwd site --prod     # Vercel 로 직접 올릴 때
"""

from __future__ import annotations

import argparse
import csv
import glob
import html
import json
import os
import re
import shutil
from typing import Dict, List, Optional

# 정적 사이트를 만드는 데 물리 엔진은 필요 없다. capture_sim 을 불러올 수 있으면 설정값을 그대로 쓰고,
# PyBullet 이 없는 환경(배포 서버 등)에서는 궤적 자료에 들어 있는 값과 아래 기본값으로 대체한다.
try:
    import capture_sim as _cs
    CONFIG: Optional[Dict] = _cs.CONFIG
except Exception:
    CONFIG = None

ASSET_PATHS = {"three_js": "assets/three.min.js", "earth_texture": "assets/earth.jpg"}
DEFAULT_TIMESTEP = 1.0 / 480.0

METHOD_ORDER = ["arm", "net", "harpoon"]
METHOD_LABELS = {"arm": "로봇팔", "net": "그물", "harpoon": "작살"}
# 3D 장면의 색과 맞춘다 (그물 질점 #86e07f, 테더 #ffb457). 로봇팔은 뷰어의 강조색을 쓴다
METHOD_COLORS = {"arm": "#5fc9ff", "net": "#86e07f", "harpoon": "#ffb457"}
TARGET_ORDER = ["small", "medium", "large"]
TARGET_LABELS = {"small": "소형 10 kg", "medium": "중형 500 kg", "large": "대형 2000 kg"}
TARGET_DESC = {"small": ("소형", "10 kg 큐브샛"), "medium": ("중형", "500 kg 위성"), "large": ("대형", "2,000 kg 로켓 단")}
# 첫 화면 큰 그림으로 쓸 리플레이. 앞에 있는 방식의 썸네일이 있으면 그것을 쓴다
HERO_PREFERENCE = ["net", "arm", "harpoon"]

VIEWER_THREE_MARKER = ('<!--__THREE_INLINE__--><script src="https://cdnjs.cloudflare.com/ajax/libs/'
                       'three.js/r128/three.min.js"></script>')


# =============================================================================
# 입력 읽기
# =============================================================================
def load_trajectory(path: str) -> Dict:
    """궤적 JSON, 또는 자기 완결형 뷰어 HTML 안에 박힌 궤적을 읽는다."""
    if path.lower().endswith(".json"):
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    src = open(path, encoding="utf-8").read()
    m = re.search(r"const DATA = (\{.*?);\r?\n\s*const EARTH_URI", src, re.S)
    if not m:
        raise SystemExit(f"궤적을 찾지 못했다: {path}\n`--viewer` 로 만든 HTML 이거나 `--trajectory` JSON 이어야 한다.")
    return json.loads(m.group(1).replace("<\\/", "</"))


def run_name(data: Dict, fallback: str) -> str:
    st, res = data.get("static", {}), data.get("result", {})
    method = st.get("method") or res.get("method")
    target = (st.get("target") or {}).get("kind") or res.get("target")
    spin = res.get("spin_deg_s")
    if method and target and spin is not None:
        return f"{method}_{target}_{float(spin):g}degs"
    return fallback


def run_kind(r: Dict):
    st = r["data"].get("static", {})
    return st.get("method", "?"), (st.get("target") or {}).get("kind", "?")


def load_summary(path: Optional[str]) -> Optional[Dict]:
    """격자 실험 요약에서 (대상, 방식)별 회전 속도-성공률 곡선을 읽는다. 파일이 없으면 None."""
    if not path or not os.path.exists(path):
        return None
    curves: Dict = {}
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            curves.setdefault((row["target"], row["method"]), []).append({
                "spin": float(row["spin_deg_s"]), "n": int(row["n"]), "k": int(row["successes"]),
                "rate": float(row["success_rate"]), "lo": float(row["ci_low"]), "hi": float(row["ci_high"])})
    for pts in curves.values():
        pts.sort(key=lambda p: p["spin"])
    return curves


def target_ranking(curves: Dict, target: str) -> List:
    """회전하는 대상(회전 속도 > 0)만 합친 방식별 성공률, 높은 순. 원고 요약문의 수치와 같은 기준이다."""
    out = []
    for m in METHOD_ORDER:
        pts = [p for p in curves.get((target, m), []) if p["spin"] > 0]
        n = sum(p["n"] for p in pts)
        if n:
            out.append((m, sum(p["k"] for p in pts) / n, n))
    return sorted(out, key=lambda x: -x[1])


# =============================================================================
# 페이지 생성
# =============================================================================
def build_viewer_page(template: str, data_src: str, nav: Optional[Dict] = None, title: Optional[str] = None) -> str:
    """공유 자산을 참조하는 뷰어 페이지. 궤적은 별도 스크립트가 먼저 넣어 둔다.
    nav 를 주면 뷰어 위쪽에 첫 화면과 다른 시나리오로 가는 길이 생긴다."""
    page = template
    nav_js = f"<script>window.__SITE_NAV__={json.dumps(nav, ensure_ascii=False)};</script>\n" if nav else ""
    page = page.replace(VIEWER_THREE_MARKER,
                        f'{nav_js}<script src="{data_src}"></script>\n<script src="../assets/three.min.js"></script>')
    page = page.replace("/*__DATA_JSON__*/", "")          # window.__DEBRIS_DATA__ 를 그대로 쓴다
    page = page.replace("__EARTH_DATA_URI__", "../assets/earth.jpg")
    if title:
        page = re.sub(r"<title>.*?</title>", f"<title>{html.escape(title)}</title>", page, count=1)
    return ('<!DOCTYPE html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
            '</head>\n<body style="margin:0">\n' + page + '\n</body>\n</html>\n')


INDEX_CSS = """
:root{color-scheme:dark;--bg:#03050b;--panel:rgba(14,20,34,.74);--line:#1d2940;--line2:#2b3a57;
--text:#eef2f9;--muted:#93a0b8;--dim:#66728c;--accent:#5fc9ff;--good:#4ad494;--bad:#ff6b6b;
--font:"Pretendard Variable",Pretendard,"IBM Plex Sans","Malgun Gothic",system-ui,sans-serif;
--mono:"IBM Plex Mono",Consolas,monospace}
*{box-sizing:border-box}
html{scroll-behavior:smooth}
body{margin:0;color:var(--text);font-family:var(--font);line-height:1.5;-webkit-font-smoothing:antialiased;
background-color:var(--bg);background-repeat:no-repeat;
background-image:radial-gradient(1100px 720px at 82% -12%,rgba(56,104,196,.34),transparent 62%),
radial-gradient(900px 600px at -12% 38%,rgba(36,64,130,.2),transparent 60%)}
a{color:inherit}
.sky{position:fixed;inset:0;width:100%;height:100%;z-index:-1;pointer-events:none}
.wrap{max-width:1200px;margin:0 auto;padding:0 24px}
.eyebrow{font-size:13px;font-weight:600;letter-spacing:.08em;color:var(--accent)}

.topbar{display:flex;align-items:center;justify-content:space-between;gap:16px;padding-top:22px;padding-bottom:22px}
.brand{font-weight:700;font-size:15px;letter-spacing:-.01em;text-decoration:none;display:flex;align-items:center;gap:10px}
.brand i{width:9px;height:9px;border-radius:50%;background:var(--accent);box-shadow:0 0 14px var(--accent)}
.topbar nav{display:flex;gap:4px;flex-wrap:wrap}
.topbar nav a{padding:7px 12px;border-radius:8px;color:var(--muted);text-decoration:none;font-size:14px}
.topbar nav a:hover{color:var(--text);background:rgba(255,255,255,.05)}

.hero{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.1fr);gap:52px;align-items:center;
min-height:min(calc(100vh - 80px),860px);padding-bottom:48px}
.hero.solo{grid-template-columns:minmax(0,1fr)}
.hero h1{margin:16px 0 12px;font-size:clamp(34px,4.4vw,58px);line-height:1.14;font-weight:800;letter-spacing:-.04em;text-wrap:balance}
.hero h1 .hl{background:linear-gradient(95deg,#ffffff 0%,#b5e2ff 50%,#5fc9ff 100%);-webkit-background-clip:text;background-clip:text;color:transparent}
.subtitle{margin:0 0 22px;font-size:clamp(16px,1.6vw,20px);color:#c5cfe1;font-weight:500;letter-spacing:-.01em}
.lede{margin:0;color:var(--muted);font-size:15.5px;line-height:1.8;max-width:54ch}
.facts{display:grid;grid-template-columns:repeat(4,auto);justify-content:start;gap:12px 30px;margin:28px 0 30px;
padding:18px 0;border-top:1px solid var(--line);border-bottom:1px solid var(--line)}
.facts b{display:block;font-size:24px;font-weight:700;letter-spacing:-.02em;font-variant-numeric:tabular-nums;white-space:nowrap}
.facts span{font-size:12.5px;color:var(--muted)}
.cta{display:flex;gap:10px;flex-wrap:wrap}
.btn{display:inline-flex;align-items:center;gap:8px;padding:11px 18px;border-radius:10px;border:1px solid var(--line2);
background:rgba(255,255,255,.04);text-decoration:none;font-weight:600;font-size:14.5px;transition:border-color .15s,background .15s}
.btn:hover{border-color:var(--accent)}
.btn.primary{background:var(--accent);border-color:var(--accent);color:#03111b}
.btn.primary:hover{background:#8ad8ff}
.hero-shot{position:relative;display:block;aspect-ratio:16/9;border-radius:18px;overflow:hidden;background:#02040a;
border:1px solid var(--line2);box-shadow:0 30px 80px rgba(0,0,0,.6),0 0 100px rgba(60,120,220,.2)}
.hero-shot img{display:block;width:100%;height:100%;object-fit:cover;transition:transform .7s}
.hero-shot:hover img{transform:scale(1.03)}
.hero-cap{position:absolute;left:16px;bottom:16px;display:inline-flex;align-items:center;gap:10px;padding:7px 16px 7px 7px;
border-radius:999px;background:rgba(4,7,14,.74);backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);
border:1px solid var(--line2);font-size:14px;font-weight:600}
.playbtn{display:inline-grid;place-items:center;width:30px;height:30px;border-radius:50%;background:var(--accent);
color:#03111b;font-size:12px;padding-left:2px}

section.wrap{padding-top:88px;scroll-margin-top:-60px}
.sec-head{max-width:780px;margin-bottom:28px}
.sec-head h2{margin:10px 0 10px;font-size:clamp(25px,2.8vw,36px);font-weight:750;letter-spacing:-.035em;line-height:1.25;text-wrap:balance}
.sec-head p{margin:0;color:var(--muted);font-size:15px;line-height:1.75}

.grid3{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:18px}
.res{position:relative;padding:22px 22px 18px;border-radius:16px;background:var(--panel);border:1px solid var(--line);overflow:hidden}
.res::before{content:"";position:absolute;inset:0 0 auto;height:3px;background:var(--c)}
.res-target{display:flex;align-items:baseline;gap:8px}
.res-target b{font-size:18px;font-weight:700}
.res-target span{color:var(--muted);font-size:13.5px}
.res-win{display:flex;align-items:flex-end;justify-content:space-between;gap:12px;margin:16px 0 4px}
.res-method{display:flex;align-items:center;gap:9px;font-size:27px;font-weight:800;letter-spacing:-.03em;color:var(--c)}
.rank1{padding:3px 7px;border-radius:6px;background:var(--c);color:#03111b;font-size:11.5px;font-weight:700}
.res-rate{font-family:var(--mono);font-size:40px;font-weight:500;line-height:1;letter-spacing:-.03em;font-variant-numeric:tabular-nums}
.res-rate small{margin-left:2px;font-size:18px;color:var(--muted)}
.res-cap{color:var(--dim);font-size:12px}
.chart{display:block;width:100%;height:auto;margin:12px 0 8px}
.chart .grid{stroke:var(--line);stroke-width:1}
.chart .tick{fill:var(--dim);font-size:10px;font-family:var(--mono)}
.chart .axis{fill:var(--dim);font-size:10.5px}
.chart .band{opacity:.17}
.chart polyline{fill:none;stroke-linejoin:round;stroke-linecap:round}
.chart .win{stroke-width:3}
.chart .other{stroke-width:1.6;opacity:.6}
.res-rank{list-style:none;margin:0;padding:12px 0 0;border-top:1px dashed var(--line);display:grid;gap:6px;font-size:13.5px}
.res-rank li{display:flex;align-items:center;gap:8px;color:var(--muted)}
.res-rank li:first-child{color:var(--text);font-weight:600}
.res-rank i{width:9px;height:9px;border-radius:50%;background:var(--c)}
.res-rank b{margin-left:auto;font-family:var(--mono);font-weight:500;font-variant-numeric:tabular-nums}

.run{display:flex;flex-direction:column;border-radius:16px;overflow:hidden;background:var(--panel);border:1px solid var(--line);
text-decoration:none;transition:border-color .2s,transform .2s,box-shadow .2s}
.run:hover{border-color:var(--c);transform:translateY(-3px);box-shadow:0 18px 50px rgba(0,0,0,.5)}
.thumb{position:relative;aspect-ratio:16/9;overflow:hidden;background:radial-gradient(ellipse at 50% 130%,rgba(70,130,220,.4),transparent 60%),#05080f}
.thumb img{display:block;width:100%;height:100%;object-fit:cover;transition:transform .5s}
.run:hover .thumb img{transform:scale(1.04)}
.thumb .ph{position:absolute;inset:0;display:grid;place-items:center;color:var(--c);font-size:22px;font-weight:700}
.badge{position:absolute;top:12px;left:12px;padding:4px 10px;border-radius:999px;border:1px solid;font-size:12px;font-weight:600;
background:rgba(4,7,14,.76);backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px)}
.badge.ok{color:var(--good);border-color:rgba(74,212,148,.5)}
.badge.no{color:var(--bad);border-color:rgba(255,107,107,.5)}
.thumb .playbtn{position:absolute;right:12px;bottom:12px;width:40px;height:40px;font-size:14px;transition:transform .2s}
.run:hover .thumb .playbtn{transform:scale(1.12)}
.run-body{padding:16px 18px 18px}
.run h3{display:flex;align-items:center;gap:9px;margin:0 0 4px;font-size:18px;font-weight:700;letter-spacing:-.01em}
.run h3 i{width:10px;height:10px;border-radius:50%;background:var(--c);box-shadow:0 0 12px var(--c)}
.run .sub{color:var(--muted);font-size:13px}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:12px 14px;margin:14px 0 0;padding-top:14px;border-top:1px dashed var(--line)}
.stats dt{color:var(--muted);font-size:12px}
.stats dd{margin:2px 0 0;font-family:var(--mono);font-size:16px;font-variant-numeric:tabular-nums}

.guide{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px}
.guide>div{padding:20px 22px;border-radius:16px;background:var(--panel);border:1px solid var(--line);color:var(--muted);font-size:14px;line-height:1.8}
.guide h3{margin:0 0 8px;color:var(--text);font-size:15px;font-weight:700}
kbd{padding:1px 6px;border-radius:5px;border:1px solid var(--line2);background:rgba(255,255,255,.05);color:var(--text);font-family:var(--mono);font-size:12px}
footer.wrap{margin-top:72px;padding-top:24px;padding-bottom:56px;border-top:1px solid var(--line);color:var(--dim);font-size:12.5px}
@media (max-width:980px){.hero{grid-template-columns:minmax(0,1fr);min-height:0;gap:36px;padding-top:8px}}
@media (max-width:640px){.wrap{padding:0 16px}.topbar nav{display:none}.facts{grid-template-columns:1fr 1fr}section.wrap{padding-top:56px}
.res-rate{font-size:34px}}
@media (prefers-reduced-motion:reduce){html{scroll-behavior:auto}*{transition:none!important}}
"""

# 배경 별: 화면 크기에 맞춰 흩뿌리고 천천히 반짝이게 한다. 움직임 줄이기 설정이면 멈춘 채 그린다
SKY_JS = """
(function () {
  const c = document.querySelector(".sky"), g = c.getContext("2d");
  const still = matchMedia("(prefers-reduced-motion: reduce)").matches;
  let stars = [], w = 0, h = 0;
  function init() {
    const dpr = Math.min(devicePixelRatio || 1, 2);
    w = innerWidth; h = innerHeight; c.width = w * dpr; c.height = h * dpr; g.setTransform(dpr, 0, 0, dpr, 0, 0);
    let s = 7; const rnd = () => (s = (s * 16807) % 2147483647) / 2147483647;
    stars = Array.from({ length: Math.round(w * h / 4200) }, () => ({
      x: rnd() * w, y: rnd() * h, r: 0.35 + Math.pow(rnd(), 5) * 1.5, a: 0.2 + rnd() * 0.6, p: rnd() * 6.28, f: 0.3 + rnd() }));
  }
  function draw(t) {
    g.clearRect(0, 0, w, h); g.fillStyle = "#dfe8ff";
    for (const st of stars) {
      g.globalAlpha = still ? st.a : st.a * (0.7 + 0.3 * Math.sin(st.p + t * 0.0012 * st.f));
      g.beginPath(); g.arc(st.x, st.y, st.r, 0, 6.283); g.fill();
    }
    if (!still) requestAnimationFrame(draw);
  }
  addEventListener("resize", () => { init(); if (still) draw(0); });
  init(); requestAnimationFrame(draw);
})();
"""


def scene_facts(runs: List[Dict]) -> Dict:
    """색인 페이지에 쓸 설정값. capture_sim 이 있으면 그쪽을, 없으면 궤적 자료를 쓴다."""
    st = runs[0]["data"].get("static", {}) if runs else {}
    success = st.get("success") or {}
    orbit = st.get("orbit") or {}
    if CONFIG is not None:
        success = CONFIG["success"]
        orbit = CONFIG["orbit"]
    return {
        "threshold": float(success.get("rel_omega_threshold_deg_s", 1.0)),
        "hold": float(success.get("hold_time", 3.0)),
        "altitude": float(orbit.get("altitude_km", 500.0)),
        "timestep": float(CONFIG["sim"]["timestep"]) if CONFIG is not None
        else float(st.get("timestep", DEFAULT_TIMESTEP)),
    }


def success_chart(curves: Dict, target: str, winner: str) -> str:
    """대상 하나의 회전 속도-성공률 곡선(SVG). 1위 방식은 굵게 그리고 95 % 신뢰구간을 음영으로 깐다."""
    W, H, L, R, T, B = 320, 172, 36, 8, 8, 32
    pw, ph = W - L - R, H - T - B
    pts_all = [p for m in METHOD_ORDER for p in curves.get((target, m), [])]
    smax = max(p["spin"] for p in pts_all) or 1.0
    X = lambda s: L + s / smax * pw
    Y = lambda r: T + (1 - r) * ph
    out = [f'<svg class="chart" viewBox="0 0 {W} {H}" role="img" '
           f'aria-label="{TARGET_DESC.get(target, (target, ""))[0]} 대상의 회전 속도별 방식 성공률">']
    for r in (0, 0.5, 1):
        out.append(f'<line class="grid" x1="{L}" x2="{L + pw}" y1="{Y(r):.1f}" y2="{Y(r):.1f}"/>'
                   f'<text class="tick" x="{L - 6}" y="{Y(r) + 3.5:.1f}" text-anchor="end">{r * 100:.0f}%</text>')
    step = 10 if smax >= 20 else 5
    for s in range(0, int(smax) + 1, step):
        out.append(f'<text class="tick" x="{X(s):.1f}" y="{T + ph + 14}" text-anchor="middle">{s}</text>')
    out.append(f'<text class="axis" x="{L + pw}" y="{H - 2}" text-anchor="end">회전 속도 (deg/s)</text>')
    win = curves.get((target, winner), [])
    if win:
        band = ([f"{X(p['spin']):.1f},{Y(p['hi']):.1f}" for p in win] +
                [f"{X(p['spin']):.1f},{Y(p['lo']):.1f}" for p in reversed(win)])
        out.append(f'<polygon class="band" points="{" ".join(band)}" fill="{METHOD_COLORS[winner]}"/>')
    for m in [m for m in METHOD_ORDER if m != winner] + [winner]:
        pts = curves.get((target, m), [])
        if pts:
            line = " ".join(f"{X(p['spin']):.1f},{Y(p['rate']):.1f}" for p in pts)
            out.append(f'<polyline class="{"win" if m == winner else "other"}" points="{line}" stroke="{METHOD_COLORS[m]}"/>')
    out.append("</svg>")
    return "".join(out)


def build_results(curves: Dict) -> str:
    cards = []
    n_per = 0
    for target in TARGET_ORDER:
        rank = target_ranking(curves, target)
        if not rank:
            continue
        win, rate, n_per = rank[0]
        tname, tdesc = TARGET_DESC.get(target, (target, ""))
        items = "".join(f'<li style="--c:{METHOD_COLORS[m]}"><i></i>{METHOD_LABELS[m]}<b>{r * 100:.1f}%</b></li>'
                        for m, r, _ in rank)
        cards.append(f"""
        <article class="res" style="--c:{METHOD_COLORS[win]}">
          <div class="res-target"><b>{tname}</b><span>{tdesc}</span></div>
          <div class="res-win">
            <div class="res-method"><span class="rank1">1위</span>{METHOD_LABELS[win]}</div>
            <div class="res-rate">{rate * 100:.1f}<small>%</small></div>
          </div>
          <div class="res-cap">회전하는 대상 포획 성공률</div>
          {success_chart(curves, target, win)}
          <ol class="res-rank">{items}</ol>
        </article>""")
    spins = sorted({p["spin"] for pts in curves.values() for p in pts if p["spin"] > 0})
    span = f"{spins[0]:g}–{spins[-1]:g} deg/s" if spins else "회전"
    return f"""
  <section id="results" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">핵심 결과</div>
      <h2>파편의 질량에 따라 가장 유효한 방식이 달라진다</h2>
      <p>회전하는 대상({span})에 대한 방식별 포획 성공률이다. 조합마다 {n_per}회를 시뮬레이션했다.
      곡선은 회전 속도별 성공률이고, 음영은 1위 방식의 95 % 신뢰구간이다.</p>
    </div>
    <div class="grid3">{"".join(cards)}
    </div>
  </section>"""


def build_index(runs: List[Dict], curves: Optional[Dict] = None, thumbs: Optional[Dict] = None) -> str:
    thumbs = thumbs or {}
    fx = scene_facts(runs)

    cards = []
    for r in runs:
        res = r["data"].get("result", {})
        method, target = run_kind(r)
        ok = bool(res.get("success"))
        ct = res.get("capture_time")
        frames = len(r["data"].get("frames", []))
        dur = r["data"]["frames"][-1]["t"] if frames else 0
        label = f"{METHOD_LABELS.get(method, method)} · {TARGET_LABELS.get(target, target)}"
        thumb = thumbs.get(r["name"])
        img = (f'<img src="{thumb}" alt="{html.escape(label)} 포획 장면" loading="lazy">' if thumb
               else f'<div class="ph">{html.escape(METHOD_LABELS.get(method, method))}</div>')
        cards.append(f"""
      <a class="run" href="runs/{html.escape(r['name'])}.html" style="--c:{METHOD_COLORS.get(method, '#5fc9ff')}">
        <div class="thumb">{img}<span class="badge {'ok' if ok else 'no'}">{'포획 성공' if ok else '포획 실패'}</span><span class="playbtn" aria-hidden="true">▶</span></div>
        <div class="run-body">
          <h3><i></i>{html.escape(label)}</h3>
          <div class="sub">회전 {float(res.get('spin_deg_s', 0)):g} deg/s · 시드 {res.get('seed', 0)} · {float(dur):.0f} s 재생</div>
          <dl class="stats">
            <div><dt>감속 시간</dt><dd>{f'{float(ct):.1f} s' if ct else '—'}</dd></div>
            <div><dt>최대 충격량</dt><dd>{float(res.get('impulse', 0)):.1f} N·s</dd></div>
            <div><dt>잔류 상대 각속도</dt><dd>{float(res.get('final_rel_omega_deg_s', 0)):.2f} °/s</dd></div>
            <div><dt>서비서 자세 변화</dt><dd>{float(res.get('servicer_attitude_error', 0)):.0f}°</dd></div>
          </dl>
        </div>
      </a>""")

    # 첫 화면 오른쪽 큰 그림
    hero = next((r for m in HERO_PREFERENCE for r in runs if run_kind(r)[0] == m and r["name"] in thumbs), None)
    hero_shot = ""
    if hero:
        m, t = run_kind(hero)
        hero_shot = f"""
    <a class="hero-shot" href="runs/{html.escape(hero['name'])}.html">
      <img src="{thumbs[hero['name']]}" alt="{METHOD_LABELS.get(m, m)} 포획 장면">
      <span class="hero-cap"><span class="playbtn" aria-hidden="true">▶</span>{METHOD_LABELS.get(m, m)} 포획 리플레이 보기</span>
    </a>"""

    facts, lede_runs = "", ""
    if curves:
        total = sum(p["n"] for pts in curves.values() for p in pts)
        spins = sorted({p["spin"] for pts in curves.values() for p in pts})
        targets = [t for t in TARGET_ORDER if any(k[0] == t for k in curves)]
        masses = " · ".join(TARGET_DESC[t][1].split(" kg")[0] for t in targets)
        facts = f"""
      <div class="facts">
        <div><b>{total:,}회</b><span>몬테카를로 시행</span></div>
        <div><b>{len({k[1] for k in curves})}가지</b><span>포획 방식</span></div>
        <div><b>{masses} kg</b><span>파편 질량</span></div>
        <div><b>{spins[0]:g}–{spins[-1]:g}°/s</b><span>회전 속도</span></div>
      </div>"""
        lede_runs = (f" 질량 {masses} kg 파편을 {spins[0]:g}–{spins[-1]:g} deg/s 로 회전시키고 "
                     f"{total:,}회 포획을 시도해, 상대 각속도가 {fx['threshold']:g} deg/s 이하로 잦아들어 "
                     f"{fx['hold']:g} 초 이상 유지되면 성공으로 셌다.")

    return f"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>회전 우주 파편의 포획 방식 비교</title>
<meta name="description" content="로봇팔·그물·작살의 회전 우주 파편 포획을 물리 시뮬레이션으로 비교한 결과와 3D 리플레이">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/orioncactus/pretendard@v1.3.9/dist/web/variable/pretendardvariable-dynamic-subset.min.css">
<style>{INDEX_CSS}</style>
</head>
<body>
  <canvas class="sky" aria-hidden="true"></canvas>
  <div class="topbar wrap">
    <a class="brand" href="#"><i></i>회전 우주 파편 포획 시뮬레이션</a>
    <nav>{'<a href="#results">핵심 결과</a>' if curves else ''}<a href="#replays">3D 리플레이</a><a href="#guide">보는 법</a></nav>
  </div>

  <header class="hero wrap{'' if hero_shot else ' solo'}">
    <div>
      <div class="eyebrow">2026 공군사관학교 미래 항공우주 학술대회</div>
      <h1>회전하는 우주 파편의<br><span class="hl">포획 방식 비교</span></h1>
      <p class="subtitle">로봇팔 · 그물 · 작살의 물리 시뮬레이션 기반 정량 평가</p>
      <p class="lede">지구 저궤도 파편은 대부분 회전하고 있어, 제거하려면 회전체를 안전하게 붙잡아야 한다.
      PyBullet 다물체 동역학에 상대 궤도 역학을 더해 세 방식을 같은 조건에서 비교했다.{lede_runs}</p>{facts}
      <div class="cta">{'<a class="btn primary" href="#results">핵심 결과 보기</a>' if curves else ''}<a class="btn{'' if curves else ' primary'}" href="#replays">3D 리플레이</a></div>
    </div>{hero_shot}
  </header>
{build_results(curves) if curves else ''}
  <section id="replays" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">3D 리플레이</div>
      <h2>포획 순간 다시 보기</h2>
      <p>방식마다 대표 시행 하나를 물리 엔진이 계산한 궤적 그대로 재생한다. 시각화는 결과를 바꾸지 않는다.</p>
    </div>
    <div class="grid3">
{"".join(cards)}
    </div>
  </section>

  <section id="guide" class="wrap">
    <div class="guide">
      <div>
        <h3>보는 법</h3>
        마우스 드래그로 회전, 휠로 확대·축소, <kbd>Shift</kbd> 드래그로 이동한다.
        <kbd>Space</kbd> 재생·정지, <kbd>1</kbd>~<kbd>4</kbd> 카메라 추적 대상,
        <kbd>P</kbd> 발표 모드(핵심 수치만 크게), <kbd>O</kbd> 카메라 자동 회전.
        그물이 솜뭉치처럼 보이면 아래 "질점" 을 꺼서 스프링 선만 남긴다.
      </div>
      <div>
        <h3>성공 판정</h3>
        대상과 서비스 위성의 상대 각속도가 {fx['threshold']:g} deg/s 이하로 떨어지고 그 상태가
        {fx['hold']:g} 초 이상 유지되며 결합이 끊기지 않으면 성공으로 본다.
        뷰어 그래프의 가로 점선이 이 기준이고, 초록 음영은 결합이 유지된 구간이다.
      </div>
    </div>
  </section>

  <footer class="wrap">고도 {fx['altitude']:.0f} km 원궤도 기준 · 적분 간격 1/{round(1 / fx['timestep'])} s · 2026 공군사관학교 미래 항공우주 학술대회</footer>
  <script>{SKY_JS}</script>
</body>
</html>
"""


VERCEL_JSON = """{
  "headers": [
    {
      "source": "/assets/(.*)",
      "headers": [{ "key": "Cache-Control", "value": "public, max-age=31536000, immutable" }]
    },
    {
      "source": "/data/(.*)",
      "headers": [{ "key": "Cache-Control", "value": "public, max-age=3600" }]
    }
  ]
}
"""


# =============================================================================
# 썸네일
# =============================================================================
BROWSERS = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "google-chrome", "chromium", "chromium-browser", "microsoft-edge",
]


def find_browser() -> Optional[str]:
    for b in BROWSERS:
        p = b if os.path.isabs(b) else shutil.which(b)
        if p and os.path.exists(p):
            return p
    return None


def thumb_time(data: Dict) -> float:
    """썸네일 시각: 첫 접촉 2.5 초 뒤. 그물이 감싸거나 팔이 붙잡거나 테더가 당겨진 장면이 나온다."""
    t_end = data["frames"][-1]["t"]
    t0 = data.get("result", {}).get("first_contact_time")
    return min(t_end, float(t0) + 2.5) if t0 is not None else t_end / 2


def to_jpeg(png: str) -> str:
    """PIL 이 있으면 JPEG 로 바꿔 용량을 크게 줄인다. 없으면 PNG 를 그대로 둔다."""
    try:
        from PIL import Image
    except ImportError:
        return png
    jpg = os.path.splitext(png)[0] + ".jpg"
    with Image.open(png) as im:
        im.convert("RGB").save(jpg, quality=86, optimize=True, progressive=True)
    os.remove(png)
    return jpg


def capture_thumbs(out_dir: str, runs: List[Dict], size=(1280, 720)) -> None:
    """헤드리스 Edge/Chrome 으로 각 뷰어를 ?clean&t=… 로 열어 assets/thumbs/<이름>.jpg 를 찍는다.
    file:// 로 열면 지구 텍스처가 WebGL 에 올라가지 않으므로 잠깐 로컬 서버를 띄워서 연다."""
    import functools
    import http.server
    import subprocess
    import tempfile
    import threading

    browser = find_browser()
    if not browser:
        print("썸네일: Edge 나 Chrome 을 찾지 못해 건너뛴다")
        return
    # 브라우저는 상대 경로를 자기 실행 폴더 기준으로 풀므로 절대 경로로 넘긴다
    out_dir = os.path.abspath(out_dir)
    thumbs_dir = os.path.join(out_dir, "assets", "thumbs")
    os.makedirs(thumbs_dir, exist_ok=True)

    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass

    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), functools.partial(Quiet, directory=out_dir))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    port = httpd.server_address[1]
    try:
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as profile:
            for r in runs:
                url = f"http://127.0.0.1:{port}/runs/{r['name']}.html?clean&t={thumb_time(r['data']):.2f}"
                png = os.path.join(thumbs_dir, r["name"] + ".png")
                subprocess.run([browser, "--headless=new", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
                                "--hide-scrollbars", f"--window-size={size[0]},{size[1]}", "--virtual-time-budget=8000",
                                f"--user-data-dir={profile}", f"--screenshot={png}", url],
                               capture_output=True, timeout=180)
                if not os.path.exists(png):
                    print(f"  썸네일 실패: {r['name']}")
                    continue
                print(f"  썸네일 {r['name']:30s} -> {os.path.basename(to_jpeg(png))}")
    finally:
        httpd.shutdown()


def find_thumbs(out_dir: str, runs: List[Dict]) -> Dict[str, str]:
    out = {}
    for r in runs:
        for ext in ("jpg", "png"):
            if os.path.exists(os.path.join(out_dir, "assets", "thumbs", f"{r['name']}.{ext}")):
                out[r["name"]] = f"assets/thumbs/{r['name']}.{ext}"
                break
    return out


# =============================================================================
# 빌드
# =============================================================================
def build(inputs: List[str], out_dir: str, summary: Optional[str] = None, thumbs: bool = False) -> List[Dict]:
    here = os.path.dirname(os.path.abspath(__file__))
    template = open(os.path.join(here, "viewer_template.html"), encoding="utf-8").read()
    for sub in ("runs", "data", "assets"):
        os.makedirs(os.path.join(out_dir, sub), exist_ok=True)

    # 공유 자산
    three = CONFIG["scene"]["three_js"] if CONFIG is not None else ASSET_PATHS["three_js"]
    earth = CONFIG["scene"]["earth_texture"] if CONFIG is not None else ASSET_PATHS["earth_texture"]
    for src_rel, dst in ((three, "three.min.js"), (earth, "earth.jpg")):
        src = os.path.join(here, src_rel)
        if not os.path.exists(src):
            raise SystemExit(f"공유 자산이 없다: {src}")
        shutil.copyfile(src, os.path.join(out_dir, "assets", dst))

    runs = []
    for path in inputs:
        data = load_trajectory(path)
        name = run_name(data, os.path.splitext(os.path.basename(path))[0])
        payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
        with open(os.path.join(out_dir, "data", f"{name}.js"), "w", encoding="utf-8") as f:
            f.write("window.__DEBRIS_DATA__=" + payload + ";\n")
        runs.append({"name": name, "data": data, "source": path})
        print(f"  {name:32s} <- {os.path.basename(path)}")

    runs.sort(key=lambda r: (METHOD_ORDER.index(run_kind(r)[0]) if run_kind(r)[0] in METHOD_ORDER else 9, r["name"]))

    # 뷰어 위쪽의 시나리오 이동 막대. 같은 방식·대상이 둘 이상이면 회전 속도로 구분한다
    kinds = [run_kind(r) for r in runs]
    nav_runs = []
    for r, (m, t) in zip(runs, kinds):
        label = f"{METHOD_LABELS.get(m, m)} · {TARGET_DESC.get(t, (t, ''))[0]}"
        if kinds.count((m, t)) > 1:
            label += f" {float(r['data'].get('result', {}).get('spin_deg_s', 0)):g}°/s"
        nav_runs.append({"name": r["name"], "href": f"{r['name']}.html", "label": label,
                         "color": METHOD_COLORS.get(m, "#5fc9ff")})
    for r, (m, t) in zip(runs, kinds):
        nav = {"home": "../index.html", "runs": [dict(n, current=n["name"] == r["name"]) for n in nav_runs]}
        title = f"{METHOD_LABELS.get(m, m)} 포획 · {TARGET_LABELS.get(t, t)} · 3D 리플레이"
        with open(os.path.join(out_dir, "runs", f"{r['name']}.html"), "w", encoding="utf-8") as f:
            f.write(build_viewer_page(template, f"../data/{r['name']}.js", nav, title))

    if thumbs:
        capture_thumbs(out_dir, runs)
    curves = load_summary(summary or os.path.join(here, "results", "results_summary.csv"))
    with open(os.path.join(out_dir, "index.html"), "w", encoding="utf-8") as f:
        f.write(build_index(runs, curves, find_thumbs(out_dir, runs)))
    with open(os.path.join(out_dir, "vercel.json"), "w", encoding="utf-8") as f:
        f.write(VERCEL_JSON)
    return runs


def total_size(path: str) -> int:
    return sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(path) for f in fs)


def main(argv=None):
    ap = argparse.ArgumentParser(description="웹 배포용 정적 사이트 빌드")
    ap.add_argument("--inputs", nargs="+", default=None,
                    help="궤적 JSON 또는 자기 완결형 뷰어 HTML. 기본값은 demo/*.html")
    ap.add_argument("--out", default="site")
    ap.add_argument("--summary", default=None,
                    help="핵심 결과 절에 쓸 격자 실험 요약 CSV. 기본값은 results/results_summary.csv")
    ap.add_argument("--thumbs", action="store_true", help="헤드리스 Edge/Chrome 으로 리플레이 썸네일을 새로 찍는다")
    ap.add_argument("--serve", action="store_true", help="빌드 후 로컬 서버로 확인")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args(argv)

    inputs = args.inputs or sorted(glob.glob("demo/*.html"))
    if not inputs:
        raise SystemExit("입력이 없다. `--viewer` 로 리플레이를 먼저 만들거나 --inputs 로 지정할 것.")
    print(f"입력 {len(inputs)}개")
    runs = build(inputs, args.out, args.summary, args.thumbs)
    size = total_size(args.out)
    print(f"\n사이트 {len(runs)}개 시행, 총 {size / 1e6:.1f} MB -> {os.path.abspath(args.out)}")
    print("배포:  site/ 를 커밋해 main 에 푸시하면 GitHub Pages 로 올라간다")

    if args.serve:
        import functools
        import http.server
        import socketserver
        handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=args.out)
        with socketserver.TCPServer(("", args.port), handler) as httpd:
            print(f"\nhttp://localhost:{args.port} 에서 확인 (Ctrl+C 로 종료)")
            httpd.serve_forever()


if __name__ == "__main__":
    main()
