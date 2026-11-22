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
    python build_site.py --standalone demo        # 인터넷 없이 열리는 단독 뷰어도 새 템플릿으로 다시 쓴다
    python build_site.py --inputs a.json b.html   # 파일을 직접 지정
    python build_site.py --serve                  # 빌드 후 로컬에서 확인 (http://localhost:8000)

썸네일은 리플레이를 바꿨을 때만 다시 찍으면 된다. 빌드는 site/ 를 지우지 않으므로 한 번 찍은 썸네일이 남는다.

배포
    site/ 를 커밋해 main 에 푸시하면 GitHub Actions 가 GitHub Pages 로 올린다 (.github/workflows/pages.yml).
    npx vercel --cwd site --prod     # Vercel 로 직접 올릴 때
"""

from __future__ import annotations

import argparse
import base64
import csv
import glob
import html
import json
import math
import os
import re
import shutil
import statistics
from typing import Dict, List, Optional
from urllib.parse import quote

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
TARGET_LABELS = {"small": "소형 10 kg", "medium": "중형 500 kg", "large": "대형 2,000 kg"}
TARGET_DESC = {"small": ("소형", "10 kg 큐브샛"), "medium": ("중형", "500 kg 위성"), "large": ("대형", "2,000 kg 로켓 단")}
# 첫 화면 큰 그림으로 쓸 리플레이. 앞에 있는 방식의 썸네일이 있으면 그것을 쓴다
HERO_PREFERENCE = ["net", "arm", "harpoon"]
# 첫 화면에 싣는 팀 정보. 공군사관학교 마크는 assets/ 에 아래 이름의 그림이 있을 때만 붙는다
TEAM = {"name": "영윤준서결", "members": ["배영윤", "최윤준", "이준서", "김결"]}
EMBLEM_NAMES = ["afa_emblem.png", "afa_emblem.svg", "afa_emblem.webp", "afa_emblem.jpg"]
# 결과 카드에 붙이는 해설. 오해받기 쉬운 결과에 원고와 같은 설명을 단다.
# zero_at_rest 는 "정지한 대상에서도 0 %" 라는 문장의 전제라 빌드할 때 자료로 확인하고, 맞지 않으면 싣지 않는다
RESULT_NOTES = {
    ("large", "net"): {"zero_at_rest": True,
                       "text": "그물 0 %는 회전하지 않는 대상에서도 0 %다. 한 변 5 m 그물이 지름 2 m·길이 3 m 원통"
                               "(외접 지름 3.6 m)을 감싸지 못한 그물 크기의 한계이며, 더 큰 그물로는 달라질 수 있다."},
}
# 실험 조건과 연구의 한계. 원고 표 1·2.3 절·3 장·4.7 절과 같은 내용이다 (값은 capture_sim.CONFIG 와 맞춰 확인했다).
# 원고를 고치면 여기도 같이 고친다
METHOD_SPEC = [
    ("궤도 환경", "고도 500 km 원궤도의 LVLH 좌표계. Clohessy–Wiltshire 상대 운동과 중력구배 토크를 매 스텝 부여"),
    ("물리 엔진", "PyBullet 다물체 동역학 · 적분 간격 1/480 s · 시행당 최대 180 s · 각감쇠 0 (각운동량 보존)"),
    ("서비스 위성", "1,000 kg · 한 변 1.5 m 정육면체 · x 방향에서 접근"),
    ("대상", "소형 한 변 0.3 m 정육면체 10 kg · 중형 본체와 좌우 2 m 태양전지판 500 kg · 대형 반지름 1 m·길이 3 m 원통 2,000 kg"),
    ("회전", "0~30 deg/s, 2 deg/s 간격 16 단계 · 접근축에 수직인 z 축 주축 회전 (중형은 x·y 축 추가)"),
    ("로봇팔", "6 관절, 도달 거리 2.6 m · 예비 접근 → 대기 → 파지 3 단계 · 순응 파지"),
    ("그물", "9 × 9 질점, 인장 전용 스프링 · 한 변 2 / 7 / 5 m (소형 / 중형 / 대형) · 접촉 후 둘레를 조이고 윈치로 감속"),
    ("작살", "2 kg, 12 m/s (RemoveDEBRIS 실증값 20 m/s 보다 보수적) · 관통 후 테더 윈치로 감속"),
    ("불확실성", "표준편차 회전축 10° · 회전 속도 5 % · 상대 위치 0.1 m · 상대 속도 0.02 m/s · 지향 1° · 초기 자세 10°"),
    ("반복", "조합당 30 회 (같은 반복 번호는 모든 조합이 같은 시드) · 총 4,320 회 + 회전축 추가 2,700 회"),
    ("통계", "Wilson 95 % 신뢰구간 · McNemar 정확검정 · Wilcoxon 부호순위 검정 · Holm–Bonferroni 보정"),
    ("수치 검증", "토크 없는 회전을 Runge–Kutta 해와 비교 (상대 오차 7 × 10⁻⁴) · 적분 간격 1/480·1/960·1/1920 s 수렴 시험"),
]
LIMITS = [
    ("가정값의 근거", "상당수가 문헌 근거 없이 정해졌다. 민감도 분석에서 결론을 가장 크게 흔든 값은 로봇팔의 그리퍼 접근 속도와 "
                     "표면 마찰계수이며, 마찰계수는 실측으로 확정해야 한다."),
    ("구현체 비교", "비교한 것은 세 방식 일반이 아니라 이 연구의 구현체다. 그물·작살의 성능은 윈치와 그물 크기 설계에 크게 좌우된다."),
    ("회전축", "세 주축을 모두 실험한 것은 중형 대상뿐이다. 소형·대형의 결론은 z 축 회전에서 얻었다."),
    ("외부 검증", "검증은 내부 일관성 확인에 해당한다. 지상 실험이나 궤도 실증 자료와의 비교가 필요하다."),
    ("작살 관통", "임계 속도·입사각 규칙으로 판정하며 재료의 파괴 역학은 다루지 않는다."),
]
FUTURE_WORK = "소형·대형 대상의 회전축 실험, 표면 마찰계수 실측, 윈치 장력 제어와 연성 도킹 최적화, 지상 실험을 통한 외부 검증"
# 슬라이드와 첫 화면의 '휴대폰으로 보기' 에 쓰는 사이트 주소 QR 코드. assets/ 에 있을 때만 붙는다
QR_NAME = "site_qr.svg"
# 회전축 추가 실험(중형 대상). 축 설명은 README 의 회전축 표와 같다
AXIS_ORDER = ["z", "x", "y"]
AXIS_DESC = {"z": "본 실험 · 최대 관성축", "x": "접근축 방향 · 중간 관성축", "y": "태양전지판 방향 · 최소 관성축"}
# 링크 미리보기(og:image)는 절대 주소여야 하므로 배포 주소를 알아야 한다. 다른 곳에 올리면 --site-url 로 바꾼다
SITE_URL = "https://leeejunseo.github.io/2026_Future-Aerospace-Conference/"
FAVICON = ("data:image/svg+xml," + quote(
    "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'>"
    "<circle cx='32' cy='32' r='31' fill='#05070d'/>"
    "<ellipse cx='32' cy='32' rx='25' ry='9' fill='none' stroke='#5fc9ff' stroke-width='3.5' transform='rotate(-28 32 32)'/>"
    "<circle cx='32' cy='32' r='8' fill='#86e07f'/></svg>", safe=""))

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


def load_axis(folder: str) -> Optional[Dict]:
    """회전축 추가 실험 요약. 축마다 방식별 성공률과 같은 축 안 방식 짝비교(Holm 보정 McNemar p)를 읽는다."""
    summ, tests = os.path.join(folder, "axis_summary.csv"), os.path.join(folder, "axis_tests.csv")
    if not (os.path.exists(summ) and os.path.exists(tests)):
        return None
    rates: Dict = {}
    with open(summ, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            rates.setdefault(row["axis"], {})[row["method"]] = (float(row["success_rate"]), int(row["n"]))
    pvals: Dict = {}
    with open(tests, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            if row["family"] == "축 안 방식 비교":
                a, b = (s.strip() for s in row["compare"].split("-"))
                pvals[(row["axis"], a, b)] = pvals[(row["axis"], b, a)] = float(row["mcnemar_p_holm"])
    return {"rates": rates, "p": pvals}


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
def head_meta(title: str, desc: str, url: Optional[str] = None, image: Optional[str] = None) -> str:
    """파비콘과 링크 미리보기(카카오톡·슬랙 등이 읽는 Open Graph) 태그. url·image 는 절대 주소여야 한다."""
    tags = [f'<link rel="icon" href="{FAVICON}">',
            '<meta property="og:type" content="website">',
            f'<meta property="og:title" content="{html.escape(title)}">',
            f'<meta property="og:description" content="{html.escape(desc)}">']
    if url:
        tags.append(f'<meta property="og:url" content="{html.escape(url)}">')
    if image:
        tags += [f'<meta property="og:image" content="{html.escape(image)}">',
                 '<meta name="twitter:card" content="summary_large_image">']
    return "\n".join(tags) + "\n"


def wrap_viewer(page: str, title: Optional[str], head: str = "") -> str:
    if title:
        page = re.sub(r"<title>.*?</title>", f"<title>{html.escape(title)}</title>", page, count=1)
    return ('<!DOCTYPE html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
            + head + '</head>\n<body style="margin:0">\n' + page + '\n</body>\n</html>\n')


def nav_script(nav: Optional[Dict]) -> str:
    payload = json.dumps(nav, ensure_ascii=False).replace("</", "<\\/")
    return f"<script>window.__SITE_NAV__={payload};</script>\n" if nav else ""


# 템플릿에서 치환하는 자리. 템플릿을 고치다 하나라도 사라지면 궤적이 빠진 페이지가 조용히 만들어지고,
# --standalone demo 는 그 페이지로 입력 파일을 덮어쓰므로 빌드 전에 멈춘다 (capture_sim.build_html 과 같은 검사)
TEMPLATE_MARKERS = ["/*__DATA_JSON__*/window.__DEBRIS_DATA__ || null", "__EARTH_DATA_URI__", VIEWER_THREE_MARKER]


def check_template(template: str) -> None:
    missing = [m for m in TEMPLATE_MARKERS if m not in template]
    if missing:
        raise SystemExit("viewer_template.html 에서 치환 자리를 찾지 못했다: " + ", ".join(missing))


def build_viewer_page(template: str, data_src: str, nav: Optional[Dict] = None, title: Optional[str] = None,
                      head: str = "") -> str:
    """공유 자산을 참조하는 뷰어 페이지. 궤적은 별도 스크립트가 먼저 넣어 둔다.
    nav 를 주면 뷰어 위쪽에 첫 화면과 다른 시나리오로 가는 길이 생긴다."""
    page = template.replace(VIEWER_THREE_MARKER,
                            f'{nav_script(nav)}<script src="{data_src}"></script>\n<script src="../assets/three.min.js"></script>')
    page = page.replace("/*__DATA_JSON__*/", "")          # window.__DEBRIS_DATA__ 를 그대로 쓴다
    page = page.replace("__EARTH_DATA_URI__", "../assets/earth.jpg")
    return wrap_viewer(page, title, head)


def build_standalone_page(template: str, data: Dict, three_js: str, earth_uri: str,
                          nav: Optional[Dict] = None, title: Optional[str] = None) -> str:
    """인터넷 없이 더블클릭으로 열리는 단독 뷰어. three.js·지구 텍스처·궤적을 모두 파일 안에 넣는다.
    capture_sim.TrajectoryRecorder.build_html 과 같은 치환이라 이 파일도 다시 build_site 의 입력으로 쓸 수 있다.
    물리 엔진 없이 만들 수 있으므로 뷰어 템플릿만 바꿨을 때 재시뮬레이션 없이 다시 만든다."""
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    page = template.replace("/*__DATA_JSON__*/window.__DEBRIS_DATA__ || null", payload)
    page = page.replace("__EARTH_DATA_URI__", earth_uri)
    page = page.replace(VIEWER_THREE_MARKER, nav_script(nav) + "<script>" + three_js + "</script>")
    return wrap_viewer(page, title, f'<link rel="icon" href="{FAVICON}">\n')


INDEX_CSS = """
:root{color-scheme:dark;--bg:#03050b;--panel:rgba(14,20,34,.74);--line:#1d2940;--line2:#2b3a57;
--text:#eef2f9;--muted:#93a0b8;--dim:#7f8ba5;--accent:#5fc9ff;--good:#4ad494;--bad:#ff6b6b;--warn:#f4b84a;
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
.hero h1{margin:14px 0 12px;font-size:clamp(34px,4vw,56px);line-height:1.14;font-weight:800;letter-spacing:-.04em;text-wrap:balance}
.hero h1 .hl{background:linear-gradient(95deg,#ffffff 0%,#b5e2ff 50%,#5fc9ff 100%);-webkit-background-clip:text;background-clip:text;color:transparent}
.subtitle{margin:0 0 16px;font-size:clamp(16px,1.6vw,20px);color:#c5cfe1;font-weight:500;letter-spacing:-.01em}
.team{display:flex;align-items:center;gap:12px;margin:0 0 20px}
.team img{width:52px;height:52px;flex:none;box-sizing:border-box;padding:5px;background:#fff;border-radius:50%;object-fit:contain}
.team b{display:block;font-size:15px;font-weight:700}
.team span{color:var(--muted);font-size:14px}
.lede{margin:0;color:var(--muted);font-size:15.5px;line-height:1.8;max-width:54ch}
.facts{display:flex;flex-wrap:wrap;gap:12px 22px;margin:24px 0 28px;
padding:18px 0;border-top:1px solid var(--line);border-bottom:1px solid var(--line)}
.facts b{display:block;font-size:clamp(18px,1.7vw,22px);font-weight:700;letter-spacing:-.02em;font-variant-numeric:tabular-nums;white-space:nowrap}
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
.res-cap{color:var(--dim);font-size:12.5px}
.res-note{margin-top:12px;padding:10px 12px;border-radius:10px;background:rgba(255,255,255,.04);color:var(--muted);font-size:12.5px;line-height:1.65}
.chart{display:block;width:100%;height:auto;margin:12px 0 8px}
.chart .grid{stroke:var(--line);stroke-width:1}
.chart .tick{fill:var(--dim);font-size:11.5px;font-family:var(--mono)}
.chart .axis{fill:var(--dim);font-size:11.5px}
.chart .band{opacity:.17}
.chart polyline{fill:none;stroke-linejoin:round;stroke-linecap:round}
.chart .win{stroke-width:3}
.chart .other{stroke-width:1.6;opacity:.6}
.res-rank{list-style:none;margin:0;padding:12px 0 0;border-top:1px dashed var(--line);display:grid;gap:6px;font-size:13.5px}
.res-rank li{display:flex;align-items:center;gap:8px;color:var(--muted)}
.res-rank li:first-child{color:var(--text);font-weight:600}
.res-rank i{width:9px;height:9px;border-radius:50%;background:var(--c)}
.res-rank b{margin-left:auto;font-family:var(--mono);font-weight:500;font-variant-numeric:tabular-nums}

.axis{margin-top:18px;padding:24px;border-radius:16px;background:var(--panel);border:1px solid var(--line)}
.axis h3{margin:8px 0 6px;font-size:21px;font-weight:700;letter-spacing:-.02em;text-wrap:balance}
.axis>p{margin:0 0 20px;max-width:780px;color:var(--muted);font-size:14px;line-height:1.7}
.axis-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px 28px}
.axis-name{margin-bottom:10px;font-size:16px;font-weight:700}
.axis-name span{display:block;color:var(--muted);font-size:12.5px;font-weight:500}
.bar{display:grid;grid-template-columns:44px 1fr 52px;align-items:center;gap:10px;margin:7px 0;color:var(--muted);font-size:13px}
.bar.win{color:var(--text);font-weight:600}
.track{height:10px;border-radius:5px;background:rgba(255,255,255,.06);overflow:hidden}
.track i{display:block;height:100%;border-radius:5px;background:var(--c)}
.bar:not(.win) .track i{opacity:.55}
.bar b{text-align:right;font-family:var(--mono);font-weight:500;font-variant-numeric:tabular-nums}
.axis-gap{margin-top:10px;padding-top:10px;border-top:1px dashed var(--line);color:var(--muted);font-size:13px}
.axis-gap b{color:var(--text);font-family:var(--mono);font-weight:500}
.axis-gap .ns{color:var(--warn)}
.grid4{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:16px}

.tbl-wrap{overflow-x:auto;border-radius:16px;background:var(--panel);border:1px solid var(--line)}
.cost{width:100%;min-width:820px;border-collapse:collapse;font-size:14px}
.cost th{padding:14px 14px 10px;text-align:left;color:var(--muted);font-size:12.5px;font-weight:600;vertical-align:bottom;
border-bottom:1px solid var(--line2);white-space:nowrap}
.cost th small{display:block;color:var(--dim);font-size:11.5px;font-weight:500}
.cost td{padding:8px 14px;border-bottom:1px solid var(--line)}
.cost tbody tr:last-child td{border-bottom:0}
.cost tr.first td{border-top:1px solid var(--line2)}
.cost tbody tr:first-child td{border-top:0}
.cost .tg{vertical-align:top;padding-top:12px;font-weight:700;white-space:nowrap}
.cost .tg span{display:block;color:var(--muted);font-size:12px;font-weight:500}
.cost .m{display:flex;align-items:center;gap:8px;white-space:nowrap;color:var(--muted)}
.cost .m i{width:9px;height:9px;border-radius:50%;background:var(--c)}
.cost .m em{padding:1px 6px;border-radius:5px;background:var(--c);color:#03111b;font-size:11px;font-style:normal;font-weight:700}
.cost .win .m{color:var(--text);font-weight:600}
.v{display:grid;grid-template-columns:64px minmax(40px,1fr);align-items:center;gap:10px}
.v b{text-align:right;font-family:var(--mono);font-weight:500;font-variant-numeric:tabular-nums;white-space:nowrap}
.v>span{height:6px;border-radius:3px;background:rgba(255,255,255,.06);overflow:hidden}
.v>span i{display:block;height:100%;border-radius:3px;background:var(--c);opacity:.75}
.v.na b{color:var(--dim);font-family:var(--font);font-size:12.5px}
.v.na>span{background:none}
.swipe{display:none;margin:8px 0 0;color:var(--dim);font-size:12.5px}
.cost-notes{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;margin-top:16px}
.cost-notes div{padding:15px 18px;border-radius:14px;border:1px solid var(--line);background:rgba(255,255,255,.025);
color:var(--muted);font-size:13.5px;line-height:1.7}
.cost-notes b{display:block;margin-bottom:3px;color:var(--text);font-size:14px}

.concl{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:16px}
.concl article{display:flex;flex-direction:column;padding:20px 22px;border-radius:16px;background:var(--panel);border:1px solid var(--line)}
.concl .no{font-size:12.5px;font-weight:600;color:var(--accent);letter-spacing:.02em}
.concl h3{margin:6px 0 8px;font-size:17.5px;font-weight:700;letter-spacing:-.02em;line-height:1.4;text-wrap:balance}
.concl p{margin:0;color:var(--muted);font-size:14px;line-height:1.75}
.ev{display:flex;flex-wrap:wrap;gap:6px;margin-top:auto;padding-top:14px}
.ev a{padding:4px 11px;border-radius:999px;border:1px solid var(--line2);font-size:12.5px;text-decoration:none;transition:border-color .15s}
.ev a:hover{border-color:var(--accent)}
.future{margin:18px 0 0;color:var(--muted);font-size:14px;line-height:1.7}
.future b{margin-right:8px;color:var(--text)}

.method{display:grid;grid-template-columns:minmax(0,1.3fr) minmax(0,1fr);gap:18px;align-items:start}
.method>div{padding:20px 22px;border-radius:16px;background:var(--panel);border:1px solid var(--line)}
.method h3{margin:0 0 14px;font-size:16px;font-weight:700}
.spec{display:grid;grid-template-columns:92px minmax(0,1fr);gap:9px 16px;margin:0;font-size:13.5px;line-height:1.6}
.spec dt{color:var(--muted)}
.spec dd{margin:0}
.limits{margin:0;padding-left:20px;color:var(--muted);font-size:14px;line-height:1.7}
.limits li{margin-bottom:10px}
.limits li::marker{color:var(--dim)}
.limits b{color:var(--text);font-weight:600}
.run-note{margin:14px 0 0;padding-top:12px;border-top:1px dashed var(--line);color:var(--muted);font-size:13px;line-height:1.65}
.qr{display:flex;gap:16px;align-items:flex-start}
.qr img{flex:none;width:112px;height:112px;padding:6px;border-radius:10px;background:#fff}

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
@media (max-width:980px){.hero{grid-template-columns:minmax(0,1fr);min-height:0;gap:36px;padding-top:8px}
.method{grid-template-columns:minmax(0,1fr)}}
@media (max-width:860px){.swipe{display:block}}
@media (max-width:640px){.wrap{padding:0 16px}.topbar nav{display:none}.facts>div{flex:1 1 40%}section.wrap{padding-top:56px}
.res-rate{font-size:34px}.spec{grid-template-columns:minmax(0,1fr);gap:0}.spec dd{margin-bottom:10px}.qr img{display:none}
.cost{min-width:0;font-size:13px}.cost th,.cost td{padding:8px 9px}.cost .tg span,.v>span,.cost th small{display:none}.v{display:block}.cost .m em{display:none}}
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


def fmt_p(p: Optional[float]) -> str:
    if p is None:
        return ""
    if p < 0.001:
        return "p &lt; 0.001"
    return f"p = {p:.3f}" + (' · <span class="ns">유의하지 않음</span>' if p >= 0.05 else "")


def build_axis(axis: Dict) -> str:
    """회전축 추가 실험 카드: 축마다 방식별 성공률 막대와 1·2위 차이."""
    cols, gaps, winners, n_cell, n_extra = [], {}, [], 0, 0
    for ax in AXIS_ORDER:
        r = axis["rates"].get(ax)
        if not r or len(r) < 2:
            continue
        rank = sorted(r.items(), key=lambda kv: -kv[1][0])
        (w, (rw, n_cell)), (s, (rs, _)) = rank[0], rank[1]
        winners.append(w)
        gaps[ax] = rw - rs
        if ax != "z":
            n_extra += sum(n for _, n in r.values())
        bars = "".join(
            f'<div class="bar{" win" if m == w else ""}" style="--c:{METHOD_COLORS[m]}"><span>{METHOD_LABELS[m]}</span>'
            f'<span class="track"><i style="width:{r[m][0] * 100:.1f}%"></i></span><b>{r[m][0] * 100:.1f}%</b></div>'
            for m in METHOD_ORDER if m in r)
        cols.append(f"""
        <div>
          <div class="axis-name">{ax} 축<span>{AXIS_DESC.get(ax, '')}</span></div>
          {bars}
          <div class="axis-gap">1·2위 차이 <b>{(rw - rs) * 100:+.1f} %p</b> · {fmt_p(axis['p'].get((ax, w, s)))}</div>
        </div>""")
    if not cols:
        return ""
    if len(set(winners)) == 1:
        lead = f"회전축을 바꿔도 {METHOD_LABELS[winners[0]]}이 1위다"
        others = [g for a, g in gaps.items() if a != "z"]
        if "z" in gaps and others and max(others) < gaps["z"] / 2:
            lead = f"회전축을 바꿔도 {METHOD_LABELS[winners[0]]}이 1위지만, 2위와의 차이는 크게 줄었다"
    else:
        lead = "회전축에 따라 1위 방식이 달라진다"
    return f"""
    <div class="axis">
      <div class="eyebrow">추가 실험 · 중형 500 kg 위성</div>
      <h3>{lead}</h3>
      <p>본 실험은 중형 대상을 z 축으로 돌렸다. 같은 시드로 x·y 축을 돌린 {n_extra:,}회를 더해 비교했다(축·방식당 {n_cell}회).
      1·2위 차이의 p 는 Holm 으로 보정한 McNemar 검정 값이다.</p>
      <div class="axis-grid">{"".join(cols)}
      </div>
    </div>"""


def build_results(curves: Dict, axis: Optional[Dict] = None) -> str:
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
        notes = []
        for m, _, _ in rank:
            note = RESULT_NOTES.get((target, m))
            if not note:
                continue
            rest = [p for p in curves.get((target, m), []) if p["spin"] == 0]
            if note.get("zero_at_rest") and not (rest and rest[0]["k"] == 0):
                print(f"  해설 생략: {target}/{m} 는 정지 대상 성공률이 0 이 아니어서 해설 문장과 맞지 않는다")
                continue
            notes.append(f'<p class="res-note">{html.escape(note["text"])}</p>')
        cards.append(f"""
        <article class="res" style="--c:{METHOD_COLORS[win]}">
          <div class="res-target"><b>{tname}</b><span>{tdesc}</span></div>
          <div class="res-win">
            <div class="res-method"><span class="rank1">1위</span>{METHOD_LABELS[win]}</div>
            <div class="res-rate">{rate * 100:.1f}<small>%</small></div>
          </div>
          <div class="res-cap">회전하는 대상 포획 성공률</div>
          {success_chart(curves, target, win)}
          <ol class="res-rank">{items}</ol>{"".join(notes)}
        </article>""")
    spins = sorted({p["spin"] for pts in curves.values() for p in pts if p["spin"] > 0})
    span = f"{spins[0]:g}–{spins[-1]:g} deg/s" if spins else "회전"
    return f"""
  <section id="results" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">핵심 결과</div>
      <h2>파편의 질량에 따라 가장 유효한 방식이 달라진다</h2>
      <p>큰 숫자와 순위는 회전하는 대상({span})만 모은 방식별 포획 성공률이다(조합마다 {n_per}회).
      곡선은 정지(0 deg/s)를 포함한 회전 속도별 성공률이고, 음영은 1위 방식의 95 % 신뢰구간이다.</p>
    </div>
    <div class="grid3">{"".join(cards)}
    </div>{build_axis(axis) if axis else ""}
  </section>"""


def _num(v) -> Optional[float]:
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if x == x else None                    # NaN 은 없는 값으로 본다


def load_costs(summary_path: str, raw_path: str) -> Optional[Dict]:
    """성공률 밖의 지표를 원고와 같은 기준으로 계산한다. 두 파일 중 하나라도 없으면 None.
    잔류 각속도·최대 충격량·서비스 위성 자세 변화는 원고 표 2 기준이다: 정지를 포함한 회전 속도 16 칸에서
    잔류 각속도는 칸 중앙값들의 중앙값(위쪽), 충격량과 자세 변화는 칸 평균들의 평균.
    감속 시간은 원고 표 3 기준이다: 회전 대상(> 0 deg/s)에서 성공한 시행의 중앙값."""
    if not (os.path.exists(summary_path) and os.path.exists(raw_path)):
        return None
    cells: Dict = {}
    with open(summary_path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            cells.setdefault((row["target"], row["method"]), []).append(row)
    times: Dict = {}
    with open(raw_path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            ct = _num(row.get("capture_time"))
            if (_num(row["spin_deg_s"]) or 0) > 0 and row["success"].strip().lower() in ("true", "1") and ct is not None:
                times.setdefault((row["target"], row["method"]), []).append(ct)
    out = {}
    for key, g in cells.items():
        rel = sorted(v for v in (_num(r["final_rel_omega_median_deg_s"]) for r in g) if v is not None)
        imp = [v for v in (_num(r["impulse_mean_Ns"]) for r in g) if v is not None]
        att = [v for v in (_num(r["servicer_attitude_error_mean_deg"]) for r in g) if v is not None]
        t = times.get(key, [])
        out[key] = {"capture": statistics.median(t) if t else None, "n_capture": len(t),
                    "rel": rel[len(rel) // 2] if rel else None,
                    "impulse": sum(imp) / len(imp) if imp else None,
                    "attitude": sum(att) / len(att) if att else None}
    return out


def keep_units(text: str) -> str:
    """숫자와 단위 사이를 줄바꿈 없는 공백으로 바꿔 '57.8' 과 '%p' 가 다른 줄로 갈라지지 않게 한다."""
    return re.sub(r"(\d) (%p|%|deg/s|N·s|s\b)", "\\1\u00a0\\2", text)


def rate_at(curves: Dict, target: str, method: str, spin: float) -> Optional[float]:
    return next((p["rate"] for p in curves.get((target, method), []) if abs(p["spin"] - spin) < 1e-9), None)


def build_costs(curves: Dict, costs: Dict) -> str:
    """'포획의 대가' 절: 대상·방식별 성공률, 감속 시간, 잔류 각속도, 최대 충격량, 서비스 위성 자세 변화 표.
    값 옆 막대는 같은 열 안에서 비교하라는 표시다. 감속 시간은 초와 분이 섞여 로그 눈금으로 그린다."""
    max_time = float(CONFIG["sim"]["max_time"]) if CONFIG is not None else 180.0
    present = [(t, m) for t in TARGET_ORDER for m in METHOD_ORDER if (t, m) in costs]
    if not present:
        return ""
    top = lambda k: max((costs[x][k] or 0) for x in present) or 1.0
    imp_max, rel_max = top("impulse"), top("rel")

    def cell(v, frac, fmt, c):
        if v is None:
            return '<td><span class="v na"><b>—</b><span></span></span></td>'
        return (f'<td><span class="v" style="--c:{c}"><b>{fmt.format(v)}</b>'
                f'<span><i style="width:{max(0.02, min(frac, 1.0)) * 100:.0f}%"></i></span></span></td>')

    body = []
    for t in TARGET_ORDER:
        rank = target_ranking(curves, t) if curves else []
        rates = {m: r for m, r, _ in rank}
        winner = rank[0][0] if rank else None
        ms = [m for m in METHOD_ORDER if (t, m) in costs]
        for i, m in enumerate(ms):
            c, x = METHOD_COLORS[m], costs[(t, m)]
            ct = x["capture"]
            tg = (f'<td class="tg" rowspan="{len(ms)}">{TARGET_DESC[t][0]}<span>{TARGET_DESC[t][1]}</span></td>' if i == 0 else "")
            rate = rates.get(m)
            body.append(
                f'<tr class="{"first " if i == 0 else ""}{"win" if m == winner else ""}">{tg}'
                f'<td><span class="m" style="--c:{c}"><i></i>{METHOD_LABELS[m]}{" <em>1위</em>" if m == winner else ""}</span></td>'
                + cell(rate * 100 if rate is not None else None, rate or 0, "{:.1f} %", c)
                + (cell(ct, math.log10(max(ct, 1.0)) / math.log10(max_time), "{:.1f} s" if ct < 10 else "{:.0f} s", c)
                   if ct is not None else '<td><span class="v na"><b>성공 없음</b><span></span></span></td>')
                + cell(x["rel"], (x["rel"] or 0) / rel_max, "{:.2f}", c)
                + cell(x["impulse"], (x["impulse"] or 0) / imp_max, "{:.1f}", c)
                + cell(x["attitude"], (x["attitude"] or 0) / 180.0, "{:.0f}°", c)
                + "</tr>")

    # 표 아래 세 줄 요약. 문장의 숫자는 모두 위 표에서 다시 계산한다
    notes = []
    arm_ct = [costs[(t, "arm")]["capture"] for t in TARGET_ORDER if (t, "arm") in costs and costs[(t, "arm")]["capture"]]
    teth_ct = [costs[(t, m)]["capture"] for t in TARGET_ORDER for m in ("net", "harpoon")
               if (t, m) in costs and costs[(t, m)]["capture"]]
    if arm_ct and teth_ct:
        notes.append(("감속 시간", f"성공한 시행의 중앙값이 로봇팔은 {min(arm_ct):.1f}~{max(arm_ct):.1f} s, "
                                  f"그물·작살은 {min(teth_ct):.0f}~{max(teth_ct):.0f} s 다. 강체로 붙잡으면 각운동량이 바로 넘어오지만, "
                                  f"테더는 윈치로 천천히 감속시킨다."))
    arm_imp = [costs[(t, "arm")]["impulse"] for t in TARGET_ORDER if (t, "arm") in costs]
    worst = max(present, key=lambda k: costs[k]["impulse"] or 0)
    if arm_imp:
        notes.append(("최대 충격량", f"로봇팔이 {min(arm_imp):.1f}~{max(arm_imp):.1f} N·s 로 가장 작고, "
                                    f"{METHOD_LABELS[worst[1]]} {TARGET_DESC[worst[0]][0]}이 {costs[worst]['impulse']:.1f} N·s 로 가장 크다. "
                                    f"그물 중형의 큰 충격은 성공·실패 시행에서 비슷해, 윈치가 대상을 끌어당겨 부딪히는 도킹 단계에서 생긴 것으로 본다."))
    med = [costs[("medium", m)]["attitude"] for m in METHOD_ORDER if ("medium", m) in costs]
    if len(med) == len(METHOD_ORDER):
        notes.append(("서비스 위성 자세", f"중형 대상에서는 세 방식 모두 서비스 위성이 평균 {min(med):.0f}° 이상 돌아갔다. "
                                         f"대상의 각운동량이 옮겨 온 결과라, 방식을 고를 때 자세 제어 용량을 함께 따져야 한다."))
    note_html = "".join(f"<div><b>{h}</b>{html.escape(keep_units(s))}</div>" for h, s in notes)
    return f"""
  <section id="costs" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">포획의 대가</div>
      <h2>성공해도 감속에 걸리는 시간은 초 단위와 분 단위로 갈린다</h2>
      <p>포획은 성공 여부만으로 끝나지 않는다. 성공률을 뺀 네 지표는 막대가 짧을수록 좋다.
      감속 시간은 회전 대상에서 성공한 시행의 중앙값(로그 눈금), 잔류 상대 각속도는 중앙값,
      최대 충격량과 서비스 위성 자세 변화는 정지를 포함한 전체 시행의 평균이다. 원고 표 2·3 과 같은 기준이다.</p>
    </div>
    <div class="tbl-wrap">
      <table class="cost">
        <thead><tr><th>대상</th><th>방식</th><th>회전 대상 성공률<small>높을수록 좋다</small></th><th>감속 시간<small>성공 시행 중앙값</small></th>
        <th>잔류 상대 각속도<small>deg/s</small></th><th>최대 충격량<small>N·s</small></th><th>서비스 위성 자세 변화<small>평균</small></th></tr></thead>
        <tbody>{"".join(body)}</tbody>
      </table>
    </div>
    <p class="swipe">표를 옆으로 밀어 나머지 열을 본다</p>
    <div class="cost-notes">{note_html}</div>
  </section>"""


def build_conclusions(curves: Dict, costs: Optional[Dict], axis: Optional[Dict], runs: List[Dict]) -> str:
    """결론 다섯 가지 (원고 4.8 절). 숫자는 자료에서 다시 계산하고, 근거가 되는 절과 리플레이로 가는 길을 단다."""
    have = {run_kind(r): r["name"] for r in runs}
    link = lambda m, t, label: (f'<a href="runs/{html.escape(have[(m, t)])}.html">▶ {label}</a>' if (m, t) in have else "")
    items = []

    # 1. 대상마다 1위가 다르다
    wins = {t: target_ranking(curves, t)[0] for t in TARGET_ORDER if target_ranking(curves, t)}
    if wins:
        same = len({w[0] for w in wins.values()}) == 1
        body = " · ".join(f"{TARGET_DESC[t][0]} {METHOD_LABELS[m]} {r * 100:.1f} %" for t, (m, r, _) in wins.items())
        body = f"회전하는 대상에서 1위는 {body} 였다."
        if axis:
            gaps = {}
            for ax in AXIS_ORDER:
                r = axis["rates"].get(ax)
                if r and len(r) >= 2:
                    top2 = sorted((v[0] for v in r.values()), reverse=True)[:2]
                    gaps[ax] = (top2[0] - top2[1]) * 100
            others = [g for ax, g in gaps.items() if ax != "z"]
            if "z" in gaps and others and max(others) < gaps["z"]:
                body += (f" 중형의 1·2위 차이 {gaps['z']:.1f} %p 는 회전축을 바꾸면 {min(others):.1f}~{max(others):.1f} %p 로 "
                         "줄어, 회전축도 방식을 고르는 변수다.")
        items.append(("대상마다 유효한 방식이 다르다" if same else "단일한 최적 포획 방식은 없다", body,
                      '<a href="#results">핵심 결과</a>'))

    # 2. 실패 이유
    items.append(("방식마다 실패하는 이유가 다르다",
                  "로봇팔은 태양전지판 같은 돌출 구조물에, 그물은 대상에 비해 작은 그물 크기에 막혔다. "
                  "작살은 대상에 박히는 데에는 대부분 성공했지만, 그 뒤 회전을 줄이지 못했다.",
                  link("arm", "medium", "로봇팔 · 중형 실패") + link("net", "large", "그물 · 대형 실패")))

    # 3. 성공 여부만으로는 부족하다
    body = "중형에서 로봇팔과 작살은 성공률로는 구별되지 않았지만(p = 0.13) 잔류 각속도로는 작살이 6.17 deg/s 낮아 뚜렷이 갈렸다."
    if costs:
        arm_ct = [costs[(t, "arm")]["capture"] for t in TARGET_ORDER if costs.get((t, "arm"), {}).get("capture")]
        teth = [costs[(t, m)]["capture"] for t in TARGET_ORDER for m in ("net", "harpoon") if costs.get((t, m), {}).get("capture")]
        if arm_ct and teth:
            body = (f"성공해도 감속 시간이 로봇팔 {min(arm_ct):.1f}~{max(arm_ct):.1f} s, 그물·작살 {min(teth):.0f}~{max(teth):.0f} s 로 "
                    "크게 달랐다. " + body)
    items.append(("성공 여부만으로는 부족하다", body,
                  ('<a href="#costs">포획의 대가</a>' if costs else "") + link("arm", "small", "로봇팔 · 소형 성공")))

    # 4. 빠를수록 어렵다는 직관이 깨진다
    a, b = rate_at(curves, "medium", "net", 0), rate_at(curves, "medium", "net", 18)
    c, d = rate_at(curves, "small", "arm", 14), rate_at(curves, "small", "arm", 28)
    body = "회전이 빠를수록 포획이 어렵다는 직관은 방식에 따라 성립하지 않았다."
    if None not in (a, b, c, d) and b > a and d > c:
        body = (f"중형 대상의 그물은 정지 {a * 100:.0f} % 에서 18 deg/s {b * 100:.0f} % 로 오히려 올랐고, "
                f"소형 대상의 로봇팔은 14 deg/s {c * 100:.0f} % 로 떨어졌다가 28 deg/s {d * 100:.0f} % 로 회복했다.")
    items.append(("빠를수록 어렵다는 직관은 방식에 따라 다르다", body, '<a href="#results">회전 속도별 곡선</a>'))

    # 5. 서비스 위성 자세
    med = [costs[("medium", m)]["attitude"] for m in METHOD_ORDER if costs and ("medium", m) in costs]
    body = "대상의 각운동량이 서비스 위성으로 옮겨 가므로, 방식을 고를 때 서비스 위성의 자세 제어 용량을 함께 따져야 한다."
    if len(med) == len(METHOD_ORDER):
        body = f"중형 대상에서는 세 방식 모두 서비스 위성이 평균 {min(med):.0f}° 이상 돌아갔다. " + body
    items.append(("포획에는 서비스 위성 자세라는 대가가 따른다", body, link("harpoon", "medium", "작살 · 중형 성공")))

    cards = "".join(f"""
      <article>
        <div class="no">결론 {i}</div>
        <h3>{h}</h3>
        <p>{html.escape(keep_units(b))}</p>
        {f'<div class="ev">{ev}</div>' if ev else ''}
      </article>""" for i, (h, b, ev) in enumerate(items, 1))
    return f"""
  <section id="conclusion" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">결론</div>
      <h2>회전 파편 포획에 정답 하나는 없다</h2>
      <p>세 방식을 같은 물리 환경과 같은 판정 기준으로 {sum(p["n"] for pts in curves.values() for p in pts):,}회,
      회전축 추가 실험까지 비교해 얻은 다섯 가지다. 각 카드의 링크로 근거가 되는 결과와 장면을 바로 볼 수 있다.</p>
    </div>
    <div class="concl">{cards}
    </div>
    <p class="future"><b>후속 연구</b> {html.escape(FUTURE_WORK)}</p>
  </section>"""


def build_method() -> str:
    spec = "".join(f"<dt>{html.escape(k)}</dt><dd>{html.escape(v)}</dd>" for k, v in METHOD_SPEC)
    lims = "".join(f"<li><b>{html.escape(k)}.</b> {html.escape(v)}</li>" for k, v in LIMITS)
    return f"""
  <section id="method" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">실험 조건과 한계</div>
      <h2>무엇을 어떻게 비교했고, 어디까지 말할 수 있나</h2>
    </div>
    <div class="method">
      <div>
        <h3>실험 조건</h3>
        <dl class="spec">{spec}</dl>
      </div>
      <div>
        <h3>연구의 한계</h3>
        <ol class="limits">{lims}</ol>
      </div>
    </div>
  </section>"""


def run_note(r: Dict) -> Optional[str]:
    """리플레이 카드의 해설 한 줄. 장면을 원고의 결론과 잇는다. 장면 자료가 해설의 전제와 맞을 때만 단다."""
    method, target = run_kind(r)
    res, status = r["data"].get("result", {}), r["data"].get("status_table", [])
    ok, ct = bool(res.get("success")), _num(res.get("capture_time"))
    att = _num(res.get("servicer_attitude_error")) or 0.0
    if (method, target) == ("arm", "medium") and not ok and not any(re.search(r"phase=(close|grasped)", s) for s in status):
        return "태양전지판이 서비스 위성과 먼저 부딪혀 파지 단계에 들어가지 못했다. 돌출 구조물이 로봇팔의 약점이다 (결론 2)."
    if (method, target) == ("arm", "small") and ok and ct is not None:
        return f"강체로 붙잡아 첫 접촉 후 {ct:.1f} s 만에 감속했다. 그물·작살은 같은 감속에 수십~백수십 초가 걸린다 (결론 3)."
    if (method, target) == ("net", "medium") and ok:
        return "형상과 상관없이 감쌀 수 있는 그물은 태양전지판에 걸려 오히려 유리하다. 중형에서 그물이 1위인 이유다."
    if (method, target) == ("harpoon", "medium") and ok and att >= 90:
        return f"포획에는 성공했지만 서비스 위성이 {att:.0f}° 돌아갔다. 대상의 각운동량이 옮겨 온 결과로, 포획의 대가다 (결론 5)."
    if (method, target) == ("net", "large") and not ok:
        est, lost = _num(res.get("bond_established_time")), _num(res.get("bond_lost_time"))
        text = ("한 변 5 m 그물이 지름 2 m·길이 3 m 원통을 감싸지 못해 "
                + (f"붙은 지 {lost - est:.1f} s 만에 벗겨졌다" if est is not None and lost is not None else "결합하지 못했다")
                + ". 대형에서 그물이 0 %인 이유다 (결론 2).")
        max_time = float(CONFIG["sim"]["max_time"]) if CONFIG is not None else 180.0
        sim_time = _num(res.get("sim_time")) or max_time
        if sim_time < max_time - 1 and r["data"].get("frames"):
            text += f" 장면은 앞 {r['data']['frames'][-1]['t']:.0f} s 만 기록했다."
        return text
    return None


def build_index(runs: List[Dict], curves: Optional[Dict] = None, thumbs: Optional[Dict] = None,
                axis: Optional[Dict] = None, site_url: str = SITE_URL, out_dir: Optional[str] = None,
                costs: Optional[Dict] = None) -> str:
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
        note = run_note(r)
        cards.append(f"""
      <a class="run" href="runs/{html.escape(r['name'])}.html" style="--c:{METHOD_COLORS.get(method, '#5fc9ff')}">
        <div class="thumb">{img}<span class="badge {'ok' if ok else 'no'}">{'포획 성공' if ok else '포획 실패'}</span><span class="playbtn" aria-hidden="true">▶</span></div>
        <div class="run-body">
          <h3><i></i>{html.escape(label)}</h3>
          <div class="sub">회전 {float(res.get('spin_deg_s', 0)):g} deg/s · 시드 {res.get('seed', 0)} · {float(dur):.0f} s 재생</div>
          <dl class="stats">
            <div><dt>감속 시간</dt><dd>{f'{float(ct):.1f} s' if ct else '—'}</dd></div>
            <div><dt>최대 충격량</dt><dd>{float(res.get('impulse', 0)):.1f} N·s</dd></div>
            <div><dt>잔류 상대 각속도</dt><dd>{float(res.get('final_rel_omega_deg_s', 0)):.2f} deg/s</dd></div>
            <div><dt>서비스 위성 최대 자세 변화</dt><dd>{float(res.get('servicer_attitude_error', 0)):.0f}°</dd></div>
          </dl>{f'''
          <p class="run-note">{html.escape(keep_units(note))}</p>''' if note else ''}
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

    # 리플레이 절 설명. 한 대상에 여러 방식이 있으면 같은 대상에서 결과가 갈리는 비교를 짚어 준다
    kinds = [run_kind(r) for r in runs]
    multi = next((t for t in TARGET_ORDER if len({m for m, k in kinds if k == t}) > 1), None)
    replay_note = "방식마다 대표 시행을 물리 엔진이 계산한 궤적 그대로 재생한다. 시각화는 결과를 바꾸지 않는다."
    if multi:
        ms = [METHOD_LABELS.get(m, m) for m in METHOD_ORDER if (m, multi) in kinds]
        replay_note = (f"{TARGET_DESC[multi][0]} {TARGET_DESC[multi][1]}은 {'·'.join(ms)} 장면을 모두 넣어, 같은 대상에서 "
                       f"결과가 어떻게 갈리는지 비교할 수 있다. 물리 엔진이 계산한 궤적을 그대로 재생하며, 시각화는 결과를 바꾸지 않는다.")

    # 팀 소개. 마크 그림이 있으면 왼쪽에 붙인다
    emblem = next((f"assets/{n}" for n in EMBLEM_NAMES if os.path.exists(os.path.join(out_dir, "assets", n))), None) if out_dir else None
    has_qr = bool(out_dir) and os.path.exists(os.path.join(out_dir, "assets", QR_NAME))
    qr_img = f'<img src="assets/{QR_NAME}" alt="이 사이트 주소 QR 코드" width="112" height="112">' if has_qr else ""
    team = (f'<div class="team">' + (f'<img src="{emblem}" alt="공군사관학교 마크">' if emblem else "") +
            f'<div><b>팀 {html.escape(TEAM["name"])}</b><span>{" · ".join(html.escape(m) for m in TEAM["members"])}</span></div></div>')

    facts = ""
    total = 0
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
        <div><b>{spins[0]:g}–{spins[-1]:g} deg/s</b><span>회전 속도</span></div>
      </div>"""
    desc = (f"로봇팔·그물·작살의 회전 우주 파편 포획을 {total:,}회 물리 시뮬레이션으로 비교한 결과와 3D 리플레이"
            if total else "로봇팔·그물·작살의 회전 우주 파편 포획을 물리 시뮬레이션으로 비교한 3D 리플레이")
    og_image = thumbs.get(hero["name"]) if hero else None

    return f"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>회전 우주 파편의 포획 방식 비교</title>
<meta name="description" content="{html.escape(desc)}">
{head_meta("회전 우주 파편의 포획 방식 비교", desc, site_url, site_url + og_image if og_image else None)}<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap" media="print" onload="this.media='all'">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/orioncactus/pretendard@v1.3.9/dist/web/variable/pretendardvariable-dynamic-subset.min.css" media="print" onload="this.media='all'">
<style>{INDEX_CSS}</style>
</head>
<body>
  <canvas class="sky" aria-hidden="true"></canvas>
  <div class="topbar wrap">
    <a class="brand" href="#"><i></i>회전 우주 파편 포획 시뮬레이션</a>
    <nav>{'<a href="#results">핵심 결과</a>' if curves else ''}{'<a href="#costs">포획의 대가</a>' if costs else ''}{'<a href="#conclusion">결론</a>' if curves else ''}<a href="#replays">3D 리플레이</a><a href="#method">조건·한계</a><a href="#guide">보는 법</a></nav>
  </div>

  <header class="hero wrap{'' if hero_shot else ' solo'}">
    <div>
      <div class="eyebrow">2026 공군사관학교 미래 항공우주 학술대회</div>
      <h1>회전하는 우주 파편의<br><span class="hl">포획 방식 비교</span></h1>
      <p class="subtitle">로봇팔 · 그물 · 작살의 물리 시뮬레이션 기반 정량 평가</p>
      {team}
      <p class="lede">지구 저궤도 파편은 대부분 회전하고 있어, 제거하려면 회전체를 안전하게 붙잡아야 한다.
      PyBullet 다물체 동역학에 상대 궤도 역학을 더해 세 방식을 같은 조건에서 비교했고, 대상과 서비스 위성의
      상대 각속도가 {fx['threshold']:g} deg/s 이하로 잦아들어 {fx['hold']:g} 초 이상 유지되면 성공으로 셌다.</p>{facts}
      <div class="cta">{'<a class="btn primary" href="#results">핵심 결과 보기</a>' if curves else ''}<a class="btn{'' if curves else ' primary'}" href="#replays">3D 리플레이</a></div>
    </div>{hero_shot}
  </header>
{build_results(curves, axis) if curves else ''}{build_costs(curves, costs) if costs else ''}{build_conclusions(curves, costs, axis, runs) if curves else ''}
  <section id="replays" class="wrap">
    <div class="sec-head">
      <div class="eyebrow">3D 리플레이</div>
      <h2>포획 순간 다시 보기</h2>
      <p>{replay_note}</p>
    </div>
    <div class="grid4">
{"".join(cards)}
    </div>
  </section>
{build_method()}
  <section id="guide" class="wrap">
    <div class="guide">
      <div>
        <h3>보는 법</h3>
        마우스 드래그로 회전, 휠로 확대·축소, <kbd>Shift</kbd> 드래그로 이동한다.
        <kbd>Space</kbd> 재생·정지, <kbd>1</kbd>~<kbd>4</kbd> 카메라 추적 대상,
        <kbd>P</kbd> 발표 모드(핵심 수치만 크게), <kbd>O</kbd> 카메라 자동 회전, <kbd>N</kbd> 그물 질점 표시.
        질점은 실제 충돌 반경으로 그려 그물이 솜뭉치처럼 보이므로, 발표 모드에서는 자동으로 숨겨 그물 선만 남긴다.
        발표 모드는 다른 시나리오로 옮겨도 유지되고, 전체화면은 <kbd>F11</kbd> 로 켜야 옮겨도 풀리지 않는다.
      </div>
      <div>
        <h3>성공 판정</h3>
        대상과 서비스 위성의 상대 각속도가 {fx['threshold']:g} deg/s 이하로 떨어지고 그 상태가
        {fx['hold']:g} 초 이상 유지되며 결합이 끊기지 않으면 성공으로 본다.
        뷰어 그래프의 가로 점선이 이 기준이고, 초록 음영은 결합이 유지된 구간이다.
        감속 시간은 첫 접촉부터 상대 각속도가 기준 아래로 내려갈 때까지 걸린 시간이다.
      </div>
      <div>
        <h3>휴대폰으로 보기</h3>
        <div class="qr">{qr_img}<div>
        한 손가락으로 끌면 장면이 돌아가고, 두 손가락을 오므리거나 벌리면 축소·확대된다.
        휴대폰을 가로로 돌리면 장면이 넓게 보인다. 리플레이 하나는 1~4 MB 라 처음 열 때 잠시 걸릴 수 있다.</div></div>
      </div>
    </div>
  </section>

  <footer class="wrap">팀 {html.escape(TEAM['name'])} ({" · ".join(html.escape(m) for m in TEAM['members'])}) · 2026 공군사관학교 미래 항공우주 학술대회 · 고도 {fx['altitude']:.0f} km 원궤도 기준 · 적분 간격 1/{round(1 / fx['timestep'])} s</footer>
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
                # nodes=0.3: 그물·테더 질점을 줄여 그물 모양이 보이게 한다 (논문 그림과 같은 방식)
                url = f"http://127.0.0.1:{port}/runs/{r['name']}.html?clean&nodes=0.3&t={thumb_time(r['data']):.2f}"
                # 임시 폴더에 먼저 찍고, 성공했을 때만 기존 썸네일(jpg·png)을 바꾼다. 실패하면 이전 것을 남기고 알린다
                png = os.path.join(profile, r["name"] + ".png")
                try:
                    subprocess.run([browser, "--headless=new", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
                                    "--hide-scrollbars", f"--window-size={size[0]},{size[1]}", "--virtual-time-budget=8000",
                                    f"--user-data-dir={os.path.join(profile, 'browser')}", f"--screenshot={png}", url],
                                   capture_output=True, timeout=180)
                except subprocess.TimeoutExpired:
                    pass
                if not os.path.exists(png):
                    print(f"  썸네일 실패: {r['name']} (이전 썸네일이 있으면 그대로 쓴다)")
                    continue
                for ext in ("jpg", "png"):
                    old = os.path.join(thumbs_dir, f"{r['name']}.{ext}")
                    if os.path.exists(old):
                        os.remove(old)
                dst = os.path.join(thumbs_dir, r["name"] + ".png")
                shutil.move(png, dst)
                print(f"  썸네일 {r['name']:30s} -> {os.path.basename(to_jpeg(dst))}")
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
def build(inputs: List[str], out_dir: str, summary: Optional[str] = None, thumbs: bool = False,
          standalone: Optional[str] = None, site_url: str = SITE_URL) -> List[Dict]:
    here = os.path.dirname(os.path.abspath(__file__))
    template = open(os.path.join(here, "viewer_template.html"), encoding="utf-8").read()
    check_template(template)
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
    for n in EMBLEM_NAMES + [QR_NAME]:           # 공군사관학교 마크와 사이트 QR 코드 (있을 때만)
        if os.path.exists(os.path.join(here, "assets", n)):
            shutil.copyfile(os.path.join(here, "assets", n), os.path.join(out_dir, "assets", n))

    runs, used = [], set()
    for path in inputs:
        data = load_trajectory(path)
        name = run_name(data, os.path.splitext(os.path.basename(path))[0])
        # 이름은 방식·대상·회전 속도로만 짓는다(기존 주소를 지키려고). 시드나 회전 방식만 다른 시행이 겹치면 뒤에 붙여 구분한다
        if name in used:
            res = data.get("result", {})
            base, name = name, f"{name}_s{res.get('seed', 0)}" + ("" if res.get("spin_mode", "principal") == "principal" else f"_{res['spin_mode']}")
            k = 2
            while name in used:
                name, k = f"{base}_{k}", k + 1
            print(f"  이름 겹침: {os.path.basename(path)} 은 {name} 로 쓴다")
        used.add(name)
        payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
        with open(os.path.join(out_dir, "data", f"{name}.js"), "w", encoding="utf-8") as f:
            f.write("window.__DEBRIS_DATA__=" + payload + ";\n")
        runs.append({"name": name, "data": data, "source": path})
        print(f"  {name:32s} <- {os.path.basename(path)}")

    # 대상 → 방식 순. 같은 대상의 방식들이 나란히 놓여 결과가 갈리는 것을 바로 비교할 수 있다
    order = lambda seq, x: seq.index(x) if x in seq else 9
    runs.sort(key=lambda r: (order(TARGET_ORDER, run_kind(r)[1]), order(METHOD_ORDER, run_kind(r)[0]), r["name"]))

    # 뷰어 위쪽의 시나리오 이동 막대. 같은 방식·대상이 둘 이상이면 회전 속도로 구분한다
    kinds = [run_kind(r) for r in runs]
    nav_runs = []
    for r, (m, t) in zip(runs, kinds):
        label = f"{METHOD_LABELS.get(m, m)} · {TARGET_DESC.get(t, (t, ''))[0]}"
        if kinds.count((m, t)) > 1:
            label += f" {float(r['data'].get('result', {}).get('spin_deg_s', 0)):g} deg/s"
        nav_runs.append({"name": r["name"], "href": f"{r['name']}.html", "label": label,
                         "color": METHOD_COLORS.get(m, "#5fc9ff")})
    titles = {r["name"]: f"{METHOD_LABELS.get(m, m)} 포획 · {TARGET_LABELS.get(t, t)} · 3D 리플레이"
              for r, (m, t) in zip(runs, kinds)}
    nav_for = lambda name, home: {"home": home, "runs": [dict(n, current=n["name"] == name) for n in nav_runs]}

    def write_viewers(found: Dict[str, str]) -> None:
        for r in runs:
            name, res = r["name"], r["data"].get("result", {})
            desc = (f"회전 {float(res.get('spin_deg_s', 0)):g} deg/s · "
                    + ("포획 성공" if res.get("success") else "포획 실패") + " · 물리 시뮬레이션 3D 리플레이")
            head = head_meta(titles[name], desc, f"{site_url}runs/{name}.html",
                             site_url + found[name] if name in found else None)
            with open(os.path.join(out_dir, "runs", f"{name}.html"), "w", encoding="utf-8") as f:
                f.write(build_viewer_page(template, f"../data/{name}.js", nav_for(name, "../index.html"), titles[name], head))

    write_viewers(find_thumbs(out_dir, runs))
    if thumbs:
        capture_thumbs(out_dir, runs)               # 방금 쓴 뷰어 페이지를 열어 찍는다
        write_viewers(find_thumbs(out_dir, runs))   # 썸네일이 생겼으니 링크 미리보기 그림을 넣어 다시 쓴다
    found = find_thumbs(out_dir, runs)
    summary_path = summary or os.path.join(here, "results", "results_summary.csv")
    curves = load_summary(summary_path)
    axis = load_axis(os.path.join(here, "results_axis"))
    costs = load_costs(summary_path, os.path.join(os.path.dirname(summary_path), "results_raw.csv"))
    with open(os.path.join(out_dir, "index.html"), "w", encoding="utf-8") as f:
        f.write(build_index(runs, curves, found, axis, site_url, out_dir, costs))

    # 오프라인 발표 대비용 단독 HTML. 첫 화면이 없으므로 이동 막대에는 시나리오끼리만 잇는다
    if standalone:
        os.makedirs(standalone, exist_ok=True)
        three_js = open(os.path.join(here, three), encoding="utf-8").read().replace("</script", "<\\/script")
        with open(os.path.join(here, earth), "rb") as f:
            earth_uri = "data:image/jpeg;base64," + base64.b64encode(f.read()).decode("ascii")
        for r in runs:
            path = os.path.join(standalone, f"{r['name']}.html")
            with open(path, "w", encoding="utf-8") as f:
                f.write(build_standalone_page(template, r["data"], three_js, earth_uri, nav_for(r["name"], None),
                                              titles[r["name"]]))
            print(f"  오프라인 {path} ({os.path.getsize(path) / 1e6:.1f} MB)")
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
    ap.add_argument("--standalone", default=None, metavar="DIR",
                    help="인터넷 없이 열리는 단독 뷰어 HTML 을 DIR 에 쓴다 (예: demo). 입력과 같은 폴더여도 된다")
    ap.add_argument("--site-url", default=SITE_URL, help="배포 주소. 링크 미리보기 그림의 절대 주소에 쓴다")
    ap.add_argument("--serve", action="store_true", help="빌드 후 로컬 서버로 확인")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args(argv)

    inputs = args.inputs or sorted(glob.glob("demo/*.html"))
    if not inputs:
        raise SystemExit("입력이 없다. `--viewer` 로 리플레이를 먼저 만들거나 --inputs 로 지정할 것.")
    print(f"입력 {len(inputs)}개")
    runs = build(inputs, args.out, args.summary, args.thumbs, args.standalone,
                 args.site_url if args.site_url.endswith("/") else args.site_url + "/")
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
