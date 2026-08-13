#!/usr/bin/env python3
"""诊断飞行页面在无头下的加载状态(增强版)。"""
import time, json
from playwright.sync_api import sync_playwright

URL = "http://localhost:8080/?autocollect=1&numMaps=1&samples=3&radius=10&maxDepth=20&mode=da360&server=http://localhost:8003"

launch_args = ["--use-gl=angle","--use-angle=swiftshader","--enable-unsafe-swiftshader",
               "--ignore-gpu-blocklist","--enable-webgl","--no-sandbox","--disable-dev-shm-usage"]

with sync_playwright() as pw:
    browser = pw.chromium.launch(headless=True, args=launch_args)
    page = browser.new_page(viewport={"width":1280,"height":800})
    logs = []
    reqfail = []
    page.on("console", lambda m: logs.append(f"[page:{m.type}] {m.text}"))
    page.on("pageerror", lambda e: logs.append(f"[pageerror] {e}"))
    page.on("requestfailed", lambda r: reqfail.append(f"[reqfail] {r.url} :: {r.failure}"))
    page.goto(URL, wait_until="domcontentloaded", timeout=60000)
    time.sleep(40)
    state = page.evaluate("""() => ({
        cesium: typeof window.Cesium,
        readyResolved: !!(window.__googleTilesCesiumReadyResolve === undefined ? 'n/a' : 'set'),
        banner: (document.getElementById('runtime-error-banner')||{}).textContent || '',
        overlayVisible: (document.getElementById('loading-overlay')||{}).className || '',
        cds: window.__cdsStatus || null,
    })""")
    print("=== STATE ===")
    print(json.dumps(state, indent=2, default=str)[:2000])
    print("=== REQ FAIL (last 15) ===")
    for l in reqfail[-15:]:
        print(l)
    print("=== LOGS (last 30) ===")
    for l in logs[-30:]:
        print(l)
    browser.close()
