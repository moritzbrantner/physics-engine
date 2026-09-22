"""Exercise the canonical tower page with real UI inputs and its actual WASM artifact.
Run after build-pages.sh with a local server. Requires playwright and chromium.
The virtual clock advances browser callbacks deterministically; these are NOT timing benchmarks.
"""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--output', default='tower-browser-evidence')
    parser.add_argument('--chromium', default=None)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=args.chromium, headless=True,
            args=['--no-sandbox', '--enable-unsafe-swiftshader', '--use-gl=angle', '--use-angle=swiftshader'])
        page = browser.new_page(viewport={'width': 1440, 'height': 960}, device_scale_factor=1)
        errors = []
        page.on('pageerror', lambda err: errors.append(str(err)))
        page.clock.install()
        page.goto(args.url.rstrip('/') + '/scenarios/tower/?response=physical&crates=free&projectile-type=arrow&projectile-impact=impact-retire', wait_until='networkidle')
        page.wait_for_function("document.querySelector('#status').textContent.includes('Tower ready') || document.querySelector('#status').textContent.includes('Unable')")
        assert 'Tower ready' in page.locator('#status').inner_text()
        page.clock.run_for(4000)
        state = lambda: page.evaluate('window.physicsTowerState')
        before = state()
        assert before['solver'] == 'fixed-step-f64' and not before['error'] and before['awake'] == 0, before
        page.locator('#close-settings').click()
        assert page.evaluate('document.activeElement.id') == 'scene'
        page.clock.run_for(32)
        page.screenshot(path=str(out / 'tower-ready.png'), full_page=True)
        # Change type with a shot still in flight: no fixture reset and per-shot shapes survive.
        page.locator('#pause').click()
        page.keyboard.press('f')
        page.keyboard.press('1')
        page.keyboard.press('f')
        page.clock.run_for(16)
        live = state()
        assert live['ticks'] >= before['ticks'] and live['projectileCount'] >= 1, live
        assert 3 in live['roles'] and 4 in live['roles'], live
        page.locator('#reset').click(); page.clock.run_for(4000)
        # Non-overlapping fast mixed volley: callbacks/time continue through every impact.
        for i in range(18):
            page.keyboard.press(str(i % 3 + 1))
            page.keyboard.press('f')
            page.clock.run_for(100)
            assert not state()['error'], state()
        page.clock.run_for(5000)
        hit = state()
        assert hit['ticks'] > before['ticks'] and not hit['error'], hit
        assert hit['cratePoses'] != before['cratePoses'], 'projectiles must visibly disturb crates'
        assert hit['bodyCount'] - hit['projectileCount'] == 44, hit
        page.screenshot(path=str(out / 'tower-after-projectiles.png'), full_page=True)
        # Reset works after impacts. Aimed near miss does not wake the tower.
        page.locator('#reset').click()
        page.clock.run_for(4000)
        reset = state()
        page.keyboard.down('ArrowRight')
        page.clock.run_for(320)
        page.keyboard.up('ArrowRight')
        for i in range(3):
            page.keyboard.press(str(i + 1)); page.keyboard.press('f'); page.clock.run_for(300)
        page.clock.run_for(2000)
        miss = state()
        assert miss['cratePoses'] == reset['cratePoses'] and miss['awake'] == 0 and not miss['error'], miss
        # Existing first-person movement/jump and pause/single-step controls remain usable.
        page.locator('#reset').click(); page.clock.run_for(4000)
        position = state()['playerPose']
        page.keyboard.down('d'); page.clock.run_for(250); page.keyboard.up('d')
        assert state()['playerPose'][0] > position[0], 'first-person movement'
        page.keyboard.press('Space'); page.clock.run_for(200)
        assert state()['playerPose'][1] > position[1], 'jump'
        page.locator('#pause').click(); page.clock.run_for(32)
        paused = state(); assert paused['paused']
        page.clock.run_for(500); assert state()['ticks'] == paused['ticks']
        page.locator('#single-step').click(); page.clock.run_for(32)
        assert state()['ticks'] == paused['ticks'] + 1
        page.locator('#reset').click(); page.clock.run_for(4000)
        assert state()['projectileCount'] == 0 and not state()['error']
        # Unsupported event-solver settings must not masquerade as functional controls.
        page.locator('#open-settings').click()
        assert page.locator('#fixed-geometry-setting').is_disabled()
        assert all(x.is_disabled() for x in page.locator('[data-stabilization-pair]').all())
        page.locator('#start-performance-log').click()
        page.locator('#close-settings').click(); page.keyboard.press('f'); page.clock.run_for(500)
        page.locator('#open-settings').click()
        with page.expect_download() as download:
            page.locator('#download-performance-log').click()
        target = out / 'browser-session.json'; download.value.save_as(target)
        session = json.loads(target.read_text())
        assert session['scenario']['solver'] == 'fixed-step-f64'
        assert session['scenario']['character_response'] == 'physical'
        assert session['scenario']['crate_motion'] == 'free'
        assert session['summary']['physics_work']['fixed_substeps'] > 0
        page.locator('#close-settings').click()
        # Screenshot stabilization waits for real animation frames after a resize.
        # All deterministic gameplay assertions above are complete; resume browser time
        # for presentation capture rather than leaving its paint callbacks paused.
        page.clock.resume()
        page.set_viewport_size({'width': 390, 'height': 844})
        page.evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))')
        page.wait_for_function('document.querySelector("#settings-panel").getBoundingClientRect().left >= window.innerWidth')
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
        page.screenshot(path=str(out / 'tower-mobile.png'), full_page=True)
        assert not state()['error'], state()
        assert not errors, errors
        result = {'passed': True, 'before': before, 'hit': hit, 'miss': miss, 'mixed_live': live,
                  'page_errors': errors, 'url': page.url, 'note': 'Virtual-clock UI acceptance; not a performance benchmark.'}
        (out / 'browser-result.json').write_text(json.dumps(result, indent=2))
        browser.close()


if __name__ == '__main__':
    main()
