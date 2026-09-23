"""Exercise the canonical tower page with real UI inputs and its actual WASM artifact.
Run after build-pages.sh with a local server. Requires playwright and chromium.
Use real animation/timer progress for locator actionability and held-input durations.
Readiness and state transitions use predicates. This is behavioral acceptance, not a
deterministic replay or a physics timing benchmark; those run through the WASM harness.
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
        page.goto(args.url.rstrip('/') + '/scenarios/tower/?response=physical&crates=free&projectile-type=arrow&projectile-impact=impact-retire', wait_until='networkidle')
        page.wait_for_function("document.querySelector('#status').textContent.includes('Tower ready') || document.querySelector('#status').textContent.includes('Unable')")
        assert 'Tower ready' in page.locator('#status').inner_text()
        page.wait_for_timeout(4000)
        def paint():
            page.evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))')

        def click(selector):
            # Real frames keep Playwright's visibility/obstruction/stability checks alive.
            # Do not force clicks or disable checks to compensate for a paused fake clock.
            page.locator(selector).click()
            paint()

        state = lambda: page.evaluate('window.physicsTowerState')
        page.wait_for_function('window.physicsTowerState && !window.physicsTowerState.error && window.physicsTowerState.awake === 0')
        before = state()
        assert before['solver'] == 'fixed-step-f64' and not before['error'] and before['awake'] == 0, before
        click('#close-settings')
        assert page.evaluate('document.activeElement.id') == 'scene'
        page.wait_for_timeout(32)
        page.screenshot(path=str(out / 'tower-ready.png'), full_page=True)
        # Change type with a shot still in flight: no fixture reset and per-shot shapes survive.
        click('#pause')
        page.keyboard.press('f')
        page.keyboard.press('1')
        page.keyboard.press('f')
        page.wait_for_timeout(16)
        page.wait_for_function('window.physicsTowerState.roles.includes(3) && window.physicsTowerState.roles.includes(4)')
        live = state()
        assert live['ticks'] >= before['ticks'] and live['projectileCount'] >= 1, live
        assert 3 in live['roles'] and 4 in live['roles'], live
        click('#reset'); page.wait_for_timeout(4000)
        # Non-overlapping fast mixed volley: callbacks/time continue through every impact.
        for i in range(18):
            page.keyboard.press(str(i % 3 + 1))
            page.keyboard.press('f')
            page.wait_for_timeout(100)
            assert not state()['error'], state()
        page.wait_for_timeout(5000)
        hit = state()
        assert hit['ticks'] > before['ticks'] and not hit['error'], hit
        assert hit['cratePoses'] != before['cratePoses'], 'projectiles must visibly disturb crates'
        assert hit['bodyCount'] - hit['projectileCount'] == 44, hit
        page.screenshot(path=str(out / 'tower-after-projectiles.png'), full_page=True)
        # Reset works after impacts. Aimed near miss does not wake the tower.
        click('#reset')
        page.wait_for_timeout(4000)
        reset = state()
        page.keyboard.down('ArrowRight')
        page.wait_for_timeout(320)
        page.keyboard.up('ArrowRight')
        for i in range(3):
            page.keyboard.press(str(i + 1)); page.keyboard.press('f'); page.wait_for_timeout(300)
        page.wait_for_timeout(2000)
        miss = state()
        assert miss['cratePoses'] == reset['cratePoses'] and miss['awake'] == 0 and not miss['error'], miss
        # Existing first-person movement/jump and pause/single-step controls remain usable.
        click('#reset'); page.wait_for_timeout(4000)
        position = state()['playerPose']
        page.keyboard.down('d'); page.wait_for_timeout(250); page.keyboard.up('d')
        assert state()['playerPose'][0] > position[0], 'first-person movement'
        page.keyboard.press('Space'); page.wait_for_timeout(200)
        assert state()['playerPose'][1] > position[1], 'jump'
        click('#pause'); page.wait_for_timeout(32)
        page.wait_for_function('window.physicsTowerState.paused')
        paused = state(); assert paused['paused']
        page.wait_for_timeout(500); assert state()['ticks'] == paused['ticks']
        click('#single-step'); page.wait_for_timeout(32)
        page.wait_for_function('(tick) => window.physicsTowerState.ticks === tick', arg=paused['ticks'] + 1)
        assert state()['ticks'] == paused['ticks'] + 1
        click('#reset'); page.wait_for_timeout(4000)
        assert state()['projectileCount'] == 0 and not state()['error']
        # Unsupported event-solver settings must not masquerade as functional controls.
        click('#open-settings')
        assert page.locator('#fixed-geometry-setting').is_disabled()
        assert all(x.is_disabled() for x in page.locator('[data-stabilization-pair]').all())
        click('#start-performance-log')
        click('#close-settings'); page.keyboard.press('f'); page.wait_for_timeout(500)
        click('#open-settings')
        with page.expect_download() as download:
            click('#download-performance-log')
        target = out / 'browser-session.json'; download.value.save_as(target)
        session = json.loads(target.read_text())
        assert session['scenario']['solver'] == 'fixed-step-f64'
        assert session['scenario']['character_response'] == 'physical'
        assert session['scenario']['crate_motion'] == 'free'
        assert session['summary']['physics_work']['fixed_substeps'] > 0
        click('#close-settings')
        # Presentation uses the same real animation clock as input actionability.
        page.set_viewport_size({'width': 390, 'height': 844})
        page.evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))')
        page.wait_for_function('document.querySelector("#settings-panel").getBoundingClientRect().left >= window.innerWidth')
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
        page.screenshot(path=str(out / 'tower-mobile.png'), full_page=True)
        assert not state()['error'], state()
        assert not errors, errors
        result = {'passed': True, 'before': before, 'hit': hit, 'miss': miss, 'mixed_live': live,
                  'page_errors': errors, 'url': page.url, 'note': 'Real-clock, real-input acceptance; no mocked timers or forced clicks. Not a physics benchmark or deterministic browser replay.'}
        (out / 'browser-result.json').write_text(json.dumps(result, indent=2))
        browser.close()


if __name__ == '__main__':
    main()
