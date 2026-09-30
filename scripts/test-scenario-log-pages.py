"""Accept automatic logs on built scenario pages and their real WASM artifact.

Run after build-pages.sh with a local server. Uses the existing Playwright setup.
Controlled export traps verify reporting boundaries; the event-reference direct-hit
case also exercises a real returned engine error. This is not physics replay or
performance evidence. BFCache navigation records whether Chromium retained the
page; restoration through session storage is checked when it did not.
"""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright

INSTRUMENT = """(() => {
  window.logProbe = { name: null, remaining: 0, completed: 0, calls: 0 };
  window.logPageShows = [];
  addEventListener('pageshow', e => logPageShows.push(e.persisted));
  function wrap(result) {
    const exports = Object.fromEntries(Object.entries(result.instance.exports).map(([name, value]) => {
      if (typeof value !== 'function') return [name, value];
      return [name, (...args) => {
        logProbe.calls++;
        if (name === logProbe.name) {
          if (logProbe.remaining === 0) throw new WebAssembly.RuntimeError('boundary trap: ' + name);
          logProbe.remaining--;
          const answer = value(...args);
          logProbe.completed++;
          return answer;
        }
        return value(...args);
      }];
    }));
    return { module: result.module, instance: { exports } };
  }
  for (const name of ['instantiate', 'instantiateStreaming']) {
    const original = WebAssembly[name];
    WebAssembly[name] = async (...args) => wrap(await original(...args));
  }
})();"""


def log_state(page):
    return page.evaluate("""async () => {
      const {scenarioLog} = await import('../../scenario-log.mjs');
      return { active: scenarioLog.recorder.active, failed: scenarioLog.failed,
        frames: (scenarioLog.failed ? scenarioLog.lastCrash : scenarioLog.recorder.capture()).summary.recorded_frames };
    }""")


def arm(page, name, completed=0):
    page.evaluate("""({name,completed}) => {
      logProbe.name = name; logProbe.remaining = completed; logProbe.completed = 0;
    }""", {'name': name, 'completed': completed})


def download_report(page, output, action):
    with page.expect_download() as event:
        action()
    page.wait_for_function('window.physicsCrashLog?.outcome === "crashed"')
    event.value.save_as(str(output))
    report = json.loads(output.read_text())
    assert report == page.evaluate('physicsCrashLog')
    assert report['outcome'] == 'crashed'
    assert report['failure']['stack']
    assert not log_state(page)['active']
    return report


def reset_page(page):
    close_settings = page.locator('#close-settings')
    if close_settings.count() and close_settings.is_visible():
        close_settings.click()
    page.locator('#reset').click()


def stopped(page):
    calls = page.evaluate('logProbe.calls')
    page.evaluate('new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)))')
    assert page.evaluate('logProbe.calls') == calls, 'failed page must stop engine/render calls'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--output', default='scenario-log-pages-evidence')
    parser.add_argument('--chromium', default=None)
    parser.add_argument('--case', choices=['startup', 'projectile', 'reset', 'engine', 'initialization', 'navigation'])
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    base = args.url.rstrip('/')
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=args.chromium, headless=True,
            ignore_default_args=['--disable-back-forward-cache'],
            args=['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
        context = browser.new_context(accept_downloads=True, viewport={'width': 1440, 'height': 960})
        context.add_init_script(INSTRUMENT)

        def open_page(scenario, query=''):
            page = context.new_page()
            page.goto(f'{base}/scenarios/{scenario}/{query}', wait_until='networkidle')
            page.wait_for_function("document.querySelector('#status').textContent.match(/ready| s simulated|Unable/i)")
            assert not log_state(page)['failed'], page.locator('#status').inner_text()
            return page

        def passed(case, **detail):
            results.append({'case': case, 'passed': True, **detail})
            print(f'{case}: passed', flush=True)

        if args.case in [None, 'startup']:
            for scenario in ['sandbox', 'tower', 'parkour', 'fixed-step']:
                page = open_page(scenario)
                assert log_state(page)['active']
                page.wait_for_function("""async () => (await import('../../scenario-log.mjs')).scenarioLog.recorder.capture().summary.recorded_frames > 0""")
                with page.expect_download() as current:
                    page.locator('#download-performance-log').click()
                current.value.save_as(str(output / f'{scenario}-current.json'))
                session = json.loads((output / f'{scenario}-current.json').read_text())
                assert session['scenario']['id'] == scenario
                assert session['summary']['recorded_frames'] > 0
                assert log_state(page)['active'], 'manual snapshot must not stop automatic capture'
                before = page.evaluate("sessionStorage.getItem('physics-engine:last-crash')")
                reset_page(page)
                assert log_state(page)['active'] and not log_state(page)['failed']
                assert page.evaluate("sessionStorage.getItem('physics-engine:last-crash')") == before
                page.screenshot(path=str(output / f'{scenario}-ready.png'))
                passed(f'{scenario}-startup-export-reset')
                page.close()

        if args.case in [None, 'projectile']:
            page = open_page('tower')
            arm(page, 'approximate_set_projectile_type')
            report = download_report(page, output / 'projectile-trap.json',
                lambda: page.locator('#projectile-type').select_option('rigid'))
            operation = report['failure']['operation']
            assert operation is not None, 'projectile trap lost the attempted operation'
            assert operation['name'] == 'projectile-configuration'
            assert operation['detail']['projectile_type'] == 'rigid'
            assert operation['detail']['arguments'] == [2]
            stopped(page)
            passed('projectile-selection-trap')
            page.close()

        if args.case in [None, 'reset']:
            page = open_page('fixed-step')
            arm(page, 'approximate_step_velocity', completed=7)
            report = download_report(page, output / 'reset-trap.json', lambda: reset_page(page))
            assert page.evaluate('logProbe.completed') == 7
            failure = report['failure']
            assert failure['operation']['name'] == 'physics-step'
            assert failure['operation']['detail']['step_index'] == 8
            assert len(failure.get('pending_physics_steps_ms', [])) == 7, 'reset trap lost completed warm-up calls'
            assert report['summary']['recorded_frames'] == 0, 'unfinished reset is not a completed frame'
            stopped(page)
            page.evaluate('logProbe.name = null')
            reset_page(page)
            assert log_state(page)['active'] and not log_state(page)['failed']
            assert page.evaluate('physicsLabState.ticks') == 240
            passed('manual-reset-warmup-trap-and-retry')
            page.close()

        if args.case in [None, 'engine']:
            page = open_page('fixed-step', '?solver=event&crates=free&projectile=arrow')
            report = download_report(page, output / 'engine-returned-error.json', lambda: page.locator('#hit').click())
            failure = report['failure']
            assert failure['source'] == 'engine'
            assert (failure['engine_code'], failure['engine_detail']) == (6, 611)
            assert failure['operation']['name'] == 'physics-step'
            assert any(marker['name'] == 'shoot' for marker in report['raw']['markers'])
            assert failure['pending_physics_steps_ms']
            stopped(page)
            reset_page(page)
            assert not log_state(page)['failed'] and log_state(page)['active']
            assert page.evaluate('physicsLabState.error') is None
            passed('real-reference-engine-error-and-reset', engine_code=6, engine_detail=611)
            page.close()

        if args.case in [None, 'initialization']:
            for scenario in ['tower', 'fixed-step']:
                page = context.new_page()
                page.route('**/physics_engine_demo.wasm', lambda route: route.abort())
                report = download_report(page, output / f'{scenario}-initialization.json',
                    lambda: page.goto(f'{base}/scenarios/{scenario}/', wait_until='networkidle'))
                assert report['failure']['operation']['name'] == 'initialization'
                assert report['summary']['recorded_frames'] == 0
                stopped(page)
                passed(f'{scenario}-actual-initialization-failure')
                page.close()

        if args.case in [None, 'navigation']:
            page = open_page('tower')
            downloads = []
            page.on('download', lambda event: downloads.append(event))
            report = download_report(page, output / 'navigation-crash.json',
                lambda: page.evaluate("setTimeout(() => {throw new Error('navigation crash fixture')}, 0)"))
            assert len(downloads) == 1
            page.reload(wait_until='networkidle')
            page.wait_for_function('window.physicsCrashLog?.failure?.message === "navigation crash fixture"')
            assert len(downloads) == 1, 'reload must not repeat the automatic download'
            assert page.evaluate('physicsCrashLog') == report
            assert log_state(page)['active'] and not log_state(page)['failed']
            page.goto(base + '/', wait_until='networkidle')
            page.go_back(wait_until='networkidle')
            page.wait_for_function('window.physicsCrashLog?.failure?.message === "navigation crash fixture"')
            restored_from_bfcache = page.evaluate('logPageShows.at(-1)')
            close_settings = page.locator('#close-settings')
            if close_settings.count() and close_settings.is_visible():
                close_settings.click()
            with page.expect_download() as manual:
                page.get_by_role('link', name='Download crash log (JSON)').click()
            manual.value.save_as(str(output / 'navigation-manual.json'))
            assert json.loads((output / 'navigation-manual.json').read_text()) == report
            assert len(downloads) == 2
            passed('reload-and-real-history-navigation', bfcache_restored=restored_from_bfcache)
            page.close()
        context.close()
        browser.close()
    (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(f'Full scenario-page logs: {len(results)} cases passed.')


if __name__ == '__main__':
    main()
