"""Exercise the real crash logger's browser boundary without network or a physics fixture.

Requires Playwright and Chromium, as does test-tower-browser.py. This isolates Blob
URL/download ownership and global error delivery; it is not full scenario-page
acceptance or a claim that terminating a browser process is catchable.
"""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--chromium', default=None)
    parser.add_argument('--output', default='scenario-log-browser-evidence')
    args = parser.parse_args()
    site = Path(__file__).resolve().parents[1] / 'site'
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    sources = {name: (site / name).read_text() for name in ['performance-log.mjs', 'scenario-log.mjs', 'bootstrap.mjs']}
    results = []
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=args.chromium, headless=True,
            args=['--no-sandbox'])
        try:
            for mode in ['engine', 'trap', 'initialization', 'fixed-initialization', 'rejection', 'download-blocked']:
                page = browser.new_page(accept_downloads=True)
                scenario = 'fixed-step' if mode == 'fixed-initialization' else 'tower'
                page.set_content(f'<body data-scenario="{scenario}"><p id="status">Ready</p></body>')
                # Supply the actual modules in memory. No replacement of recorder behavior,
                # DOM, Blob URLs, error events or browser downloads is involved.
                page.evaluate('''async sources => {
                    const recorder = URL.createObjectURL(new Blob([sources['performance-log.mjs']], {type:'text/javascript'}));
                    const source = sources['scenario-log.mjs'].replace('./performance-log.mjs', recorder);
                    const logger = URL.createObjectURL(new Blob([source], {type:'text/javascript'}));
                    window.testLog = (await import(logger)).scenarioLog;
                    const rejectedModule = URL.createObjectURL(new Blob(["throw new Error('initialization fixture failed')"], {type:'text/javascript'}));
                    const settings = URL.createObjectURL(new Blob(['export {}'], {type:'text/javascript'}));
                    const bootstrapSource = sources['bootstrap.mjs'].replace('./scenario-log.mjs', logger)
                        .replace('./app.js', rejectedModule).replace('./fixed-step-lab.js', rejectedModule)
                        .replace('./physics-settings.mjs', settings);
                    window.testBootstrap = URL.createObjectURL(new Blob([bootstrapSource], {type:'text/javascript'}));
                    window.testModuleUrls = [recorder, logger, settings, rejectedModule, testBootstrap];
                }''', sources)
                assert page.evaluate('testLog.recorder.active')
                downloads = []
                page.on('download', lambda download: downloads.append(download))
                if not mode.endswith('initialization'):
                    page.evaluate('''() => {
                        testLog.recorder.recordFrame({frame_interval_ms:16,callback_ms:1,render_performed:false});
                        testLog.begin('physics-step', {step_index:4,velocity:[0,0],jump:0});
                    }''')
                if mode == 'download-blocked':
                    # Simulate a throwing delivery boundary, not an engine exception.
                    page.evaluate('''() => {
                        window.originalClick = HTMLAnchorElement.prototype.click;
                        HTMLAnchorElement.prototype.click = () => {throw new Error('delivery blocked')};
                    }''')
                with (page.expect_download() if mode != 'download-blocked' else nullcontext()) as automatic:
                    if mode.endswith('initialization'):
                        page.evaluate('import(testBootstrap)')
                    elif mode == 'rejection':
                        page.evaluate("void Promise.reject(new Error('rejected operation'))")
                    elif mode == 'trap':
                        page.evaluate("setTimeout(() => {throw new WebAssembly.RuntimeError('injected trap')}, 0)")
                    else:
                        page.evaluate("testLog.fail(new Error('injected failure'), {source:'engine',engine_code:6,engine_detail:611})")
                page.wait_for_function('window.physicsCrashLog?.outcome === "crashed"')
                report = page.evaluate('physicsCrashLog')
                assert report['summary']['recorded_frames'] == (0 if mode.endswith('initialization') else 1)
                assert not page.evaluate('testLog.recorder.active')
                if mode.endswith('initialization'):
                    assert report['failure']['source'] == 'module-initialization'
                    assert report['failure']['operation']['name'] == 'initialization'
                    assert report['scenario']['id'] == scenario
                if mode == 'trap':
                    assert report['failure']['source'] == 'window.error'
                    assert report['failure']['name'] == 'RuntimeError'
                if mode == 'rejection':
                    assert report['failure']['source'] == 'unhandledrejection'
                if mode == 'download-blocked':
                    assert not downloads
                    page.evaluate('() => {HTMLAnchorElement.prototype.click = originalClick;}')
                else:
                    assert len(downloads) == 1
                    path = out / f'{mode}.json'
                    automatic.value.save_as(str(path))
                    assert json.loads(path.read_text()) == report
                # The report is downloadable even when automatic delivery fails. BFCache
                # pagehide must not revoke a URL belonging to the retained document.
                page.evaluate("dispatchEvent(new PageTransitionEvent('pagehide', {persisted:true}))")
                with page.expect_download() as manual:
                    page.get_by_role('link', name='Download crash log (JSON)').click()
                manual.value.save_as(str(out / f'{mode}-manual.json'))
                assert json.loads((out / f'{mode}-manual.json').read_text()) == report
                before = len(downloads)
                page.evaluate("testLog.fail(new Error('secondary failure'))")
                assert page.evaluate('physicsCrashLog.failure.message') == report['failure']['message']
                assert len(downloads) == before
                page.evaluate('testLog.start()')
                assert page.evaluate('testLog.recorder.active && !testLog.failed')
                assert page.evaluate('testLog.recorder.capture().summary.recorded_frames') == 0
                assert page.evaluate('physicsCrashLog.failure.message') == report['failure']['message']
                results.append({'case': mode, 'passed': True})
                page.evaluate('testModuleUrls.forEach(url => URL.revokeObjectURL(url))')
                page.close()
        finally:
            browser.close()
    (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(f'Crash-log browser boundary: {len(results)} cases passed.')


if __name__ == '__main__':
    main()
