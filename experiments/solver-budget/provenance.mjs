// Resolve identity once per report. A dirty checkout is never labelled an exact source build.
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
export function sourceIdentity(cwd=fileURLToPath(new URL('../../',import.meta.url))) {
  const git=(...args)=>execFileSync('git',args,{cwd,encoding:'utf8',stdio:['ignore','pipe','ignore']}).trim();
  try { return {revision:git('rev-parse','HEAD'),tree:git('rev-parse','HEAD^{tree}'),
    dirty:git('status','--porcelain','--untracked-files=normal').length>0}; }
  catch { return {revision:null,tree:null,dirty:null}; }
}
