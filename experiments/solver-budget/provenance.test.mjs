import test from 'node:test';
import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {mkdtempSync,writeFileSync,rmSync} from 'node:fs';
import {join} from 'node:path';
import {tmpdir} from 'node:os';
import {sourceIdentity} from './provenance.mjs';
test('source identity records the actual checkout, dirty files and missing metadata',()=>{
 const dir=mkdtempSync(join(tmpdir(),'physics-budget-identity-'));
 try {
  assert.deepEqual(sourceIdentity(dir),{revision:null,tree:null,dirty:null});
  const git=(...args)=>execFileSync('git',args,{cwd:dir,stdio:'pipe',encoding:'utf8'}).trim();
  git('init','--quiet');git('config','user.name','Physics test');git('config','user.email','physics@example.invalid');
  writeFileSync(join(dir,'fixture'),'test');git('add','fixture');git('commit','--quiet','-m','fixture');
  assert.deepEqual(sourceIdentity(dir),{revision:git('rev-parse','HEAD'),tree:git('rev-parse','HEAD^{tree}'),dirty:false});
  writeFileSync(join(dir,'fixture'),'changed');assert.equal(sourceIdentity(dir).dirty,true);
  git('checkout','--','fixture');writeFileSync(join(dir,'new'),'new');assert.equal(sourceIdentity(dir).dirty,true);
 } finally {rmSync(dir,{recursive:true,force:true});}
});
test('default source root resolves this repository when git metadata is available',()=>{
 const identity=sourceIdentity();
 // Source archives intentionally have no git identity; never fabricate an old commit.
 if(identity.revision!==null){assert.match(identity.revision,/^[a-f0-9]{40}$/);assert.match(identity.tree,/^[a-f0-9]{40}$/);}
});
