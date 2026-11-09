#!/usr/bin/env bash
# Sanity checks (a)-(d') for the RoboPRO backend. Run from benchmarks/robopro with the env sourced and the pool up.
set -u
T=pick_boxdrink_from_basket; R=runs/$T; mkdir -p $R
for f in gate_history.jsonl gate_log.jsonl; do [ -f $R/$f ] && cp $R/$f sanity/$f.bak; done
echo "== (a) trivial pipeline through eval.py"
$SIM_PY tools/eval.py --task $T --pipeline examples/trivial_pipeline.py --seeds 3 > sanity/a.out 2>/dev/null
python3 -c "import json; d=json.load(open('sanity/a.out')); print({k: d[k] for k in ('success_rate','seeds')}, [ (r['steps'], r['error'], r['trace']) for r in d['per_seed']])"
echo "== (b) no-op variation"
$SIM_PY tools/gate.py --task $T --variation sanity/noop.yaml > sanity/b.out 2>/dev/null; grep -E '"pass"|"reason"' sanity/b.out
echo "== (c) third consecutive parameter-tuning variation"
cp sanity/c_fake_gate_history.jsonl $R/gate_history.jsonl
$SIM_PY tools/gate.py --task $T --variation sanity/tune3.yaml > sanity/c.out 2>/dev/null; grep -E '"pass"|"reason"|tuning_streak' sanity/c.out
rm -f $R/gate_history.jsonl
echo "== (d) success check modified (hash in INTEGRITY.json changed)"
cp tools/INTEGRITY.json sanity/INTEGRITY.backup.json
python3 -c "import json; p='tools/INTEGRITY.json'; d=json.load(open(p)); d['check_success:$T']='0'*64; json.dump(d, open(p,'w'), indent=2)"
$SIM_PY tools/gate.py --task $T --variation sanity/integ.yaml > sanity/d.out 2>/dev/null; grep -E '"pass"|"reason"' sanity/d.out
cp sanity/INTEGRITY.backup.json tools/INTEGRITY.json
echo "== (d') protected backend file modified"
cp gah_env.py sanity/gah_env.backup.py
echo "# sanity: one appended comment line" >> gah_env.py
$SIM_PY tools/gate.py --task $T --variation sanity/integ.yaml > sanity/d2.out 2>/dev/null; grep -E '"pass"|"reason"' sanity/d2.out
cp sanity/gah_env.backup.py gah_env.py
$SIM_PY tools/integrity.py 2>/dev/null
# keep the sanity gate decisions out of the training record
mv $R/gate_log.jsonl sanity/gate_log_sanity.jsonl 2>/dev/null
for f in gate_history.jsonl gate_log.jsonl; do [ -f sanity/$f.bak ] && mv sanity/$f.bak $R/$f; done
rm -f sanity/*.gate.json.tmp
