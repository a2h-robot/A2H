#!/usr/bin/env bash
# Sanity checks (a)-(d) for SANITY.md. Run from the repo root with benchmarks/robopro/env.sh sourced and the
# worker pool running (or GAH_LOCAL_WORKERS set). Uses a scratch gate history and restores the real one.
set -u
B=benchmarks/robopro; T=put_spoon_on_plate_ks; R=$B/runs/$T; mkdir -p $R
[ -f $R/gate_history.jsonl ] && mv $R/gate_history.jsonl $R/gate_history.jsonl.sanity_bak
echo "== (a) trivial pipeline end to end"
python $B/tools/eval.py --task $T --pipeline $B/examples/trivial_pipeline.py --seeds 3 --no-frames > $B/sanity/a.json
echo "exit $?"; python -c "import json; d=json.load(open('$B/sanity/a.json')); print('success_rate', d['success_rate'], [(r['seed'], r['success'], r.get('error'), r.get('trace')) for r in d['per_seed']])"
echo "== (b) no-op variation"; python $B/tools/gate.py --task $T --variation $B/sanity/noop.yaml; echo "exit $?"
cp $B/sanity/history_two_tuning.jsonl $R/gate_history.jsonl
echo "== (c) third parameter-tuning variation in a row"; python $B/tools/gate.py --task $T --variation $B/sanity/tune3.yaml; echo "exit $?"
SHADOW=$(mktemp -d "${TMPDIR:-/var/tmp}/gah_rp_shadow.XXXX")
for e in "$BENCH_ROOT"/*; do [ "$(basename "$e")" = bench_envs ] || ln -s "$e" "$SHADOW/"; done
cp -r "$BENCH_ROOT/bench_envs" "$SHADOW/bench_envs"
sed -i 's/eps1 = 0.03/eps1 = 0.05/' "$SHADOW/bench_envs/kitchens/$T.py"
echo "== (d) success check modified (eps1 0.03 -> 0.05 in a shadow benchmark tree)"
BENCH_ROOT=$SHADOW python $B/tools/gate.py --task $T --variation $B/sanity/tune3.yaml; echo "exit $?"
echo "== (d') a protected tool file modified"; echo "# tampered" >> $B/tools/pool.py
python $B/tools/gate.py --task $T --variation $B/sanity/tune3.yaml; echo "exit $?"; sed -i '$ d' $B/tools/pool.py
echo "== integrity after restoring"; python $B/tools/integrity.py; echo "exit $?"
rm -rf "$SHADOW" $R/gate_history.jsonl $B/sanity/*.gate.json
[ -f $R/gate_history.jsonl.sanity_bak ] && mv $R/gate_history.jsonl.sanity_bak $R/gate_history.jsonl
true
