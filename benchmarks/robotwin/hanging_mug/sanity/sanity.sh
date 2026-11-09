#!/usr/bin/env bash
# Sanity checks (b)-(d) for SANITY.md. Run from the repo root with benchmarks/robotwin/env.sh sourced.
# Uses a scratch copy of runs/hanging_mug/gate_*.jsonl and restores them afterwards.
set -u
B=benchmarks/robotwin; R=$B/runs/hanging_mug; mkdir -p $R
for f in gate_history.jsonl gate_log.jsonl; do [ -f $R/$f ] && mv $R/$f $R/$f.sanity_bak; done
echo "== (b) no-op variation"; python $B/tools/gate.py --task hanging_mug --variation $B/sanity/noop.yaml; echo "exit $?"
cp $B/sanity/history_two_tuning.jsonl $R/gate_history.jsonl
echo "== (c) third parameter-tuning variation in a row"; python $B/tools/gate.py --task hanging_mug --variation $B/sanity/tune3.yaml; echo "exit $?"
SHADOW=$(mktemp -d /var/tmp/gah_rt_shadow.XXXX)
for e in "$ROBOTWIN_ROOT"/*; do [ "$(basename $e)" = envs ] || ln -s "$e" "$SHADOW/"; done
cp -r "$ROBOTWIN_ROOT/envs" "$SHADOW/envs"
sed -i 's/eps = 0.02/eps = 0.05/' "$SHADOW/envs/hanging_mug.py"
echo "== (d) success check modified (eps 0.02 -> 0.05 in a shadow RoboTwin tree)"; ROBOTWIN_ROOT=$SHADOW python $B/tools/gate.py --task hanging_mug --variation $B/sanity/tune3.yaml; echo "exit $?"
echo "== (d') a protected tool file modified"; echo "# tampered" >> $B/tools/oracle.py; python $B/tools/gate.py --task hanging_mug --variation $B/sanity/tune3.yaml; echo "exit $?"; sed -i '$ d' $B/tools/oracle.py
python $B/tools/integrity.py
rm -rf "$SHADOW" $R/gate_history.jsonl $R/gate_log.jsonl $B/sanity/*.gate.json
for f in gate_history.jsonl gate_log.jsonl; do [ -f $R/$f.sanity_bak ] && mv $R/$f.sanity_bak $R/$f; done
true
