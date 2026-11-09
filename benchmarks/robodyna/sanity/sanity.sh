#!/usr/bin/env bash
# Sanity checks (b)-(d) for SANITY.md, on place_block_belt. Run from the repo root on <sim-host>
# (benchmarks/robodyna/remote.sh bash benchmarks/robodyna/sanity/sanity.sh).
# Uses a scratch copy of runs/place_block_belt/gate_*.jsonl and restores them afterwards.
set -u
B=benchmarks/robodyna; T=place_block_belt; R=$B/runs/$T; mkdir -p $R
ROOT=${ROBODYNA_ROOT:-$HOME/RoboDyna}
for f in gate_history.jsonl gate_log.jsonl; do [ -f $R/$f ] && mv $R/$f $R/$f.sanity_bak; done
echo "== (b) no-op variation"; python $B/tools/gate.py --task $T --variation $B/sanity/noop.yaml; echo "exit $?"
echo "== (b') unsupported key"; python $B/tools/gate.py --task $T --variation $B/sanity/unsupported.yaml; echo "exit $?"
cp $B/sanity/history_two_tuning.jsonl $R/gate_history.jsonl
echo "== (c) third parameter-tuning variation in a row"; python $B/tools/gate.py --task $T --variation $B/sanity/tune3.yaml; echo "exit $?"
SHADOW=$(mktemp -d /var/tmp/gah_rd_shadow.XXXX)
for e in "$ROOT"/* "$ROOT"/.robodyna_commit; do [ "$(basename $e)" = envs ] || ln -s "$e" "$SHADOW/"; done
cp -r "$ROOT/envs" "$SHADOW/envs"
sed -i 's/            and self.in_bowl$/            and (self.in_bowl or True)/' "$SHADOW/envs/$T.py"
grep -c "in_bowl or True" "$SHADOW/envs/$T.py" | sed 's/^/   shadow edits applied: /'
echo "== (d) success check modified (in_bowl -> always true, in a shadow RoboDyna tree)"; ROBODYNA_ROOT=$SHADOW python $B/tools/gate.py --task $T --variation $B/sanity/tune3.yaml; echo "exit $?"
echo "== (d') a protected tool file modified"; echo "# tampered" >> $B/tools/eval.py; python $B/tools/gate.py --task $T --variation $B/sanity/tune3.yaml; echo "exit $?"; sed -i '$ d' $B/tools/eval.py
python $B/tools/integrity.py
rm -rf "$SHADOW" $R/gate_history.jsonl $R/gate_log.jsonl $B/sanity/*.gate.json
for f in gate_history.jsonl gate_log.jsonl; do [ -f $R/$f.sanity_bak ] && mv $R/$f.sanity_bak $R/$f; done
true
