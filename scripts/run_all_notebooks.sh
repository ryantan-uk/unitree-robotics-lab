#!/usr/bin/env bash
# 从头执行全部 (或指定的) Notebook，任何一格报错即失败。结果写到 _out/ (不提交)。
#   bash scripts/run_all_notebooks.sh            全部
#   bash scripts/run_all_notebooks.sh 06 07      只跑编号以 06、07 开头的
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/env.sh" >/dev/null
python3 "$REPO/scripts/connect.py" --offline >/dev/null 2>&1 && source "$REPO/scripts/env.sh" >/dev/null
cd "$REPO/notebooks" || exit 1
mkdir -p "$REPO/_out"
if [ $# -eq 0 ]; then set -- ""; fi
fail=0; n=0
for prefix in "$@"; do
  for nb in ${prefix}*.ipynb; do
    [ -f "$nb" ] || continue
    n=$((n+1)); t0=$(date +%s)
    if jupyter nbconvert --to notebook --execute --ExecutePreprocessor.timeout=900 \
         --output-dir "$REPO/_out" "$nb" >"$REPO/_out/${nb%.ipynb}.log" 2>&1; then
      printf 'PASS  %-40s %4ss\n' "$nb" "$(( $(date +%s) - t0 ))"
    else
      printf 'FAIL  %-40s %4ss   (见 _out/%s.log)\n' "$nb" "$(( $(date +%s) - t0 ))" "${nb%.ipynb}"
      tail -n 25 "$REPO/_out/${nb%.ipynb}.log"; fail=$((fail+1))
    fi
    pkill -f "ros2 bag play" 2>/dev/null; pkill -f "unitree_lab_nodes" 2>/dev/null; sleep 1
  done
done
echo "---- $n 个 Notebook, 失败 $fail 个"
exit $fail
