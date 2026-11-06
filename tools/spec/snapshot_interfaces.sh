#!/usr/bin/env bash
# drop_interfaces 정의를 specs/snapshots/에 기록한다(TC-IF-001-01 비교 기준).
#   colcon build 후: source install/setup.bash && tools/spec/snapshot_interfaces.sh
#   --check: 파일을 쓰지 않고 현재 빌드와 스냅샷이 같은지만 검사(다르면 종료 코드 1)
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/specs/snapshots"
CHECK=0; [[ "${1:-}" == "--check" ]] && CHECK=1
command -v ros2 >/dev/null 2>&1 || { echo "ros2 없음: workspace를 source한다" >&2; exit 2; }
LIST="$(ros2 interface package drop_interfaces 2>/dev/null | sort)"
[[ -n "$LIST" ]] || { echo "drop_interfaces가 빌드·source되지 않았다" >&2; exit 2; }
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
for i in $LIST; do
  f="$TMP/$(sed 's#^drop_interfaces/##; s#/#_#' <<<"$i").txt"
  ros2 interface show "$i" >"$f" || exit 2
done
if (( CHECK )); then
  diff -ru --exclude=.gitkeep "$OUT" "$TMP" && echo "스냅샷 일치" || { echo "스냅샷 불일치: SPEC-010 버전을 올리고 스냅샷을 갱신한다" >&2; exit 1; }
else
  mkdir -p "$OUT"; find "$OUT" -name '*.txt' -delete; cp "$TMP"/*.txt "$OUT"/
  echo "스냅샷 $(ls "$OUT"/*.txt | wc -l)개: $OUT"
fi
