#!/usr/bin/env bash
# 编译并运行全部 C 示例；日志写入 build/logs，任一程序非零退出即失败。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

"$ROOT/build_all.sh"

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;;
  *) EXE="" ;;
esac

LOGS="$ROOT/build/logs"
mkdir -p "$LOGS"

fail=0
count=0
for bin in "$ROOT"/build/bin/*"$EXE"; do
  [ -e "$bin" ] || continue
  name="$(basename "$bin" "$EXE")"
  count=$((count + 1))
  printf '[%2d] %-52s' "$count" "$name"
  if "$bin" > "$LOGS/$name.txt" 2>&1; then
    echo "OK  -> build/logs/$name.txt"
  else
    echo "FAIL（退出码 $?）-> build/logs/$name.txt"
    fail=1
  fi
done

echo
if [ "$fail" -ne 0 ]; then
  echo "验证失败：有程序异常退出，先看 build/logs/ 里对应日志。" >&2
  exit 1
fi
echo "验证完成：$count 个程序全部正常退出。"
