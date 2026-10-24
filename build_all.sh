#!/usr/bin/env bash
# 编译全部 24 个 C 示例（C17，零警告，警告即错误）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

CC="${CC:-gcc}"
if ! command -v "$CC" >/dev/null 2>&1; then
  for cand in /c/msys64/ucrt64/bin/gcc.exe /ucrt64/bin/gcc.exe /mingw64/bin/gcc.exe; do
    if [ -x "$cand" ]; then
      CC="$cand"
      break
    fi
  done
fi
if ! command -v "$CC" >/dev/null 2>&1; then
  echo "错误：找不到 gcc。请在 MSYS2 UCRT64 终端运行，或设置 CC=/path/to/gcc" >&2
  exit 1
fi
# cc1/ld 依赖同目录 DLL，用绝对路径调用 gcc 时也必须把它加进 PATH
export PATH="$(dirname "$(command -v "$CC")"):$PATH"

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;;
  *) EXE="" ;;
esac

OUT="$ROOT/build/bin"
mkdir -p "$OUT"

count=0
while IFS= read -r -d '' src; do
  name="$(basename "$src" .c)"
  printf '[%2d] %-52s' "$((count + 1))" "$src"
  if "$CC" -std=c17 -O2 -Wall -Wextra -Werror "$src" -o "$OUT/$name$EXE"; then
    echo "OK"
  else
    echo "FAIL"
    exit 1
  fi
  count=$((count + 1))
done < <(find "$ROOT" \
  -path "$ROOT/_ref_tmp" -prune -o \
  -path "$ROOT/.review_out" -prune -o \
  -path "$ROOT/.review_build" -prune -o \
  -path "$ROOT/build" -prune -o \
  -name '*.c' -print0 | sort -z)

echo
echo "编译完成：$count 个程序 -> build/bin/"
