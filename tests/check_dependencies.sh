#!/bin/sh
set -eu

binary=${1:?Usage: check_dependencies.sh <binary>}

if command -v otool >/dev/null 2>&1; then
    if ! dependencies=$(otool -L "$binary"); then
        echo "Could not inspect dynamic dependencies: $binary" >&2
        exit 1
    fi
    listed=$(printf '%s\n' "$dependencies" | tail -n +2 | awk '{print $1}')
    if [ -z "$listed" ]; then
        echo "No Mach-O dependencies found: $binary" >&2
        exit 1
    fi
    unexpected=$(printf '%s\n' "$listed" |
        grep -Ev '^/usr/lib/libSystem\.B\.dylib$' || true)
    if [ -n "$unexpected" ]; then
        echo "Unexpected dynamic dependencies:" >&2
        echo "$unexpected" >&2
        exit 1
    fi
elif command -v ldd >/dev/null 2>&1; then
    if ! dependencies=$(ldd "$binary"); then
        echo "Could not inspect dynamic dependencies: $binary" >&2
        exit 1
    fi
    unexpected=$(printf '%s\n' "$dependencies" | awk '{print $1}' |
        grep -Ev '^(linux-vdso\.so|libc\.so|libm\.so|/lib.*/ld-)' || true)
    if [ -n "$unexpected" ]; then
        echo "Unexpected dynamic dependencies:" >&2
        echo "$unexpected" >&2
        exit 1
    fi
else
    echo "No supported dynamic dependency inspector found" >&2
    exit 1
fi

echo "Dynamic dependency check passed"
