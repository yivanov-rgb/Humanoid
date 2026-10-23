#!/usr/bin/env bash
set -euo pipefail
sim_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$sim_dir/vendor.lock"
target="$sim_dir/vendor/ainex"
if [[ ! -d "$target/.git" ]]; then
    git clone --no-checkout "$AINEX_URL" "$target"
    git -C "$target" checkout --detach "$AINEX_COMMIT"
fi
actual=$(git -C "$target" rev-parse HEAD)
if [[ "$actual" != "$AINEX_COMMIT" ]]; then
    printf 'Vendor commit mismatch: expected %s, found %s\n' "$AINEX_COMMIT" "$actual" >&2
    exit 1
fi
if [[ -n "$(git -C "$target" status --porcelain --untracked-files=no)" ]]; then
    printf 'Vendor has modifications. Keep upstream sources unchanged.\n' >&2
    exit 1
fi
printf 'Hiwonder/ainex: %s\n' "$actual"
