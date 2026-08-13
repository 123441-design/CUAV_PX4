#!/usr/bin/env bash
#
# Regenerate the vendored third-party dependency tree on the px4/vendored-deps
# branch against a new upstream ref.
#
# Usage:  ./tools/px4/regenerate-vendored-deps.sh <upstream-ref>
#
# The ref can be any argument git rev-parse can resolve: a commit SHA, a branch
# name (including upstream/<branch>), or a tag.
#
# See README.PX4.md for the full workflow.

set -euo pipefail

SCRIPT_NAME=$(basename "$0")
DOWNLOADS_DIR="tensorflow/lite/micro/tools/make/downloads"
BRANCH_NAME="px4/vendored-deps"
UPSTREAM_REMOTE="upstream"
UPSTREAM_URL="https://github.com/tensorflow/tflite-micro.git"

die() {
    echo "error: $*" >&2
    exit 1
}

info() {
    echo "==> $*"
}

if [ $# -ne 1 ]; then
    cat >&2 <<EOF
usage: $SCRIPT_NAME <upstream-ref>

Refresh the vendored deps tree by merging <upstream-ref> and re-running
'make third_party_downloads' against the merged tree.

Examples:
    $SCRIPT_NAME upstream/main
    $SCRIPT_NAME v1.3.4
    $SCRIPT_NAME 3c0b1e30
EOF
    exit 2
fi

REF="$1"

# Sanity checks ---------------------------------------------------------------

# 1. We are in a git worktree
git rev-parse --is-inside-work-tree >/dev/null 2>&1 \
    || die "not inside a git worktree"

# 2. We are at the root of the fork
[ -f "tensorflow/lite/micro/tools/make/Makefile" ] \
    || die "run this script from the root of the PX4/tflite-micro checkout"

# 3. We are on the vendored-deps branch
CURRENT_BRANCH=$(git symbolic-ref --short HEAD 2>/dev/null || echo "")
[ "$CURRENT_BRANCH" = "$BRANCH_NAME" ] \
    || die "must be on branch '$BRANCH_NAME' (currently on '$CURRENT_BRANCH')"

# 4. Working tree is clean
if ! git diff --quiet || ! git diff --cached --quiet; then
    die "working tree has uncommitted changes; commit or stash first"
fi

# 5. Upstream remote exists
if ! git remote get-url "$UPSTREAM_REMOTE" >/dev/null 2>&1; then
    cat >&2 <<EOF
error: remote '$UPSTREAM_REMOTE' is not configured.

To add it, run:
    git remote add $UPSTREAM_REMOTE $UPSTREAM_URL
    git fetch $UPSTREAM_REMOTE

Then re-run this script.
EOF
    exit 1
fi

# 6. GNU Make >= 3.82 (the Makefile hard-fails on older make; macOS ships 3.81)
MAKE_VERSION=$(make --version 2>/dev/null | head -1 | awk '{print $NF}' || echo "")
case "$MAKE_VERSION" in
    3.8[2-9]*|3.9*|[4-9].*|[1-9][0-9]*.*) ;;
    *)
        die "GNU Make >= 3.82 is required (found '$MAKE_VERSION'). On macOS: brew install make && export PATH=\"/opt/homebrew/opt/make/libexec/gnubin:\$PATH\""
        ;;
esac

# 7. Resolve the ref to a commit SHA
if ! TARGET_SHA=$(git rev-parse --verify "$REF^{commit}" 2>/dev/null); then
    die "cannot resolve ref '$REF' to a commit. Did you forget to 'git fetch $UPSTREAM_REMOTE'?"
fi

OLD_SHA=$(git rev-parse HEAD)

info "current vendored HEAD:  $OLD_SHA"
info "target upstream commit: $TARGET_SHA ($REF)"

# Merge upstream --------------------------------------------------------------

info "merging $REF into $BRANCH_NAME"
# Prefer-theirs for paths under downloads/ because we'll wipe and regenerate
# them immediately anyway. Everything else must merge cleanly.
if ! git merge --no-ff --no-edit "$TARGET_SHA"; then
    info "merge conflict detected; attempting to resolve downloads/ conflicts automatically"
    CONFLICTS=$(git diff --name-only --diff-filter=U)
    NON_DOWNLOAD_CONFLICTS=$(echo "$CONFLICTS" | grep -v "^$DOWNLOADS_DIR/" || true)
    if [ -n "$NON_DOWNLOAD_CONFLICTS" ]; then
        echo "error: merge conflicts outside downloads/ require manual resolution:" >&2
        echo "$NON_DOWNLOAD_CONFLICTS" >&2
        git merge --abort
        exit 1
    fi
    # All remaining conflicts are inside downloads/; use theirs and carry on
    echo "$CONFLICTS" | xargs -I {} git checkout --theirs "{}"
    echo "$CONFLICTS" | xargs git add
    git commit --no-edit
fi

# Wipe and re-populate downloads/ --------------------------------------------

info "wiping $DOWNLOADS_DIR"
rm -rf "$DOWNLOADS_DIR"

info "running 'make third_party_downloads' (this will fetch ~1GB; be patient)"
make -f tensorflow/lite/micro/tools/make/Makefile \
    MICRO_LITE_EXAMPLE_TESTS= \
    MICRO_LITE_BENCHMARKS= \
    MICRO_LITE_TEST_SRCS= \
    MICRO_LITE_INTEGRATION_TESTS= \
    third_party_downloads >/dev/null

# Prune --------------------------------------------------------------------

info "removing gcc_embedded toolchain (not a build dependency)"
rm -rf "$DOWNLOADS_DIR/gcc_embedded"

info "removing .git directories from vendored deps"
find "$DOWNLOADS_DIR" -name ".git" -type d -prune -exec rm -rf {} + 2>/dev/null || true

info "pruning docs, tests, examples, and non-C++ language bindings"

# Flatbuffers: drop multi-language bindings, tests, docs, samples
FLATBUFFERS="$DOWNLOADS_DIR/flatbuffers"
[ -d "$FLATBUFFERS" ] && rm -rf \
    "$FLATBUFFERS"/tests \
    "$FLATBUFFERS"/docs \
    "$FLATBUFFERS"/samples \
    "$FLATBUFFERS"/examples \
    "$FLATBUFFERS"/android \
    "$FLATBUFFERS"/dart \
    "$FLATBUFFERS"/go \
    "$FLATBUFFERS"/grpc \
    "$FLATBUFFERS"/java \
    "$FLATBUFFERS"/js \
    "$FLATBUFFERS"/kotlin \
    "$FLATBUFFERS"/lua \
    "$FLATBUFFERS"/mjs \
    "$FLATBUFFERS"/net \
    "$FLATBUFFERS"/nim \
    "$FLATBUFFERS"/php \
    "$FLATBUFFERS"/python \
    "$FLATBUFFERS"/rust \
    "$FLATBUFFERS"/swift \
    "$FLATBUFFERS"/ts \
    "$FLATBUFFERS"/lobster \
    "$FLATBUFFERS"/goldens \
    "$FLATBUFFERS"/benchmarks \
    "$FLATBUFFERS"/snap \
    "$FLATBUFFERS"/conan \
    2>/dev/null || true

# CMSIS: drop RTOS, docs, validation
CMSIS="$DOWNLOADS_DIR/cmsis"
[ -d "$CMSIS/CMSIS" ] && rm -rf \
    "$CMSIS/CMSIS/DoxyGen" \
    "$CMSIS/CMSIS/Documentation" \
    "$CMSIS/CMSIS/CoreValidation" \
    "$CMSIS/CMSIS/RTOS" \
    "$CMSIS/CMSIS/RTOS2" \
    "$CMSIS/CMSIS/Pack" \
    "$CMSIS/CMSIS/NN/Tests" \
    "$CMSIS/CMSIS/NN/Documentation" \
    "$CMSIS/CMSIS/DSP" \
    "$CMSIS/CMSIS/DAP" \
    2>/dev/null || true

# CMSIS-NN: drop docs and tests
CMSIS_NN="$DOWNLOADS_DIR/cmsis_nn"
[ -d "$CMSIS_NN" ] && rm -rf \
    "$CMSIS_NN"/Documentation \
    "$CMSIS_NN"/Tests \
    "$CMSIS_NN"/Examples \
    2>/dev/null || true

# Pigweed: drop docs (keep everything else; TFLM picks subsets at compile time)
PIGWEED="$DOWNLOADS_DIR/pigweed"
[ -d "$PIGWEED/docs" ] && rm -rf "$PIGWEED/docs"

# Stage and report -----------------------------------------------------------

info "staging $DOWNLOADS_DIR"
git add -f "$DOWNLOADS_DIR"

SIZE=$(du -sh "$DOWNLOADS_DIR" | awk '{print $1}')
FILE_COUNT=$(find "$DOWNLOADS_DIR" -type f | wc -l | awk '{print $1}')

cat <<EOF

==> Regeneration complete.

    upstream ref:   $REF
    upstream SHA:   $TARGET_SHA
    previous HEAD:  $OLD_SHA
    downloads size: $SIZE
    file count:     $FILE_COUNT

Next steps:

    git diff --stat HEAD
    git commit -S -s -m "vendor: refresh deps against $REF"
    git push origin $BRANCH_NAME

EOF
