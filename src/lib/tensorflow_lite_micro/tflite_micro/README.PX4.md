# PX4 Vendored Dependencies Branch

This branch (`px4/vendored-deps`) is the `PX4/tflite-micro` fork branch consumed
by the `PX4/PX4-Autopilot` submodule at `src/lib/tensorflow_lite_micro/tflite_micro`.

It is identical to upstream `tensorflow/tflite-micro` at the pinned commit, with
one addition: the five third-party dependencies that upstream's Makefile
normally fetches from the internet at build time are committed in place under
`tensorflow/lite/micro/tools/make/downloads/`.

## Why this exists

PX4 requires reproducible builds: the same source tree must produce the same
binary today, tomorrow, and five years from now, on any machine, with no
external dependencies beyond the toolchain.

Upstream tflite-micro's Makefile breaks this. At build time it runs `wget` and
`git clone` against `github.com` and `pigweed.googlesource.com` to fetch five
third-party dependencies (flatbuffers, kissfft, gemmlowp, ruy, pigweed, plus
CMSIS/CMSIS-NN for ARM targets). That means a PX4 build's output depends on:

- `github.com` and `pigweed.googlesource.com` being reachable at build time
- Those hosts still serving the exact pinned versions
- Archive tarballs being byte-identical to when the MD5s were recorded (GitHub
  has silently regenerated archive zips in the past, breaking checksums across
  the ecosystem)
- No transparent proxy, no regional mirror, no mirror rewrite

Any one of those assumptions failing turns a clean checkout into a broken
build, with no change in the source tree. That is the definition of a
non-reproducible build.

This branch fixes it by committing the dependencies directly into the
repository under `tensorflow/lite/micro/tools/make/downloads/`. After
`git clone --recursive`, every byte needed to build is already on disk. Builds
become hermetic, reproducible, and offline-capable, with no PX4-hosted mirror
or external infrastructure to maintain.

Upstream's own download scripts (`flatbuffers_download.sh`, `kissfft_download.sh`,
etc.) already short-circuit with an "already exists, skipping the download"
message when the target directory is present. This branch uses that sanctioned
escape hatch: the files exist, the downloads are skipped, nothing else changes.

See PX4/PX4-Autopilot#27054 for the original report.

## One-time setup (maintainer)

Clone this fork and add upstream `tensorflow/tflite-micro` as a second remote:

```sh
git clone git@github.com:PX4/tflite-micro.git
cd tflite-micro
git checkout px4/vendored-deps
git remote add upstream https://github.com/tensorflow/tflite-micro.git
git fetch upstream
```

You will also need GNU Make 4 or later (the system `make` on macOS is too old).
Install via Homebrew and put it on PATH:

```sh
brew install make
export PATH="/opt/homebrew/opt/make/libexec/gnubin:$PATH"
```

## Updating the vendored branch from upstream

To refresh the vendored tree against a newer upstream commit, branch, or tag:

```sh
./tools/px4/regenerate-vendored-deps.sh upstream/main
./tools/px4/regenerate-vendored-deps.sh v1.3.4
./tools/px4/regenerate-vendored-deps.sh 3c0b1e30
```

The script:

1. Merges the specified upstream ref into the current branch.
2. Wipes the existing `tensorflow/lite/micro/tools/make/downloads/` tree.
3. Runs `make third_party_downloads` to re-populate it.
4. Removes the `gcc_embedded` ARM toolchain (not a build dependency, huge).
5. Removes `.git` directories from each dep.
6. Prunes docs, tests, examples, and non-C++ language bindings to keep the
   branch reviewable in size.
7. Stages the result for you to review.

Review the diff, then commit and push:

```sh
git diff --stat HEAD
git commit -S -s -m "vendor: refresh deps against <upstream-ref>"
git push origin px4/vendored-deps
```

## Updating PX4-Autopilot to use the new vendored commit

In your PX4-Autopilot checkout:

```sh
cd src/lib/tensorflow_lite_micro/tflite_micro
git fetch origin px4/vendored-deps
git checkout <new-sha>
cd -
git add src/lib/tensorflow_lite_micro/tflite_micro
git commit -S -s -m "tflite-micro: bump vendored submodule"
```

## Verifying hermeticity

After bumping the submodule in PX4-Autopilot, verify that builds succeed with
network access fully disabled. On Linux:

```sh
unshare -rn bash -c "make px4_sitl_neural"
unshare -rn bash -c "make mro_pixracerpro_neural"
unshare -rn bash -c "make px4_fmu-v6c_neural"
```

On macOS, run the build inside a container with `--network=none`.

A clean build with the network off confirms the branch is doing its job.

## Prior art

Other embedded projects following the same "vendor the deps" pattern for the
same reasons:

- [espressif/esp-tflite-micro](https://github.com/espressif/esp-tflite-micro) -
  commits pre-patched third-party sources directly under `third_party/`
- [zephyrproject-rtos/tflite-micro](https://github.com/zephyrproject-rtos/tflite-micro) -
  pulled via west manifest as a Zephyr module
- [GNU Guix tflite-micro package](http://www.mail-archive.com/guix-commits@gnu.org/msg311585.html) -
  patches the Makefile to disable downloads and supplies each dep as a
  first-class package input
- [biagiom/tflite-micro-lib](https://github.com/biagiom/tflite-micro-lib) -
  standalone library repo with vendored deps
