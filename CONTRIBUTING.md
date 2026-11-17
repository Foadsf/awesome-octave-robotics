# Contributing to awesome-octave-robotics

Thank you for considering a contribution! `awesome-octave-robotics` is a
clean-room, self-contained GNU Octave replication of robotics algorithms,
simulators, pub/sub middleware, and control frameworks — with **zero
proprietary toolbox dependencies**.

## Ground rules

* **One public function per `.m` file**, with the function name matching the
  filename. Subfunctions are allowed only within the same file and are not
  visible across files.
* **No MATLAB or Octave-Forge dependencies.** Pure Octave native code only.
* **Headless execution only.** All code must run under
  `octave-cli.exe --no-gui --quiet` (GNU Octave 11.3+).
* **No stubs or TODOs.** Every function must implement a real numerical
  algorithm.
* **Headless PNG figure export** is required for any plotting demo.

## Octave 11.3+ compatibility rules

1. **No `?:` ternary operator.** Octave does not support it — use `if/else`.
2. Use explicit `endfor`, `endif`, `endfunction`, `endswitch`, or clean
   `end` to close blocks.
3. Headless plotting must force the `gnuplot` toolkit per figure
   (`graphics_toolkit(hf, "gnuplot")`) because fltk/qt need a visible display.
   See `src/core/plot_helpers.m` (`ph_newfig`).
4. `run()` changes the working directory to the script's directory. Compute
   the repo root from `mfilename("fullpath")` for cross-directory file
   references:
   ```octave
   repo_root = fileparts(fileparts(mfilename("fullpath")));
   ```
5. Every demo must write real PNG output to `output/figures/`.
6. Every test file must `run('setup_paths.m')` before calling modules.

## Adding a new domain or algorithm

1. Create `src/domains/<area>/<algorithm>.m` with one public function.
2. Add a demo under `demos/` that runs headless and writes a PNG.
3. Add checks to the appropriate `tests/test_batch*.m` suite.
4. Run the full verification locally before opening a PR:
   ```powershell
   octave-cli --no-gui --quiet --eval "run('tests/run_all_tests.m');"
   octave-cli --no-gui --quiet --eval "run('demos/run_all_demos.m');"
   ```
   Both must exit with status `0`.

## Pull request process

1. Fork the repo and create a topic branch from `main`.
2. Make your changes, keeping commits atomic and scoped.
3. Run the full test + demo suite (exit code 0 required).
4. Open a pull request against `main`. The maintainer will review and run CI.

## Code of Conduct

This project follows the Contributor Covenant v2.1. See `CODE_OF_CONDUCT.md`.

## License

By contributing, you agree that your contributions will be licensed under the
BSD-3-Clause license (see `LICENSE`).
