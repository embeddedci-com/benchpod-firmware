# Contributing

Thanks for helping out. Issues and pull requests are welcome.

## Before you open a pull request

- Open an issue first for anything larger than a small fix, so we can agree on the approach.
- Read [AGENTS.md](AGENTS.md): it covers the iCE40 timing rules and why a hardware symptom is
  almost always firmware or gateware.
- Run the checks CI runs:

  ```bash
  git submodule update --init --recursive
  make -C stm32h563/test run        # host tests
  make -C stm32h563 -j8 all         # firmware build (needs arm-none-eabi-gcc; git submodule update --init --recursive)
  make -C ice40 PNR_STRICT=1 test   # gateware simulation (needs the OSS CAD Suite)
  ```

- A change that touches the pod's behavior needs a run on a real BenchPod. Say in the pull
  request which board, which build and which tests you ran.
- Gateware releases embed the hardware-verified images in `ice40/release/`; only a maintainer
  promotes new ones (`make -C ice40 promote` after a bench run).
- A new feature (a command, an argument, a gateware block) gets an entry in
  `features/features.toml` in the same pull request, with `rev` one above `revision`; then run
  `python3 tools/gen_features.py`. The pod does not announce features one by one: clients look
  them up in that table by the pod's `features_rev`. Don't add names to the status `caps` list
  (it is frozen; CI fails on a new name).
- Keep commits small and focused, and describe what changed and why.

## Releases

Maintainers release by pushing a `stm32-vX.Y.Z` tag that matches `stm32h563/src/version.h`.
The release pull request (the `version.h` bump) also runs
`python3 tools/gen_features.py --release X.Y.Z`, which dates the table's `"next"` entries; the
release workflow fails without it.
`tools/release_notes.py` drafts the notes from the merged pull requests.

## Security

Report vulnerabilities privately, see [SECURITY.md](SECURITY.md).
