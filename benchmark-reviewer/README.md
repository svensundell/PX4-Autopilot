# Benchmark reviewer corpus

Reference documents for PR review benchmarking (Copilot Code Review, Komainu, etc.).

## Corpus (`corpus/`)

Copies of PX4 safety and requirements documentation used as the review ground-truth source:

| File | Original path in PX4 |
|------|----------------------|
| `corpus/safety.md` | `docs/en/config/safety.md` |
| `corpus/flight_modes.md` | `docs/en/concept/flight_modes.md` |
| `corpus/detect_and_avoid.md` | `docs/en/advanced_features/detect_and_avoid.md` |
| `corpus/prearm_arm_disarm.md` | `docs/en/advanced_config/prearm_arm_disarm.md` |
| `corpus/flight_termination.md` | `docs/en/advanced_config/flight_termination.md` |
| `corpus/tuning_the_ecl_ekf.md` | `docs/en/advanced_config/tuning_the_ecl_ekf.md` |

GitHub Copilot Code Review is configured to use this folder via `.github/copilot-instructions.md` on the default branch.

## Local evaluation only

`ground-truth.json` (violation oracle for the benchmark PR) is **not** committed. Keep it locally for scoring reviewer output against the intentional violations.
