# Evidence files

| File | Contents |
| --- | --- |
| PROGRESS_LOG.md | Team observations |
| watchdog_chat_excerpt.txt | Original log excerpts |
| CURRENT_SOURCE_TEST_RESULTS.txt | Host checks and hashes |
| CALCULATION_RESULTS.txt | Planned value and sensor calculations |
| REQUIREMENT_EVIDENCE_MATRIX.md | Requirement status |
| UNIT_EVIDENCE_MAP.md | Handlers and checks |

From the repository root:

```sh
python3 -B unit_testing_result/verify_report_calculations.py
python3 -B robot_validation/software_tests/run_current_source_tests.py
```

No new hardware measurements or actual labour hours. Empty templates are not results.
