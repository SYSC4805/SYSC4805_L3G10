# Software checks

With Python 3 and clang++ or g++ installed, run:

```sh
python3 -B run_current_source_tests.py
```

The runner tests the packaged sketches with mocked hardware and records source hashes. `test_analysis.py` uses synthetic measurements. Older `*_test.cpp` files are historical snapshots.

Passing checks do not establish physical accuracy, timing or hardware reset behaviour.
