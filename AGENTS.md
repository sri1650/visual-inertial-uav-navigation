# Agent Execution Policy

## Objective
Finish this repository with minimum expensive-model usage.

## Rules
1. Work on one bounded task at a time from `tasks/TASKS.json`.
2. Read `docs/context/CURRENT_STATE.md` before coding.
3. Do not reread the whole repo unless a failing deterministic check proves it is needed.
4. Prefer compilers, linters, tests and benchmark scripts over LLM judgement.
5. Maximum two repair attempts per task. Then write a blocker and escalate only that blocker.
6. Do not rewrite passing code for style.
7. Expensive reasoning is reserved for:
   - estimator architecture decisions
   - hard numerical/concurrency bugs
   - cross-component integration failures
   - final architecture review
8. Update CURRENT_STATE only after verification passes.
9. Commit small verified milestones.
10. Never claim physical flight or hardware validation unless actually performed.

## Verification
Python:
- `python -m pytest -q`
- `python -m compileall src tests`

C++:
- `cmake -S cpp -B build/cpp`
- `cmake --build build/cpp --parallel`
- `ctest --test-dir build/cpp --output-on-failure`

Later:
- clang-format / clang-tidy when configured
- PX4 SITL integration tests
- EuRoC benchmark scripts
