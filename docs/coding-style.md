# Coding Style

Use the existing MATLAB style unless a new module needs a clearer local pattern.

- Classes use `PascalCase`: `PythonModelAdapter`, `ProjectConfig`.
- Functions and methods use `camelCase`: `loadConfig`, `createTrackingResult`.
- Constants use `UPPER_SNAKE_CASE`.
- New public MATLAB code should live under `src/matlab/+tracking`.
- New reusable examples should live under `examples/matlab`.
- New test code should live under `tests/matlab/unit`, `tests/matlab/integration`
  or `tests/python`.
- Prefer JSON configs in `configs/` when a scenario may be shared with Python.
- Keep result payloads compatible with the TrackingResult keys documented in
  `docs/architecture.md`.
