# Deep Learning

Deep-learning methods are expected to enter through Python first. MATLAB calls
Python via `tracking.learning.PythonModelAdapter`.

The current Python contract is:

```python
from target_tracking_dl import predict

result = predict(sequence, config)
```

`result` must be compatible with the TrackingResult keys:

`metadata`, `truth`, `measurements`, `estimates`, `metrics`, `config`.

This repository does not yet define a neural network architecture. Add model
code under `python/src/target_tracking_dl/` and keep MATLAB integration code
thin.
