"""Optional per-stage wall-clock timing for the HGVD explorer."""
import csv
from contextlib import contextmanager
from pathlib import Path
from time import perf_counter_ns


class TimingCsv:
    def __init__(self, filename):
        self.path = Path(filename).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', encoding='utf-8', newline='', buffering=1)
        self.writer = csv.writer(self.stream)
        self.writer.writerow(('elapsed_s', 'stage', 'duration_ms'))
        self.start_ns = perf_counter_ns()

    @contextmanager
    def measure(self, stage):
        start_ns = perf_counter_ns()
        try:
            yield
        finally:
            end_ns = perf_counter_ns()
            self.writer.writerow((
                f'{(start_ns - self.start_ns) / 1e9:.9f}',
                stage,
                f'{(end_ns - start_ns) / 1e6:.6f}',
            ))

    def close(self):
        self.stream.close()
