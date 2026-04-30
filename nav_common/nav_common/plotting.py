"""Registro de trajetória para análise offline e geração de gráficos do relatório."""

import csv
import os
import time
from typing import Optional


class TrajectoryLogger:
    """Registra poses (x, y, yaw, timestamp) em um arquivo CSV.

    Uso:
        logger = TrajectoryLogger('/tmp/trajetoria_ex2.csv')
        ...
        logger.log(x, y, yaw)
        ...
        logger.close()

    Depois, em Python/Jupyter:
        import pandas as pd
        df = pd.read_csv('/tmp/trajetoria_ex2.csv')
        plt.plot(df['x'], df['y'])
    """

    def __init__(self, filepath: str, extra_fields: Optional[list] = None):
        self.filepath = filepath
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        self.fields = ['timestamp', 'x', 'y', 'yaw']
        if extra_fields:
            self.fields.extend(extra_fields)

        self.file = open(filepath, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()

    def log(self, x: float, y: float, yaw: float, **extra):
        """Registra uma pose com timestamp atual."""
        row = {
            'timestamp': time.time(),
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(yaw, 4),
        }
        row.update(extra)
        self.writer.writerow(row)

    def close(self):
        """Fecha o arquivo CSV."""
        if self.file and not self.file.closed:
            self.file.close()

    def __del__(self):
        self.close()