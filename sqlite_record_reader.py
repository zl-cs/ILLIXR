from pathlib import Path
from typing import List
import sqlite3
import pandas as pd


def read_table(metrics_path: Path, table_name: str, index_cols: List[str]) -> pd.DataFrame:
    db_path = metrics_path / (table_name + ".sqlite")
    assert db_path.exists()
    data = (
        pd.read_sql_query(f"SELECT * FROM {table_name};", sqlite3.connect(str(db_path)))
    )
    if index_cols:
        data = (
            data
            .set_index(index_cols, verify_integrity=True)
            .sort_index()
        )
    return data
