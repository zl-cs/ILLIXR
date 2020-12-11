from typing import Iterable, TypeVar, Container
from pathlib import Path
import json
import pandas as pd


def parse_one_data(data_path: Path) -> pd.DataFrame:
    with data_path.open() as data_lines:
        data_first_line = next(data_lines)

    # First line should be JSON
    assert data_first_line.startswith("#{")
    options = json.loads(data_first_line[1:])

    assert options["version"] == "3.2"
    data = pd.read_csv(data_path, **options["pandas_kwargs"])
    assert not list(data.index.duplicates)
    return data


def parse_dir(path: Path, no_data_err=True) -> pd.DataFrame:
    datas = [
        parse_one_data(data_path)
        for data_path in path.glob("*_data.csv")
    ]
    if datas:
        return pd.concat(datas, verify_integrity=True, sort=True)
    elif no_data_err:
        raise RuntimeError(f"No *_data.csv found in {path}\n{list(path.iterdir())}")
    else:
        return pd.DataFrame()
