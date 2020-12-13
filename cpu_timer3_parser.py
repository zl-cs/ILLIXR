from typing import Iterable, TypeVar, Container
from pathlib import Path
import json
import pandas as pd


ending = "_data.csv"


def parse_one_data(data_path: Path) -> pd.DataFrame:
    # First line should be JSON metadata
    with data_path.open() as data_lines:
        data_first_line = next(data_line)
    assert data_first_line.startswith("#{")
    options = json.loads(data_first_line[1:])
    assert options["version"] == "3.2"

    data = (
        pd.read_csv(data_path, **options["pandas_kwargs"])
        .sort_index()
    )

    # Within each thread, function_id uniquely identifies a function name
    # but we only print the corresponding name on the its occurrence, to save space/memory in CSV.
    # I can achieve the same efficiency with Pandas Categorical type
    function_name_map = (
        data
        .groupby(["thread_id", "function_id"])
        .agg({"function_name": "first"})
        .assign(**{
            "function_name": lambda df: df["function_name"].astype("category")
        })
    )

    # Check that map is filled for all (thread_id, function_id)
    if not function_name_map.all():
        raise RuntimeError("\n".join([
            f"No function name for (thread_id, function_id) in {data_path!s}:",
            str(list(function_name_map.index[function_name_map == '']))
        ]))

    # Change function_id -> function_name (by joining with function_name_map)
    data = (
        data
        .drop(columns=["function_name"])
        .join(function_name_map, how="left")
    )

    # (thread_id, stack_id) is unique within the data_file, but not within the whole program!
    # Assign it as an index
    data = (
        data
        .assign(**{
            "data_file": data_path.name[:-len(ending)],
        })
        .set_index(["data_file", "thread_id", "frame_id"], drop=True, verify_integrity=True)
        .sort_index()
    )

    return data


def parse_dir(path: Path, no_data_err=True) -> pd.DataFrame:
    files = list(path.glob("*_data.csv"))
    if files:
        data = pd.concat([
            parse_one_data(file)
            for file in files
        ], verify_integrity=True, sort=True)
        return data
    elif no_data_err:
        raise RuntimeError(f"No *_data.csv found in {path}\n{list(path.iterdir())}")
    else:
        return pd.DataFrame()
