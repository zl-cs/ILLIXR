#!/usr/bin/env python3
from typing import Iterable
import itertools
import threading
from pathlib import Path
import subprocess
import concurrent.futures

def scan_for_components(parent: Path) -> Iterable[Path]:
    for component in parent.iterdir():
        if (component / "Makefile").exists():
            yield component

def clean(component: Path) -> None:
    subprocess.run(["make", "-C", str(component), "clean"], check=True)

grand_parents = [Path(), Path(".cache"), Path("..") / "ILLIXR_comp"]
with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
    components = itertools.chain.from_iterable(map(scan_for_components, grand_parents))
    list(executor.map(clean, components))
