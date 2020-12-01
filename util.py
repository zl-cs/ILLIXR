from __future__ import annotations

from typing import TypeVar, Iterable, Union, List, Optional, Any, Mapping
from pathlib import Path
import itertools
import subprocess

T = TypeVar("T")
def flatten(it: Iterable[Iterable[T]]) -> Iterable[T]:
    return itertools.chain.from_iterable(it)

class Subcommand:
    def __init__(
            self,
            command: Union[List[str], str, Path],
            short_flag: str = "-",
            long_flag: str = "-",
            kwarg_flag_sep: Optional[str] = None,
            parent: Optional[Subcommand] = None,
    ) -> None:
        if isinstance(command, Path):
            self.commands = [str(command)]
        elif isinstance(command, str):
            self.commands = [command]
        elif isinstance(command, list):
            self.commands = command
        else:
            raise TypeError(f"Type of {command} ({type(command)}) is bad")
        self.short_flag = short_flag
        self.long_flag = long_flag
        self.kwarg_flag_sep = kwarg_flag_sep
        self.parent = parent

    def to_flag(self, flag: str) -> str:
        if len(flag) == 1:
            return f"{self.short_flag}{flag}"
        else:
            return f"{self.long_flag}{flag}"

    def to_kwarg(self, var: str, obj: Any) -> List[str]:
        if isinstance(obj, bool):
            if obj:
                return [self.to_flag(var)]
            else:
                return []
        else:
            if self.kwarg_flag_sep:
                return [f"{self.to_flag(var)}{self.kwarg_flag_sep}{self.to_arg(obj)}"]
            else:
                return [self.to_flag(var), self.to_arg(obj)]

    def to_arg(self, obj: Any) -> str:
        return str(obj)

    def get_command_line(self) -> List[str]:
        parent_command = self.parent.get_command_line() if self.parent else []
        return parent_command + self.commands

    def __call__(self, *args: Any, _subprocess_kwargs: Mapping[str, Any] = {}, **kwargs: Any) -> subprocess.CompletedProcess[Any]:
        kwargs_flags = list(flatten(self.to_kwarg(var, val) for var, val in kwargs.items()))
        args_flags = list(map(self.to_arg, args))
        command = self.get_command_line() + args_flags + kwargs_flags
        proc = subprocess.run(command, check=True, **_subprocess_kwargs)
        return proc

    def __getattr__(self, name: str) -> Subcommand:
        return Subcommand(
            command=name,
            parent=self,
            short_flag=self.short_flag,
            long_flag=self.long_flag,
            kwarg_flag_sep=self.kwarg_flag_sep,
        )

# TODO: support git(C=blah).clone()
