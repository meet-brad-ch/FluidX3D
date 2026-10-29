"""Proves that a change to C++ files changed only comments.

For each file, the tool reads the version at a git revision and the version in the working tree, removes the
comments from both (// and /* */, outside string and character literals), splits what is left into tokens, and
compares the two token lists. Equal lists mean that the compiler sees the same code. A layout change (spaces, line
breaks) is not a code change and is not reported.

Usage:
    python tests/docs/comment_only_diff.py <revision> <file> [<file> ...]

Output: one line per file, "SAME <file>" or "CODE CHANGED <file>: first difference at token N (old ... | new ...)",
then "DONE: N same, M changed". The exit code is 0 only when every file is the same. A file that git or the file
system cannot read stops the tool with an error.
"""
from __future__ import annotations

import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

# A C++ token: a string or character literal (with escapes and an optional prefix such as u8 or L), a raw string
# literal R"delim(...)delim", a number, a name, or one punctuation character.
TOKEN = re.compile(
    r'(?:u8|[uUL])?R"(?P<delim>[^()\\\s]{0,16})\(.*?\)(?P=delim)"'
    r'|(?:u8|[uUL])?"(?:\\.|[^"\\\n])*"'
    r"|(?:u8|[uUL])?'(?:\\.|[^'\\\n])*'"
    r"|\.?\d(?:[eEpP][+-]|[\w.'])*"
    r"|[A-Za-z_]\w*"
    r"|\S",
    re.DOTALL,
)

CONTEXT_TOKENS = 6  # tokens shown on each side of the first difference


@dataclass(frozen=True)
class Comparison:
    """The result of one file."""

    path: str
    same: bool
    first_difference: int  # the index of the first token that differs (-1 when same)
    old_context: str
    new_context: str


class CommentStripper:
    """Removes the comments from C++ source text and keeps string and character literals as they are."""

    LITERAL_OR_COMMENT = re.compile(
        r'(?:u8|[uUL])?R"(?P<delim>[^()\\\s]{0,16})\(.*?\)(?P=delim)"'
        r'|(?:u8|[uUL])?"(?:\\.|[^"\\\n])*"'
        r"|(?:u8|[uUL])?'(?:\\.|[^'\\\n])*'"
        r"|//[^\n]*"
        r"|/\*.*?\*/",
        re.DOTALL,
    )

    def strip(self, text: str) -> str:
        """Returns text with each comment replaced by one space (so that two tokens never join)."""
        return self.LITERAL_OR_COMMENT.sub(self._keep_literal, text)

    @staticmethod
    def _keep_literal(match: re.Match[str]) -> str:
        found = match.group(0)
        is_comment = found.startswith("//") or found.startswith("/*")
        return " " if is_comment else found


class Tokenizer:
    """Splits comment-free C++ text into tokens."""

    def tokens(self, text: str) -> list[str]:
        return [m.group(0) for m in TOKEN.finditer(text)]


class SourceReader:
    """Reads a file at a git revision and in the working tree."""

    def __init__(self, revision: str) -> None:
        self.revision = revision

    def at_revision(self, path: str) -> str:
        relative = Path(path).as_posix()
        result = subprocess.run(["git", "show", f"{self.revision}:{relative}"], capture_output=True)
        if result.returncode != 0:
            raise RuntimeError(f"git show {self.revision}:{relative} failed: {result.stderr.decode(errors='replace').strip()}")
        return result.stdout.decode("utf-8")

    @staticmethod
    def in_working_tree(path: str) -> str:
        return Path(path).read_bytes().decode("utf-8")


class CommentOnlyDiff:
    """Compares the code tokens of a file's two versions."""

    def __init__(self, reader: SourceReader) -> None:
        self.reader = reader
        self.stripper = CommentStripper()
        self.tokenizer = Tokenizer()

    def compare(self, path: str) -> Comparison:
        old = self.tokenizer.tokens(self.stripper.strip(self.reader.at_revision(path)))
        new = self.tokenizer.tokens(self.stripper.strip(self.reader.in_working_tree(path)))
        if old == new:
            return Comparison(path=path, same=True, first_difference=-1, old_context="", new_context="")
        index = next((i for i, (a, b) in enumerate(zip(old, new)) if a != b), min(len(old), len(new)))
        first = max(index - CONTEXT_TOKENS, 0)
        last = index + CONTEXT_TOKENS
        return Comparison(
            path=path,
            same=False,
            first_difference=index,
            old_context=" ".join(old[first:last]),
            new_context=" ".join(new[first:last]),
        )


def main(arguments: list[str]) -> int:
    if len(arguments) < 2:
        raise SystemExit(__doc__)
    revision, paths = arguments[0], arguments[1:]
    diff = CommentOnlyDiff(SourceReader(revision))
    results = [diff.compare(p) for p in paths]
    for r in results:
        if r.same:
            print(f"SAME {r.path}")
        else:
            print(f"CODE CHANGED {r.path}: first difference at token {r.first_difference} (old {r.old_context} | new {r.new_context})")
    same = sum(1 for r in results if r.same)
    print(f"DONE: {same} same, {len(results) - same} changed")
    return 0 if same == len(results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
