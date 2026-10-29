"""Finds Doxygen comments that are present but say too little: the faults that Doxygen's own warnings do not find.

Doxygen (the repository's Doxyfile) reports a member without a comment and a parameter without @param. This tool
reports three faults in comments that do exist:

- NO BRIEF: a function's comment starts with a tag (@param, @return, ...) and has no @brief and no first sentence.
  A one-line accessor without parameters ("int x() const { return x_; }") needs only its @return.
- REPEATS NAME: a @param or @tparam text that only repeats the parameter's name ("@param cell the cell").
- NO THROWS: a function whose body has a throw, std::stod, std::stoul or std::stoi and whose comment has no @throws.
  The body is the inline body, or the definition Class::name(...) in a .cpp file given on the same command line,
  matched by name and parameter count. An exception from a called function (also from std::vector::at()) is not
  found: that is a review item. ".at(" is not a pattern, because the Setup API has members named at() that do not
  throw.

Usage:
    python tests/docs/doc_lint.py [--changed-since <revision>] <file> [<file> ...]

With --changed-since, a finding is reported only when its function's declaration line was added or changed after
the revision (git diff). Use it for a file that existed before a change, to check only the change's part.

Output: one line per finding, "<file>:<line>: <FAULT> <function>: <detail>", then "DONE: N findings in M files".
The exit code is 0 only when there is no finding. A file that cannot be read stops the tool with an error.
"""
from __future__ import annotations

import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ARTICLES = {"the", "a", "an", "its", "of", "given"}
THROWING = re.compile(r"\bthrow\b|\bstd::sto(?:d|ul|i|l|ull|f)\s*\(")
CLASS_HEAD = re.compile(r"^\s*(?:template\s*<[^>]*>\s*)?(?:class|struct)\s+(\w+)[^;]*$")
NAME_BEFORE_PAREN = re.compile(r"(~?\w+|operator\s*[^\s(]+)\s*\($")
DECLARATION_LINES = 6  # a declaration may wrap over this many lines


@dataclass(frozen=True)
class Finding:
    """One fault in one comment."""

    path: str
    line: int
    fault: str
    function: str
    detail: str


@dataclass(frozen=True)
class FunctionDoc:
    """A comment block and the function declaration under it."""

    path: str
    line: int  # the declaration's first line, 1-based
    owner: str  # the enclosing class, or "" outside a class
    name: str
    parameters: int
    comment: list[str]  # the comment lines without the leading "///"
    declaration: str
    inline_body: str  # the body when the declaration defines it, else ""


@dataclass
class SourceFile:
    """One file's text and the positions of its braces."""

    path: str
    lines: list[str]
    text: str = field(init=False)

    def __post_init__(self) -> None:
        self.text = "\n".join(self.lines)


class CodeText:
    """Small text helpers for C++ code (comments and literals are handled roughly: enough for declarations)."""

    STRIP = re.compile(r'"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|//[^\n]*|/\*.*?\*/', re.DOTALL)

    @classmethod
    def without_comments(cls, text: str) -> str:
        return cls.STRIP.sub(lambda m: m.group(0) if m.group(0)[0] in "\"'" else " ", text)

    @staticmethod
    def matching_brace(text: str, open_at: int) -> int:
        """Returns the index of the brace that closes the one at open_at (-1 when it is not closed)."""
        depth = 0
        for i in range(open_at, len(text)):
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
                if depth == 0:
                    return i
        return -1

    @staticmethod
    def parameter_count(declaration: str, name_end: int) -> int:
        """Counts the top-level commas between the parentheses that start at name_end."""
        depth = 0
        count = 0
        has_text = False
        for ch in declaration[name_end:]:
            if ch in "(<[{":
                depth += 1
                if depth == 1 and ch == "(":
                    continue
            elif ch in ")>]}":
                depth -= 1
                if depth == 0:
                    break
            elif ch == "," and depth == 1:
                count += 1
                continue
            if depth >= 1 and not ch.isspace():
                has_text = True
        params = count + 1 if has_text else 0
        return params


class DocReader:
    """Finds the function comments of a file."""

    def read(self, source: SourceFile) -> list[FunctionDoc]:
        docs: list[FunctionDoc] = []
        owners: list[tuple[int, str]] = []  # (brace depth inside the class, class name)
        depth = 0
        comment: list[str] = []
        i = 0
        while i < len(source.lines):
            raw = source.lines[i]
            stripped = raw.strip()
            if stripped.startswith("///") and not stripped.startswith("///<"):
                comment.append(stripped[3:].strip())
                i += 1
                continue
            if comment and stripped:
                doc = self._function(source, i, comment, owners[-1][1] if owners else "")
                if doc:
                    docs.append(doc)
            if stripped:
                comment = []
            head = CLASS_HEAD.match(CodeText.without_comments(raw))
            code = CodeText.without_comments(raw)
            for ch in code:
                if ch == "{":
                    depth += 1
                    if head and not owners or head and owners[-1][0] != depth:
                        owners.append((depth, head.group(1)))
                        head = None
                elif ch == "}":
                    if owners and owners[-1][0] == depth:
                        owners.pop()
                    depth -= 1
            i += 1
        return docs

    def _function(self, source: SourceFile, at: int, comment: list[str], owner: str) -> FunctionDoc | None:
        joined = " ".join(CodeText.without_comments(l).strip() for l in source.lines[at : at + DECLARATION_LINES])
        first = joined.lstrip()
        if re.match(r"(class|struct|enum|union|using|namespace|typedef|static_assert|#|template\s*<[^>]*>\s*(class|struct))\b", first):
            return None
        paren = first.find("(")
        equals = first.find("=")
        brace = first.find("{")
        semicolon = first.find(";")
        if paren < 0 or (0 <= equals < paren) or (0 <= brace < paren) or (0 <= semicolon < paren):
            return None
        match = NAME_BEFORE_PAREN.search(first[: paren + 1])
        if not match:
            return None
        name = match.group(1).replace(" ", "")
        if name in {"if", "for", "while", "switch", "return", "sizeof", "decltype"}:
            return None
        parameters = CodeText.parameter_count(first, paren)
        body = ""
        end_of_declaration = min(x for x in (first.find(";", paren), first.find("{", paren), len(first)) if x >= 0)
        if end_of_declaration < len(first) and first[end_of_declaration] == "{":
            body = self._inline_body(source, at)
        return FunctionDoc(
            path=source.path,
            line=at + 1,
            owner=owner,
            name=name,
            parameters=parameters,
            comment=list(comment),
            declaration=first[:end_of_declaration].strip(),
            inline_body=body,
        )

    @staticmethod
    def _inline_body(source: SourceFile, at: int) -> str:
        rest = CodeText.without_comments("\n".join(source.lines[at:]))
        open_at = rest.find("{")
        close_at = CodeText.matching_brace(rest, open_at)
        body = rest[open_at : close_at + 1] if close_at > 0 else rest[open_at:]
        return body


class DefinitionIndex:
    """The bodies of Class::name(...) definitions in .cpp files, by (class, name, parameter count)."""

    DEFINITION = re.compile(r"\b(\w+)::(~?\w+)\s*\(")

    def __init__(self) -> None:
        self.bodies: dict[tuple[str, str, int], list[str]] = {}

    def add(self, source: SourceFile) -> None:
        text = CodeText.without_comments(source.text)
        for match in self.DEFINITION.finditer(text):
            line_start = text.rfind("\n", 0, match.start()) + 1
            prefix = text[line_start : match.start()]
            if prefix.strip().startswith(("return", "const", "=")) or "=" in prefix or "(" in prefix:
                continue
            paren = match.end() - 1
            parameters = CodeText.parameter_count(text, paren)
            after = text[paren:]
            semicolon = after.find(";")
            brace = after.find("{")
            if brace < 0 or (0 <= semicolon < brace and ":" not in after[:brace]):
                continue
            open_at = paren + brace
            close_at = CodeText.matching_brace(text, open_at)
            if close_at < 0:
                continue
            key = (match.group(1), match.group(2), parameters)
            self.bodies.setdefault(key, []).append(text[open_at : close_at + 1])

    def body(self, doc: FunctionDoc) -> str:
        found = self.bodies.get((doc.owner, doc.name, doc.parameters), [])
        return "\n".join(found)


class DocLint:
    """Applies the three checks to the function comments."""

    TAG = re.compile(r"@(param|tparam)(?:\[[^\]]*\])?\s+(\w+)\s*(.*)")

    def __init__(self, definitions: DefinitionIndex) -> None:
        self.definitions = definitions

    def check(self, doc: FunctionDoc) -> list[Finding]:
        findings: list[Finding] = []
        label = f"{doc.owner}::{doc.name}" if doc.owner else doc.name
        if self._needs_brief(doc):
            findings.append(Finding(doc.path, doc.line, "NO BRIEF", label, "the comment starts with a tag"))
        for tag, name, text in self._parameters(doc):
            if self._repeats(name, text):
                findings.append(Finding(doc.path, doc.line, "REPEATS NAME", label, f"@{tag} {name} {text}".strip()))
        body = doc.inline_body or self.definitions.body(doc)
        has_throws = any(l.startswith(("@throws", "@throw", "@exception")) for l in doc.comment)
        if body and THROWING.search(body) and not has_throws:
            findings.append(Finding(doc.path, doc.line, "NO THROWS", label, "the body throws, the comment has no @throws"))
        return findings

    @staticmethod
    def _needs_brief(doc: FunctionDoc) -> bool:
        first = next((l for l in doc.comment if l), "")
        has_brief = any(l.startswith("@brief") for l in doc.comment) or not first.startswith("@")
        accessor = doc.parameters == 0 and re.search(r"\{\s*return\b[^;]*;\s*\}\s*$", doc.inline_body or "") is not None
        return not has_brief and not accessor

    def _parameters(self, doc: FunctionDoc) -> list[tuple[str, str, str]]:
        found: list[tuple[str, str, str]] = []
        for line in doc.comment:
            match = self.TAG.match(line)
            if match:
                found.append((match.group(1), match.group(2), match.group(3)))
        return found

    @staticmethod
    def _repeats(name: str, text: str) -> bool:
        words = [w.lower().removesuffix("'s") for w in re.findall(r"[A-Za-z']+", text)]
        meaningful = [w for w in words if w not in ARTICLES]
        name_words = {w.lower() for w in re.split(r"_|(?<=[a-z])(?=[A-Z])", name) if w}
        repeats = not meaningful or set(meaningful) <= name_words
        return repeats


class ChangedLines:
    """The line numbers of a file that were added or changed after a git revision."""

    HUNK = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@", re.MULTILINE)

    def __init__(self, revision: str) -> None:
        self.revision = revision

    def of(self, path: str) -> set[int]:
        result = subprocess.run(["git", "diff", "-U0", self.revision, "--", path], capture_output=True)
        if result.returncode != 0:
            raise RuntimeError(f"git diff {self.revision} -- {path} failed: {result.stderr.decode(errors='replace').strip()}")
        lines: set[int] = set()
        for match in self.HUNK.finditer(result.stdout.decode("utf-8", errors="replace")):
            start = int(match.group(1))
            count = int(match.group(2)) if match.group(2) is not None else 1
            lines.update(range(start, start + count))
        return lines


def main(arguments: list[str]) -> int:
    changed: ChangedLines | None = None
    if arguments[:1] == ["--changed-since"]:
        if len(arguments) < 2:
            raise SystemExit("--changed-since needs a revision")
        changed = ChangedLines(arguments[1])
        arguments = arguments[2:]
    if not arguments:
        raise SystemExit(__doc__)
    sources = [SourceFile(path=p, lines=Path(p).read_bytes().decode("utf-8").splitlines()) for p in arguments]
    definitions = DefinitionIndex()
    for source in sources:
        if source.path.endswith(".cpp"):
            definitions.add(source)
    reader = DocReader()
    lint = DocLint(definitions)
    findings = [f for source in sources for doc in reader.read(source) for f in lint.check(doc)]
    if changed is not None:
        lines_of = {source.path: changed.of(source.path) for source in sources}
        findings = [f for f in findings if f.line in lines_of[f.path]]
    for f in findings:
        print(f"{f.path}:{f.line}: {f.fault} {f.function}: {f.detail}")
    print(f"DONE: {len(findings)} findings in {len(sources)} files")
    return 0 if not findings else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
