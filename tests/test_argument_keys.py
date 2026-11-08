#!/usr/bin/env python3
"""Keep the header's public vocabulary equal to the keys read by the acquisition program."""
import pathlib
import re
import sys


def without_comments(source):
    # Preserve quoted strings so comment-looking text inside them stays literal.
    tokens = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/'
    return re.sub(tokens, lambda m: " " if m[0].startswith(("//", "/*")) else m[0], source,
                  flags=re.S)


def read_keys(source):
    source = without_comments(source)
    readers = {"get_val", "get_string", "get_double"}
    # Follow local key-reading wrappers, including resolve_nxmax's read_positive lambda.
    wrappers = re.findall(r'\bauto\s+(\w+)\s*=\s*\[[^]]*\]\s*\([^)]*\)\s*'
                          r'(?:->\s*\w+\s*)?\{([^{}]*)\}', source)
    while True:
        discovered = {name for name, body in wrappers
                      if re.search(r'\b(?:' + '|'.join(sorted(readers)) + r')\s*\(\s*key\b', body)}
        if discovered <= readers:
            break
        readers |= discovered
    keys = set(re.findall(r'\b(?:' + '|'.join(sorted(readers)) + r')\s*\(\s*"([^"\\]*)"', source))
    # Discover override-map variable names from parameters/declarations, rather than assuming "args".
    maps = set(re.findall(r'\bmap\s*<\s*(?:std::)?string\s*,\s*(?:std::)?string\s*>\s*(?:[&*]\s*)?(\w+)', source))
    if maps:
        keys |= set(re.findall(r'\b(?:' + '|'.join(sorted(maps)) + r')\s*\.\s*find\s*\(\s*"([^"\\]*)"', source))
    return keys


def main():
    root = pathlib.Path(__file__).resolve().parent.parent
    header = without_comments((root / "include/acquisition_protocol.h").read_text())
    names = re.findall(r'\{\s*"([A-Z][A-Z0-9_]*)"\s*,\s*KeyScope::', header)
    table = set(names)
    if not table or len(names) != len(table):
        sys.exit("argument key table is empty or has duplicates")
    source = "\n".join((root / name).read_text() for name in ("main.cpp", "src/functions.cpp"))
    actual = read_keys(source)
    if actual != table:
        sys.exit("argument key mismatch: missing from table: " + ", ".join(sorted(actual - table)) +
                 "; table keys never read: " + ", ".join(sorted(table - actual)))
    # Check that additions through each supported access path are visible, without changing source files.
    for call in ('get_val("NEW_KEY", path, args)', 'get_string("NEW_KEY", path, args)',
                 'get_double("NEW_KEY", path, args, value)', 'read_positive("NEW_KEY", value)',
                 'args.find("NEW_KEY")', 'overrides.find("NEW_KEY")'):
        assert read_keys(source + "\n" + call) == actual | {"NEW_KEY"}, call
    assert read_keys(source + '\nget_val("lower_case", path, args)') == actual | {"lower_case"}
    assert actual != table | {"UNREAD_KEY"}
    assert read_keys(source + '\n// get_val("COMMENT_KEY", path, args)') == actual
    print("Argument key table matches all override reads.")


if __name__ == "__main__":
    main()
