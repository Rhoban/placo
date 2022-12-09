#!/usr/bin/env python
import re
import inspect
import placo
import os

def parse_doc(name: str, doc: str) -> dict:
    definition = {"name": name, "args": [], "returns": "Any"}

    prototype = doc.strip().split("\n")[0].strip()

    result = re.fullmatch(r"^(.*?)\((.+)\) -> (.+) :$", prototype)
    if result:
        definition["name"] = result.group(1)
        definition["returns"] = result.group(3)

        args = result.group(2).split(",")
        for arg in args:
            arg = arg.strip()
            arg = arg[1:]
            arg_type = ")".join(arg.split(")")[:-1])
            arg_name = arg.split(")")[-1]
            definition["args"].append([arg_type, arg_name])

    return definition


def print_def(name: str, doc: str, prefix: str = ""):
    definition = parse_doc(name, doc)
    str_definition = prefix
    str_definition += f"def {definition['name']}("
    str_definition += ",".join([f"{arg_name}: {arg_type}" for arg_type, arg_name in definition["args"]])
    str_definition += f") -> {definition['returns']}: ..."
    print(str_definition)


for name, object in inspect.getmembers(placo):
    if isinstance(object, type):
        print(f"class {object.__name__}:")
        for _name, _object in inspect.getmembers(object):
            if not _name.startswith("_"):
                if callable(_object):
                    print_def(_name, _object.__doc__, "  ")
                else:
                    print(f"  {_name}: Any")
        print("")
    elif callable(object):
        print_def(name, object.__doc__)
        print("")
    else:
        ...

