#!/usr/bin/env python
import re
import inspect
import placo
import os
import glob
from doxygen_parse import parse_directory, get_members, rewrite_types

# Current script directory:
repo_directory = os.path.dirname(os.path.realpath(__file__))

# Ensure Doxygen is run
os.system(f"cd {repo_directory} && doxygen 1>&2")

# Read the Doxygen XML file
parse_directory(repo_directory)

# Building registry and reverse registry for class names
cxx_registry = placo.get_classes_registry()
py_registry = {}
for entry in cxx_registry:
    py_registry[cxx_registry[entry]] = entry


def get_member(class_name: str, member_name: str):
    if class_name in py_registry:
        cxx_name = py_registry[class_name]
        members = get_members(cxx_name)
        if members is not None and member_name in members:
            return members[member_name]

    return None


def cxx_type_to_py(cxx_type: str) -> str:
    if cxx_type in cxx_registry:
        return cxx_registry[cxx_type]
    elif cxx_type in rewrite_types:
        return rewrite_types[cxx_type]
    else:
        return cxx_type.replace(":", "_")


def parse_doc(name: str, doc: str) -> dict:
    definition = {"name": name, "args": [], "returns": "any"}

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


def print_class_member(class_name: str, member_name: str):
    member = get_member(class_name, member_name)

    if member is not None:
        py_type = cxx_type_to_py(member["type"])
        print(f"  {member_name}: {py_type}")
        if "brief" in member:
            print(f"  \"\"\"{member['brief']}\"\"\"")
    else:
        print(f"  {member_name}: any")


def print_class_method(class_name: str, method_name: str, doc: str, prefix: str = ""):
    if method_name == "__init__":
        member = get_member(class_name, class_name)
    else:
        member = get_member(class_name, method_name)

    if member is not None:
        # Method name
        str_definition = f"  def {method_name}("

        # Method arguments
        str_definition += ",".join([f"self: {class_name}"] + [f"{arg['name']}: {cxx_type_to_py(arg['type'])}" for arg in member["params"]])

        # Return type
        str_definition += f")"
        if member["type"]:
            py_type = cxx_type_to_py(member["type"])
            str_definition += f" -> {py_type}"
        str_definition += ":\n"

        # Brief
        brief_str = ""
        if "brief" in member:
            brief_str = member["brief"] + "\n"

        if "detailed" in member:
            for param in member["detailed"]:
                brief_str += f"\n    :param {param['name']}: {param['desc']}"

        if "returns" in member:
            brief_str += f"\n    :return: {member['returns']}"

        if brief_str:
            str_definition += f'    """\n    {brief_str}\n    """\n'
        str_definition += "    ...\n"

        print(str_definition)
        # py_type = cxx_type_to_py(member['type'])
        # print(f"  def {method_name}() -> {py_type}: ...")
    else:
        print_def(method_name, doc, prefix)


print("import numpy as np")

for name, object in inspect.getmembers(placo):
    if isinstance(object, type):
        class_name = object.__name__
        print(f"class {class_name}:")
        for _name, _object in inspect.getmembers(object):
            if not _name.startswith("_") or _name == "__init__":
                if callable(_object):
                    print_class_method(class_name, _name, _object.__doc__, "  ")
                else:
                    print_class_member(class_name, _name)
        print("")
    elif callable(object):
        print_def(name, object.__doc__)
        print("")
    else:
        ...
