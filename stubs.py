#!/usr/bin/env python
import re
import inspect
import sys
import os
import argparse

# Current script directory:
repo_directory = os.path.dirname(os.path.realpath(__file__))

# If placo.pyi file already exists next to stubs.py, we read it directly. This is a way to
# avoid running Doxygen when building placo sdist release.
if os.path.exists(f"{repo_directory}/placo.pyi"):
    with open(f"{repo_directory}/placo.pyi", "r") as f:
        print(f.read())
    exit(0)

# Prepending current directory to PYTHONPATH
sys.path = ["."] + sys.path

import placo
from doxygen_parse import parse_directory, get_members, get_metadata

module: str = "placo"

# Ensure Doxygen is run
if not os.path.exists(f"/usr/bin/doxygen"):
    sys.stderr.write("\n-----------------------\n")
    sys.stderr.write("WARNING: Doxygen is not installed\n")
    sys.stderr.write("         you should run: sudo apt install doxygen\n")
    sys.stderr.write("-----------------------\n\n")
    exit(1)

result = os.system(f"cd {repo_directory} && doxygen 1>&2")

# Read the Doxygen XML file
parse_directory(repo_directory)

# Translation for types from C++ to python
rewrite_types: dict = {
    "std::string": "str",
    "double": "float",
    "int": "int",
    "bool": "bool",
    "void": "None",
    "Eigen::MatrixXd": "numpy.ndarray",
    "Eigen::VectorXd": "numpy.ndarray",
    "Eigen::Matrix3d": "numpy.ndarray",
    "Eigen::Vector3d": "numpy.ndarray",
    "Eigen::Matrix2d": "numpy.ndarray",
    "Eigen::Vector2d": "numpy.ndarray",
    "Eigen::Affine3d": "numpy.ndarray",
}

# Building registry and reverse registry for class names
cxx_registry = placo.get_classes_registry()
py_registry = {"root": "root"}
for entry in cxx_registry:
    rewrite_types[entry] = cxx_registry[entry]
    py_registry[cxx_registry[entry]] = entry

def get_member(class_name: str, member_name: str):
    if class_name in py_registry:
        cxx_name = py_registry[class_name]
        members = get_members(cxx_name)
        if members is not None and member_name in members:
            return members[member_name]

    return None


def cxx_type_to_py(typename: str):
    """
    We apply here some heuristics to rewrite C++ types to Python
    """
    typename_raw = typename
    if typename is not None:
        typename = typename.replace("&", "")
        typename = typename.replace("*", "")
        typename = typename.strip()
        if typename.startswith("const "):
            typename = typename[6:]

        if typename in rewrite_types:
            return rewrite_types[typename]
        elif typename.startswith("std::vector"):
            return "list[" + cxx_type_to_py(typename[12:-1]) + "]"
        else:
            return f"any"

    return None


def parse_doc(name: str, doc: str) -> dict:
    """
    Parses the docstring prototypes produced by Boost.Python to infer types
    """
    definition = {"name": name, "args": [], "returns": "any"}

    prototype = doc.strip().split("\n")[0].strip()
    prototype = prototype.replace("[", "").replace("]", "")

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


def print_def_prototype(
    method_name: str,
    args: list,
    return_type: str = "any",
    doc="",
    prefix: str = "",
    static: bool = False,
):
    str_definition = ""

    if static:
        str_definition += f"{prefix}@staticmethod\n"

    str_definition += f"{prefix}def {method_name}(\n"
    for arg_name, arg_type, defvalue, comment in args:
        str_definition += f"{prefix}  {arg_name}: {arg_type}"
        if defvalue is not None:
            if arg_type in ["str", "float", "int", "bool"]:
                str_definition += f" = {defvalue.capitalize()}"
        str_definition += ","
        if comment is not None:
            str_definition += f" # {comment}"
        str_definition += "\n"
    str_definition += f"\n{prefix}) -> {return_type}:\n"
    if doc != "":
        str_definition += f'{prefix}  """{doc.strip()}"""\n'
    str_definition += f"{prefix}  ...\n"
    print(str_definition)


def print_def(name: str, doc: str, prefix: str = ""):
    definition = parse_doc(name, doc)

    print_def_prototype(
        definition["name"],
        [[arg_name, arg_type, None, None] for arg_type, arg_name in definition["args"]],
        definition["returns"],
        prefix=prefix,
    )


def print_class_member(class_name: str, member_name: str):
    member = get_member(class_name, member_name)

    if member is not None:
        py_type = cxx_type_to_py(member["type"])
        print(f"  {member_name}: {py_type} # {member['type']}")
        if "brief" in member:
            print(f'  """{member["brief"]}"""')
    else:
        print(f"  {member_name}: any")

    print("")


def print_class_method(class_name: str, method_name: str, doc: str, prefix: str = ""):
    if method_name == "__init__":
        member = get_member(class_name, class_name)
    else:
        member = get_member(class_name, method_name)

    if member is not None:
        static = member["static"]

        # Method arguments
        args = [
            [arg["name"], cxx_type_to_py(arg["type"]), arg["default"], arg["type"]]
            for arg in member["params"]
        ]
        if class_name != "root" and not member["static"]:
            args = [["self", class_name, None, None]] + args

        # Return type
        return_type = "any"
        if member["type"]:
            return_type = cxx_type_to_py(member["type"])

        # Brief
        doc = ""
        if "brief" in member and member["brief"] is not None:
            doc = member["brief"] + "\n"

        if "detailed" in member:
            for param in member["detailed"]:
                doc += f"\n{prefix}  :param {param['name']}: {param['desc']}"

        if "verbatim" in member:
            doc += f"\n{prefix}  {member['verbatim']}"

        if "returns" in member:
            doc += f"\n{prefix}  :return: {member['returns']}"

        print_def_prototype(
            method_name, args, return_type, doc=doc, prefix=prefix, static=static
        )
    else:
        print_def(method_name, doc, prefix)


print("import numpy")
print("import typing")

for name, object in inspect.getmembers(placo):
    if isinstance(object, type):
        class_name = object.__name__
        print(f"{class_name} = typing.NewType(\"{class_name}\", None)")

groups = {}

for name, object in inspect.getmembers(placo):
    if isinstance(object, type):
        class_name = object.__name__
        print(f"class {class_name}:")

        if class_name in py_registry:
            metadata = get_metadata(py_registry[class_name])

            if metadata is not None:
              namespace = "::".join(metadata['name'].split("::")[:-1][:2])
              if namespace not in groups:
                  groups[namespace] = []
              groups[namespace].append(class_name)

            if (
                metadata is not None
                and "brief" in metadata
                and metadata["brief"] is not None
            ):
                print(f"  \"\"\"{metadata['brief']}\"\"\"")

        for _name, _object in inspect.getmembers(object):
            if not _name.startswith("_") or _name == "__init__":
                if callable(_object):
                    print_class_method(class_name, _name, _object.__doc__, "  ")
                else:
                    print_class_member(class_name, _name)
        print("")
    elif callable(object):
        print_class_method("root", name, object.__doc__)
        print("")
    else:
        ...

print(f"__groups__ = {groups}")