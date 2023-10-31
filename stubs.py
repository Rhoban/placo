#!/usr/bin/env python
import re
import inspect
import placo
import os
import glob
from doxygen_parse import parse_directory, get_members, get_metadata, rewrite_types

module: str = "placo"

# Current script directory:
repo_directory = os.path.dirname(os.path.realpath(__file__))

# Ensure Doxygen is run
os.system(f"cd {repo_directory} && doxygen 1>&2")

# Read the Doxygen XML file
parse_directory(repo_directory)

# Building registry and reverse registry for class names
cxx_registry = placo.get_classes_registry()
py_registry = {module: module}
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
        return re.sub('[^a-zA-Z0-9\.]', '_', cxx_type)


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
        str_definition = f"{prefix}def {method_name}("

        # Method arguments
        params = [f"{arg['name']}: {cxx_type_to_py(arg['type'])}" for arg in member["params"]]
        if class_name != module:
            params = ["self: " + class_name] + params
        str_definition += ",".join(params)

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
                brief_str += f"\n{prefix}  :param {param['name']}: {param['desc']}"

        if "verbatim" in member:
            brief_str += f"\n{prefix}  {member['verbatim']}"

        if "returns" in member:
            brief_str += f"\n{prefix}  :return: {member['returns']}"

        if brief_str:
            str_definition += f'{prefix}  """\n{prefix}  {brief_str}\n{prefix}  """\n'
        str_definition += f"{prefix}  ...\n"

        print(str_definition)
    else:
        print_def(method_name, doc, prefix)


print("import numpy")

for name, object in inspect.getmembers(placo):
    if isinstance(object, type):
        class_name = object.__name__
        print(f"class {class_name}:")

        if class_name in py_registry:
            metadata = get_metadata(py_registry[class_name])
            if metadata is not None and "brief" in metadata:
                print(f"  \"\"\"{metadata['brief']}\"\"\"")

        for _name, _object in inspect.getmembers(object):
            if not _name.startswith("_") or _name == "__init__":
                if callable(_object):
                    print_class_method(class_name, _name, _object.__doc__, "  ")
                else:
                    print_class_member(class_name, _name)
        print("")
    elif callable(object):
        print_class_method(module, name, object.__doc__)
        print("")
    else:
        ...
