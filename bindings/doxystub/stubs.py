#!/usr/bin/env python3
import re
import inspect
import sys
import os
import argparse
from doxygen_parse import Doxygen

# Translation for types from C++ to python
cxx_to_python_overrides: dict = {
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


def cxx_type_to_py(typename: str):
    """
    We apply here some heuristics to rewrite C++ types to Python
    """
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
        extra_comment = ""
        str_definition += f"{prefix}  {arg_name}: {arg_type}"
        if defvalue is not None:
            if arg_type in ["str", "float", "int", "bool"]:
                str_definition += f" = {defvalue.capitalize()}"
            else:
                str_definition += f" = None"
                extra_comment = f" (default: {defvalue})"
        str_definition += ","
        if comment is not None:
            str_definition += f" # {comment}{extra_comment}"
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
            print(f'  """{member["brief"].strip()}"""')
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


class DoxyStubs:
    def __init__(self, module_name: str, doxygen_directory: str):
        self.module_name: str = module_name
        self.doxygen_directory: str = doxygen_directory
        self.stubs: str = ""
        self.groups: dict = {}
        self.doxygen: Doxygen = Doxygen()

        # C++/Python types conversion mapping
        self.cxx_to_python: dict = cxx_to_python_overrides.copy()
        self.python_to_cxx: dict = {}

    def process(self):
        # Load the module to stub
        exec(f"import {self.module_name}")
        self.module = eval(f"{self.module_name}")

        # Running & parsing Doxygen
        # self.run_doxygen()
        self.doxygen.parse_directory(self.doxygen_directory)

        # Headers and forward types declaration
        self.stubs += "# Doxygen stubs generation\n"
        self.stubs += "import numpy\n"
        self.stubs += "import typing\n"

        for _, object in inspect.getmembers(self.module):
            if isinstance(object, type):
                class_name = object.__name__
                self.stubs += (
                    f'{class_name} = typing.NewType("{class_name}", None)' + "\n"
                )

        # Building registry and reverse registry for class names
        cxx_registry = self.module.get_classes_registry()
        for cxx_type, python_type in cxx_registry.items():
            self.cxx_to_python[cxx_type] = python_type
            self.python_to_cxx[python_type] = cxx_type

        # Processing module members
        self.process_module_members()

        # Printing namespace groups
        self.stubs += f"__groups__ = {self.groups}\n"

    def run_doxygen(self):
        """
        Ensure Doxygen is run
        """
        if not os.path.exists(f"/usr/bin/doxygen"):
            sys.stderr.write("\n-----------------------\n")
            sys.stderr.write("WARNING: Doxygen is not installed\n")
            sys.stderr.write("         you should run: sudo apt install doxygen\n")
            sys.stderr.write("-----------------------\n\n")
            exit(1)

        os.system(f"cd {self.doxygen_directory} && doxygen 1>&2")

    def process_module_members(self):
        """
        Parse members available in the module and generate stubs
        """
        for name, object in inspect.getmembers(self.module):
            if isinstance(object, type):
                # Object is a class
                class_name = object.__name__
                self.stubs += f"class {class_name}:\n"

                if class_name in self.python_to_cxx:
                    doxygen_class = self.doxygen.get_class(
                        self.python_to_cxx[class_name]
                    )

                    # Adding the class to related groups
                    if doxygen_class is not None:
                        namespace = "::".join(doxygen_class.name.split("::")[:-1][:2])
                        if namespace not in self.groups:
                            self.groups[namespace] = []
                        self.groups[namespace].append(class_name)

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
                # Object is a function
                print_class_method("root", name, object.__doc__)
                print("")
            else:
                # Else, doing nothing
                ...


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="onshape-to-robot-bullet")
    parser.add_argument("module")
    parser.add_argument("doxygen_directory")
    args = parser.parse_args()

    # If .pyi file already exists next to stubs.py, we read it directly. This is a way to
    # avoid running Doxygen when building sdist release.
    if os.path.exists(f"{args.doxygen_directory}/{args.module}.pyi"):
        with open(f"{args.doxygen_directory}/{args.module}.pyi", "r") as f:
            print(f.read())
        exit(0)

    # Prepending current directory to PYTHONPATH
    sys.path = ["."] + sys.path

    doxy_stubs = DoxyStubs(args.module, args.doxygen_directory)
    doxy_stubs.process()
