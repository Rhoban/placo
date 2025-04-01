#!/usr/bin/env python3
import re
import inspect
import sys
import os
import argparse
import fnmatch
from doxygen_parse import Doxygen, DoxygenVariable, DoxygenFunction

# Translation for types from C++ to python
cxx_to_python_overrides: dict = {
    "std::string": "str",
    "double": "float",
    "int": "int",
    "bool": "bool",
    "void": "None",
    "Eigen::Matrix*": "numpy.ndarray",
    "Eigen::Vector*": "numpy.ndarray",
    "Eigen::Affine*": "numpy.ndarray",
}


class MethodArgument:
    """
    Argument of a method
    """

    def __init__(
        self,
        name: str,
        type: str = "",
        default_value: str | None = None,
        comment: str | None = None,
    ):
        self.name: str = name
        self.type: str = type
        self.default_value: str | None = default_value
        self.comment: str | None = comment


class MethodSignature:
    def __init__(self, name: str):
        self.name = name
        self.arguments: list[MethodArgument] = []
        self.return_type: str = "any"
        self.doc: str = ""
        self.static: bool = False

    def generate(self, prefix: str = "") -> str:
        """
        Generate python code for this method signature
        """
        str_definition = ""

        if self.static:
            str_definition += f"{prefix}@staticmethod\n"

        str_definition += f"{prefix}def {self.name}(\n"
        for argument in self.arguments:
            extra_comment = ""
            str_definition += f"{prefix}  {argument.name}"
            if argument.type:
                str_definition += f": {argument.type}"
            if argument.default_value is not None:
                if argument.type in ["str", "float", "int", "bool"]:
                    str_definition += f" = {argument.default_value.capitalize()}"
                else:
                    str_definition += f" = None"
                    extra_comment = f" (default: {argument.default_value})"
            str_definition += ","
            if argument.comment is not None:
                str_definition += f" # {argument.comment}{extra_comment}"
            str_definition += "\n"
        str_definition += f"{prefix}) -> {self.return_type}:\n"

        if self.doc != "":
            str_definition += f'{prefix}  """' + "\n"
            for line in self.doc.strip().split("\n"):
                str_definition += f"{prefix}  " + line + "\n"
            str_definition += f'{prefix}  """' + "\n"

        str_definition += f"{prefix}  ...\n\n"

        return str_definition


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

    def cxx_type_to_py(self, typename: str):
        """
        We apply here some heuristics to rewrite C++ types to Python
        """
        if typename is not None:
            # Removing references, pointers and const
            typename = typename.replace("&", "")
            typename = typename.replace("*", "")
            typename = typename.strip()
            if typename.startswith("const "):
                typename = typename[6:]

            # Searching for a match in the mapping
            for cxx_type, python_type in self.cxx_to_python.items():
                if fnmatch.fnmatch(typename, cxx_type):
                    return python_type

            # Manually handling special case of vector and map template types
            result = re.match(r"^(.+)<(.+)>$", typename)
            if result:
                tpl_class = result.group(1).strip()
                tpl_args = [arg.strip() for arg in result.group(2).strip().split(",")]

                if tpl_class == "std::vector" and len(tpl_args) == 1:
                    return "list[" + self.cxx_type_to_py(tpl_args[0]) + "]"
                elif tpl_class == "std::map" and len(tpl_args) == 2:
                    return (
                        "dict["
                        + self.cxx_type_to_py(tpl_args[0])
                        + ", "
                        + self.cxx_type_to_py(tpl_args[1])
                        + "]"
                    )

        return "any"

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

                        if doxygen_class.brief:
                            self.stubs += f'  """' + "\n"
                            self.stubs += f"  {doxygen_class.brief.strip()}\n"
                            self.stubs += f'  """' + "\n"

                # Walking through class members
                for _name, _object in inspect.getmembers(object):
                    if not _name.startswith("_") or _name == "__init__":
                        if callable(_object):
                            # Class method
                            self.process_class_method(
                                class_name, _name, _object.__doc__
                            )
                        else:
                            # Class variable
                            self.process_class_variable(class_name, _name)
                self.stubs += "\n"
            elif callable(object):
                # Object is a function
                self.process_method(name, object.__doc__)
                self.stubs += "\n"
            else:
                # Else, doing nothing
                ...

    def parse_boost_python_docstring(self, name: str, doc: str) -> MethodSignature:
        """
        Parses the docstring prototypes produced by Boost.Python to infer types
        """
        signature = MethodSignature(name)

        doc = doc.replace("[", "").replace("]", "").strip()

        result = re.match(
            r"^([^\n]*?)\(([^\n]+)\) -> ([^\n]+) :(.*?)C\+\+ signature",
            doc,
            re.MULTILINE + re.DOTALL,
        )

        if result:
            signature.name = result.group(1)
            signature.return_type = result.group(3)
            signature.doc = result.group(4).strip()

            args = result.group(2).split(",")
            for arg in args:
                arg = arg.strip()
                arg = arg[1:]
                arg_type = ")".join(arg.split(")")[:-1])
                arg_name = arg.split(")")[-1]
                argument = MethodArgument(arg_name, arg_type)
                signature.arguments.append(argument)

        return signature

    def build_signature_from_doxygen(
        self, function: DoxygenFunction
    ) -> MethodSignature:
        signature = MethodSignature(function.name)
        signature.static = function.static

        # Method arguments
        if function.class_member:
            signature.arguments.append(MethodArgument("self"))
        for parameter in function.params:
            argument = MethodArgument(
                parameter.name,
                self.cxx_type_to_py(parameter.type),
                parameter.default_value,
                parameter.type,
            )
            signature.arguments.append(argument)

        # Return type
        signature.return_type = self.cxx_type_to_py(function.type)

        # Brief
        if function.brief:
            signature.doc += function.brief.strip() + "\n"

        # Parameters description
        for parameter in function.params:
            if parameter.description:
                py_type = self.cxx_type_to_py(parameter.type)
                signature.doc += (
                    f"\n:param {py_type} {parameter.name}: {parameter.description}"
                )

        # Returns description
        if function.returns_description:
            signature.doc += f"\n:return: {function.returns_description}"

        return signature

    def process_class_method(self, class_name: str, method_name: str, doc: str):
        function = None

        if class_name in self.python_to_cxx:
            if method_name == "__init__":
                function = self.doxygen.get_class_function(
                    self.python_to_cxx[class_name], class_name
                )
            else:
                function = self.doxygen.get_class_function(
                    self.python_to_cxx[class_name], method_name
                )

        if function is not None:
            signature = self.build_signature_from_doxygen(function)
        else:
            signature = self.parse_boost_python_docstring(method_name, doc)

        self.stubs += signature.generate("  ")

    def process_method(self, function_name: str, doc: str):
        function = self.doxygen.get_function(function_name)

        if function is not None:
            signature = self.build_signature_from_doxygen(function)
        else:
            signature = self.parse_boost_python_docstring(function_name, doc)

        self.stubs += signature.generate()

    def process_class_variable(self, class_name: str, member_name: str):
        variable = None
        if class_name in self.python_to_cxx:
            variable = self.doxygen.get_class_variable(
                self.python_to_cxx[class_name], member_name
            )

        if variable is not None:
            py_type = self.cxx_type_to_py(variable.type)
            self.stubs += f"  {member_name}: {py_type} # {variable.type}\n"
            if variable.brief:
                self.stubs += f'  """' + "\n"
                self.stubs += f"  {variable.brief.strip()}\n"
                self.stubs += f'  """' + "\n"
        else:
            self.stubs += f"  {member_name}: any\n"

        self.stubs += "\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="doxystubs")
    parser.add_argument("module")
    parser.add_argument("doxygen_directory")
    parser.add_argument("output_pyi")
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

    with open(args.output_pyi, "w") as file:
        file.write(doxy_stubs.stubs)
