import glob
import re
from typing import Union
import xml.etree.ElementTree as ET


def element_to_string(element: ET.Element) -> str:
    """
    Retrieve the contents of a given node
    """
    s = element.text or ""
    for sub_element in element:
        s += ET.tostring(sub_element).decode("utf-8")
    s += element.tail
    return re.sub("<[^<]+?>", "", s)


def parse_type(node: ET.Element):
    """
    Retrieve the type of the object present in the given node
    """
    type_node = node.find("type")

    if type_node.find("ref") is not None:
        if type_node.text and "vector" in type_node.text:
            return ["std::vector", type_node.find("ref").attrib["refid"]]
        else:
            return type_node.find("ref").attrib["refid"]
    else:
        return type_node.text


class DoxygenElement:
    def __init__(self, id: str, name: str):
        self.id = id
        self.name: str = name


class DoxygenTypedef(DoxygenElement):
    pass


class DoxygenMember(DoxygenElement):
    def __init__(self, id: str, name: str):
        super().__init__(id, name)
        self.static: bool = False
        self.type: str | None = None
        self.brief: str = ""

    def parse(self, node: ET.Element):
        self.static = node.attrib["static"] == "yes"

        self.type = parse_type(node)

        # Brief description
        self.brief = node.find("briefdescription/para")
        if self.brief is not None:
            self.brief = element_to_string(self.brief)

        # Verbatim
        verbatim = node.find("detaileddescription/para/verbatim")
        if verbatim is not None:
            self.brief += "\n" + verbatim.text


class DoxygenVariable(DoxygenMember):
    pass


class DoxygenParameter:
    def __init__(self):
        self.type = None
        self.name = None
        self.default_value = None
        self.description = None

    def parse(self, node: ET.Element):
        self.type = parse_type(node)

        if node.find("declname") is not None:
            self.name = node.find("declname").text
        if node.find("defname") is not None:
            self.name = node.find("defname").text

        self.default_value = (
            node.find("defval").text if node.find("defval") is not None else None
        )


class DoxygenFunction(DoxygenMember):
    def __init__(self, id: str, name: str, class_member: bool = False):
        super().__init__(id, name)
        self.params: list[DoxygenParameter] = []
        self.returns_description = None
        self.class_member = class_member

    def parse(self, node: ET.Element):
        super().parse(node)

        # Searching for method parameters
        for param in node.findall("param"):
            parameter = DoxygenParameter()
            parameter.parse(param)
            self.params.append(parameter)

        # Searching for detailed description to retrieve parameter descriptions
        for entry in node.findall(
            "detaileddescription/para/parameterlist/parameteritem"
        ):
            param_name = entry.find("parameternamelist/parametername").text
            param_desc = element_to_string(entry.find("parameterdescription/para"))

            for param in self.params:
                if param.name == param_name:
                    param.description = param_desc

        # Description of method returns
        return_desc = node.find("detaileddescription/para/simplesect[@kind='return']")
        if return_desc:
            self.returns_description = return_desc.find("para")


class DoxygenCompound(DoxygenElement):
    def __init__(self, id: str, name: str):
        super().__init__(id, name)
        self.brief: str | None = None
        self.members: dict[str, DoxygenMember] = {}
        self.types: dict[str, str] = {}
        self.is_class: bool = False

    def parse(self, node: ET.Element):
        if node.find("briefdescription/para"):
            self.brief = element_to_string(node.find("briefdescription/para"))

        # Searching for member definitions
        for member_node in node.findall("sectiondef/memberdef"):
            # If @pyignore is added to the description, this member is skipped
            ignore = False
            sects = member_node.findall("detaileddescription/para/xrefsect")
            if sects:
                for sect in sects:
                    if "pyignore" in sect.attrib["id"]:
                        ignore = True
            if ignore:
                continue

            kind = member_node.attrib["kind"]
            id = member_node.attrib["id"]

            member = None
            if kind == "function":
                member = DoxygenFunction(
                    id, member_node.find("name").text, self.is_class
                )
                member.parse(member_node)
            elif kind == "variable":
                member = DoxygenVariable(id, member_node.find("name").text)
                member.parse(member_node)
            elif kind == "typedef":
                self.types[id] = DoxygenTypedef(id, parse_type(member_node))
            elif kind == "enum":
                self.types[id] = DoxygenTypedef(
                    id, self.name + "::" + member_node.find("name").text
                )

            if member is not None:
                self.members[id] = member


class DoxygenClass(DoxygenCompound):
    def __init__(self, id: str, name: str):
        super().__init__(id, name)
        self.is_class: bool = True


class DoxygenNamespace(DoxygenCompound):
    def __init__(self, id: str, name: str):
        super().__init__(id, name)


class Doxygen:
    def __init__(self):
        self.compounds: dict[str, DoxygenCompound] = {}
        self.elements: dict[str, DoxygenElement] = {}

    def parse_directory(self, directory: str):
        doxygen_xml_files = glob.glob(f"{directory}/xml/*.xml")

        # Parsing all files
        for xml_file in doxygen_xml_files:
            self.parse_xml(xml_file)

        # Collecting all elements (compounds, members and typedefs)
        self.elements = self.compounds.copy()
        for compound in self.compounds.values():
            for element in list(compound.members.values()) + list(
                compound.types.values()
            ):
                self.elements[element.id] = element

        # Resolving member and parameter types
        for compound in self.compounds.values():
            for member in compound.members.values():
                member.type = self.resolve_type(member.type)

                if isinstance(member, DoxygenFunction):
                    function: DoxygenFunction = member
                    for param in member.params:
                        param.type = self.resolve_type(param.type)

    def resolve_type(self, id: Union[list, str]):
        if type(id) == list:
            tpl = self.resolve_type(id[0])
            subtypes = [self.resolve_type(subid) for subid in id[1:]]
            args = ",".join(subtypes)
            return f"{tpl}<{args}>"
        elif id in self.elements:
            return self.elements[id].name
        else:
            return id

    def parse_xml(self, filename: str):
        with open(filename, "r") as f:
            xml_content = f.read()
            xml_content = ET.fromstring(xml_content)

            # Searching for compounddef nodes (classs & struct)
            for compounddef_node in xml_content.findall("compounddef"):
                self.parse_compound(compounddef_node)

    def parse_compound(self, compounddef_node: ET.Element):
        name = compounddef_node.find("compoundname").text
        compound_kind = compounddef_node.attrib["kind"]
        id = compounddef_node.attrib["id"]

        if compound_kind == "class" or compound_kind == "struct":
            compound = DoxygenClass(id, name)
        elif compound_kind == "namespace":
            compound = DoxygenNamespace(id, name)
        else:
            return

        compound.parse(compounddef_node)
        self.compounds[id] = compound

    def get_class(self, class_name: str) -> DoxygenClass | None:
        for compound in self.compounds.values():
            if compound.name == class_name:
                return compound

        return None

    def get_class_member(
        self, class_name: str, member_name: str
    ) -> DoxygenMember | None:
        class_ = self.get_class(class_name)

        if class_ is not None:
            for member in class_.members.values():
                if member.name == member_name:
                    return member

        return None

    def get_function(self, function_name: str) -> DoxygenFunction | None:
        for compound in self.compounds.values():
            if isinstance(compound, DoxygenNamespace):
                for member in compound.members.values():
                    if (
                        isinstance(member, DoxygenFunction)
                        and member.name == function_name
                    ):
                        return member

        return None

    def get_class_function(
        self, class_name: str, function_name: str
    ) -> DoxygenFunction | None:
        member = self.get_class_member(class_name, function_name)

        if isinstance(member, DoxygenFunction):
            return member

        return None

    def get_class_variable(
        self, class_name: str, function_name: str
    ) -> DoxygenVariable | None:
        member = self.get_class_member(class_name, function_name)

        if isinstance(member, DoxygenVariable):
            return member

        return None


if __name__ == "__main__":
    # parse_directory(".")
    doxygen = Doxygen()
    doxygen.parse_directory(".")
