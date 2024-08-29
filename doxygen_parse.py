import glob
import re
from typing import Union
import xml.etree.ElementTree as ET

# Mapping member definitions (id) to all informations
member_definitions: dict = {}

# Compound members
compound_members: dict = {}
compound_metadata: dict = {}

# Mapping doxygen IDs to C++ compound names
doxygen_id_to_name: dict = {}


def resolve_type(node):
    type_node = node.find("type")

    if type_node.find("ref") is not None:
        if type_node.text and "vector" in type_node.text:
          return ["std::vector", type_node.find("ref").attrib["refid"]]
        else:
          return type_node.find("ref").attrib["refid"]
    else:
        return type_node.text


def element_to_string(element):
    s = element.text or ""
    for sub_element in element:
        s += ET.tostring(sub_element).decode("utf-8")
    s += element.tail
    return re.sub('<[^<]+?>', '', s)


def parse_compound(compounddef_node: ET.Element):
    global member_definitions, compound_metadata, compound_members

    name = compounddef_node.find("compoundname").text
    compound_kind = compounddef_node.attrib["kind"]
    if compound_kind == "namespace":
        name = "root"
    id = compounddef_node.attrib["id"]
    doxygen_id_to_name[id] = name

    if name not in compound_members:
        compound_members[name] = []

    compound_metadata[name] = {
        "id": id,
        "kind": compound_kind,
        "name": name,
        "brief": element_to_string(compounddef_node.find("briefdescription/para"))
        if compounddef_node.find("briefdescription/para") is not None
        else None,
    }

    # Searching for member definitions
    for member in compounddef_node.findall("sectiondef/memberdef"):
        kind = member.attrib["kind"]
        id = member.attrib["id"]
        static = member.attrib["static"] == "yes"

        # PyIgnore ?
        ignore = False
        sects = member.findall("detaileddescription/para/xrefsect")
        if sects:
            for sect in sects:
                if "pyignore" in sect.attrib["id"]:
                    ignore = True
        if ignore:
            continue

        if kind in ["function", "variable", "namespace"]:
            member_definitions[id] = {
                "kind": kind,
                "name": member.find("name").text,
                "params": [],
                "static": static,
            }
            member_definitions[id]["type"] = resolve_type(member)

            for param in member.findall("param"):
                param_type = resolve_type(param)
                param_name = None
                if param.find("declname") is not None:
                    param_name = param.find("declname").text
                if param.find("defname") is not None:
                    param_name = param.find("defname").text
                def_val = (
                    param.find("defval").text
                    if param.find("defval") is not None
                    else None
                )
                member_definitions[id]["params"].append(
                    {"type": param_type, "name": param_name, "default": def_val}
                )

            # Brief description
            brief = member.find("briefdescription/para")
            if brief is not None:
                member_definitions[id]["brief"] = element_to_string(brief)

            # Detailed description
            member_definitions[id]["detailed"] = []
            for entry in member.findall(
                "detaileddescription/para/parameterlist/parameteritem"
            ):
                param_name = entry.find("parameternamelist/parametername").text
                param_desc = element_to_string(entry.find("parameterdescription/para"))
                member_definitions[id]["detailed"].append(
                    {"name": param_name, "desc": param_desc}
                )

            # Verbatim
            verbatim = member.find("detaileddescription/para/verbatim")
            if verbatim is not None:
                member_definitions[id]["verbatim"] = verbatim.text

            # Return type
            return_type = member.find(
                "detaileddescription/para/simplesect[@kind='return']"
            )
            if return_type:
                member_definitions[id]["returns"] = element_to_string(return_type.find("para"))

            if compound_kind == "namespace":
                compound_members[name].append(id)

        elif kind == "typedef":
            doxygen_id_to_name[id] = resolve_type(member)
        elif kind == "enum":
            doxygen_id_to_name[id] = name + "::" + member.find("name").text

    # Listing all members
    kind = compounddef_node.attrib["kind"]
    for member in compounddef_node.findall("listofallmembers/member"):
        compound_members[name].append(member.attrib["refid"])


def parse_xml(xml_file: str):
    with open(xml_file, "r") as f:
        xml_content = f.read()
        xml_content = ET.fromstring(xml_content)

        # Searching for compounddef nodes (classs & struct)
        for compounddef_node in xml_content.findall("compounddef"):
            parse_compound(compounddef_node)


def resolve_doxygen_id(id: Union[list,str]):
    if type(id) == list:
        tpl = resolve_doxygen_id(id[0])
        typ = resolve_doxygen_id(id[1])
        return f"{tpl}<{typ}>"
    elif id in doxygen_id_to_name:
        return doxygen_id_to_name[id]
    else:
        return id


def parse_directory(directory):
    global member_definitions, compound_members
    doxygen_xml_files = glob.glob(f"{directory}/xml/*.xml")

    for xml_file in doxygen_xml_files:
        parse_xml(xml_file)

    # Resolving types
    for member in member_definitions:
        member_definition = member_definitions[member]
        member_definition["type"] = resolve_doxygen_id(member_definition["type"])

        for param in member_definition["params"]:
            param["type"] = resolve_doxygen_id(param["type"])

    # Resolving compound members
    for name in compound_members:
        temp = compound_members[name]
        compound_members[name] = {}
        for id in temp:
            if id in member_definitions:
                compound_members[name][
                    member_definitions[id]["name"]
                ] = member_definitions[id]


def get_members(name: str):
    if name not in compound_members:
        return None

    return compound_members[name]


def get_metadata(name: str):
    if name not in compound_metadata:
        return None

    return compound_metadata[name]
