import sys
import xml.dom.minidom

def format_xml(xml_string):
    dom = xml.dom.minidom.parseString(xml_string)
    
    def is_joint_or_link(node):
        return node.nodeType == node.ELEMENT_NODE and (node.tagName == "joint" or node.tagName == "link")
    
    def add_newline(node):
        node.parentNode.insertBefore(dom.createTextNode("\n"), node)
        node.parentNode.insertBefore(dom.createTextNode("\n"), node.nextSibling)
    
    for node in dom.getElementsByTagName("*"):
        if is_joint_or_link(node):
            add_newline(node)
    
    formatted_xml = dom.toprettyxml(indent="  ", newl="")
    formatted_xml = "\n".join(line for line in formatted_xml.split("\n") if line.strip())
    return formatted_xml

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 fix_urdf.py <input_file>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    if not input_file.endswith(".urdf"):
        print("Input file must have a .urdf extension")
        sys.exit(1)
    
    try:
        with open(input_file, "r") as file:
            xml_string = file.read()
    except FileNotFoundError:
        print(f"File not found: {input_file}")
        sys.exit(1)
    
    formatted_xml = format_xml(xml_string)
    
    output_file = input_file[:-5] + "_formatted.urdf"
    with open(output_file, "w") as file:
        file.write(formatted_xml)
    
    print(f"Formatted URDF file saved as: {output_file}")

if __name__ == "__main__":
    main()