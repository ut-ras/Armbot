import xml.etree.ElementTree as ET
from io import StringIO

# Helper function to merge two XML elements
def merge_xml_elements(base_elem, new_elem):
    for base_child, new_child in zip(base_elem, new_elem):
        # If the child has further children, recurse
        if len(base_child) > 0:
            merge_xml_elements(base_child, new_child)
        else:
            # Merge text of the elements by adding them with a space if they are different
            if base_child.text != new_child.text:
                base_child.text += ' ' + new_child.text

# The XML content goes between the triple quotes
xml1_content = '''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<group_gains>
  <control_strategy>4</control_strategy>
  <position>
    <kp>30</kp>
    <ki>0</ki>
    <kd>0</kd>
    <feed_forward>0</feed_forward>
    <dead_zone>0</dead_zone>
    <i_clamp>1</i_clamp>
    <punch>0</punch>
    <min_target>-inf</min_target>
    <max_target>inf</max_target>
    <target_lowpass>1</target_lowpass>
    <min_output>-10</min_output>
    <max_output>10</max_output>
    <output_lowpass>1</output_lowpass>
    <d_on_error>1</d_on_error>
  </position>
  <velocity>
    <kp>0.05</kp>
    <ki>0</ki>
    <kd>0</kd>
    <feed_forward>1</feed_forward>
    <dead_zone>0</dead_zone>
    <i_clamp>0.25</i_clamp>
    <punch>0</punch>
    <min_target>-3.434687</min_target>
    <max_target>3.434687</max_target>
    <target_lowpass>1</target_lowpass>
    <min_output>-1</min_output>
    <max_output>1</max_output>
    <output_lowpass>0.75</output_lowpass>
    <d_on_error>1</d_on_error>
  </velocity>
  <effort>
    <kp>0</kp>
    <ki>0</ki>
    <kd>0</kd>
    <feed_forward>1</feed_forward>
    <dead_zone>0</dead_zone>
    <i_clamp>0.25</i_clamp>
    <punch>0</punch>
    <min_target>-10</min_target>
    <max_target>10</max_target>
    <target_lowpass>1</target_lowpass>
    <min_output>-0.2</min_output>
    <max_output>0.2</max_output>
    <output_lowpass>0.9</output_lowpass>
    <d_on_error>0</d_on_error>
  </effort>
</group_gains>
'''
xml2_content = '''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<group_gains>
  <control_strategy>4</control_strategy>
  <position>
    <kp>25</kp>
    <ki>0</ki>
    <kd>0</kd>
    <feed_forward>0</feed_forward>
    <dead_zone>0</dead_zone>
    <i_clamp>1</i_clamp>
    <punch>0</punch>
    <min_target>-inf</min_target>
    <max_target>inf</max_target>
    <target_lowpass>1</target_lowpass>
    <min_output>-10</min_output>
    <max_output>10</max_output>
    <output_lowpass>1</output_lowpass>
    <d_on_error>1</d_on_error>
  </position>
  <velocity>
    <kp>0.05</kp>
    <ki>0</ki>
    <kd>0</kd>
    <feed_forward>1</feed_forward>
    <dead_zone>0</dead_zone>
    <i_clamp>0.25</i_clamp>
    <punch>0</punch>
    <min_target>-3.434687</min_target>
    <max_target>3.434687</max_target>
    <target_lowpass>1</target_lowpass>
    <min_output>-1</min_output>
    <max_output>1</max_output>
    <output_lowpass>0.75</output_lowpass>
    <d_on_error>1</d_on_error>
  </velocity>
  <effort>
    <kp>0.24</kp>
    <ki>0</ki>
    <kd>0.001</kd>
    <feed_forward>1</feed_forward>
    <dead_zone>0</dead_zone>
    <i_clamp>0.25</i_clamp>
    <punch>-0</punch>
    <min_target>-10</min_target>
    <max_target>10</max_target>
    <target_lowpass>1</target_lowpass>
    <min_output>-1</min_output>
    <max_output>1</max_output>
    <output_lowpass>0.9</output_lowpass>
    <d_on_error>0</d_on_error>
  </effort>
</group_gains>
'''

# Parse the XML content into elements
xml1_root = ET.parse(StringIO(xml1_content)).getroot()
xml2_root = ET.parse(StringIO(xml2_content)).getroot()

# Merge xml2_root into xml1_root
merge_xml_elements(xml1_root, xml2_root)

# Print out the merged XML
merged_xml_str = ET.tostring(xml1_root).decode()
print(merged_xml_str)
