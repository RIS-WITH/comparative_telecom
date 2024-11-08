import os
import rclpy
from rclpy.node import Node
from subprocess import check_output, CalledProcessError
import importlib
from rosidl_adapter.parser import parse_message_file

class TopicServiceLister(Node):
    def __init__(self):
        super().__init__('topic_service_lister')

    def get_topic_or_service_hash(self, name, is_topic=True):
        try:
            cmd = ['ros2', 'topic', 'info', name, '-v'] if is_topic else ['ros2', 'service', 'info', name, '-v']
            output = check_output(cmd, text=True)
            for line in output.splitlines():
                if 'Topic type hash:' in line or 'Service type hash:' in line:
                    return line.split()[-1]
        except CalledProcessError:
            return 'unknown_hash'

    def convert_to_dds_c_style(self, msg_type):
        # Extraction du nom du type pour correspondre au format DDS C-style
        msg_type_parts = msg_type.split('/')
        if len(msg_type_parts) == 3:
            dds_type = f'{msg_type_parts[0]}::msg::dds_::{msg_type_parts[2]}_'
        else:
            dds_type = msg_type
        return dds_type

    def get_message_fields(self, msg_type):
        parts = msg_type.split('/')
        if len(parts) != 3:
            return None

        package, _, msg_name = parts
        msg_file = os.path.join('/opt/ros/iron/share', package, 'msg', f'{msg_name}.msg')

        try:
            # Use rosidl_adapter to parse the message definition
            message_spec = parse_message_file(package, msg_file)
            fields = [(field.name, field.type.type) for field in message_spec.fields]
            # print the fields
            return fields
        except Exception as e:
            print(f"Failed to parse message file {msg_file}: {e}")
            return None

    def generate_js_class(self, msg_type):
        fields = self.get_message_fields(msg_type)
        if not fields:
            return ""

        class_template = """
// ROS2 {type} type
class {type} {{
    constructor({fields}) {{
        {field_assignments}
    }}

    encode(cdrWriter) {{
        {encode_fields}
    }}

    static decode(cdrReader) {{
        {decode_fields}
        return new {type}({constructor_fields});
    }}
}}
"""
        type_parts = msg_type.split('/')
        type_name = type_parts[2]
        field_names = [field[0] for field in fields]
        field_assignments = "\n        ".join([f"this.{f} = {f};" for f in field_names])
        encode_fields = "\n        ".join([f"cdrWriter.write{field[1].capitalize()}(this.{field[0]});" for field in fields])
        decode_fields = "\n        ".join([f"let {field[0]} = cdrReader.read{field[1].capitalize()}();" for field in fields])
        constructor_fields = ", ".join(field_names)

        return class_template.format(type=type_name, fields=", ".join(field_names),
                                     field_assignments=field_assignments, encode_fields=encode_fields,
                                     decode_fields=decode_fields, constructor_fields=constructor_fields)

    def list_topics_and_services(self):
        ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')
        topics = self.get_topic_names_and_types()
        services = self.get_service_names_and_types()

        topic_list = []
        for topic_name, topic_types in topics:
            for topic_type in topic_types:
                dds_topic_type = self.convert_to_dds_c_style(topic_type)
                topic_hash = self.get_topic_or_service_hash(topic_name, is_topic=True)
                keyexpr = f"{ros_domain_id}/{topic_name}/{dds_topic_type}/{topic_hash}"
                topic_list.append(keyexpr)
                
                # Generate JavaScript class
                js_class = self.generate_js_class(topic_type)
                if js_class:
                    with open(f"{topic_name.replace('/', '_')}_{topic_type.replace('/', '_')}.js", 'w') as f:
                        f.write(js_class)

        service_list = []
        # for service_name, service_types in services:
        #     for service_type in service_types:
        #         dds_service_type = self.convert_to_dds_c_style(service_type)
        #         service_hash = self.get_topic_or_service_hash(service_name, is_topic=False)
        #         keyexpr = f"{ros_domain_id}/{service_name}/{dds_service_type}/{service_hash}"
        #         service_list.append(keyexpr)

        #         # Generate JavaScript class
        #         js_class = self.generate_js_class(service_type)
        #         if js_class:
        #             with open(f"{service_name.replace('/', '_')}_{service_type.replace('/', '_')}.js", 'w') as f:
        #                 f.write(js_class)

        return topic_list, service_list

def main(args=None):
    rclpy.init(args=args)
    node = TopicServiceLister()
    topics, services = node.list_topics_and_services()

    print("Topics:")
    for topic in topics:
        print(topic)

    # print("\nServices:")
    # for service in services:
    #     print(service)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
