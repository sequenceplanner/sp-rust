#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Er{\H o}s
    # SP model based Autogenerated ROS2 {{ resource_name }} simulator node using jinja2
    # V.1.0.2.
#----------------------------------------------------------------------------------------------------------------------#

import sys
import rclpy
import time
from rclpy.node import Node
from {{ package_name }}.msg import {{ message_type_interfacer_to_driver }}
from {{ package_name }}.msg import {{ message_type_driver_to_interfacer }}

class {{ capitalized_resource_name }}Simulator(Node):

    def __init__(self):
        super().__init__("{{ resource_name }}_simulator")
        
        self.msg_emulator_to_interfacer = {{ message_type_driver_to_interfacer }}()
        self.msg_interfacer_to_emulator = {{ message_type_interfacer_to_driver }}()

        self.driver_to_interfacer_tmr_period = 0.5

        # Be careful what init values are since they are being published immediately the node is up!
        {% for item in measured_variables %}
        self.{{ item }} = False
        {%- endfor %}
        {% for item in command_variables %}
        self.{{ item }} = False
        self.got_{{ item }} = False
        {%- endfor %}

        self.predicates = [{% for item in predicates -%}
                           '{{ item }}',
                           {% endfor -%}]

        self.actions = [{% for item in actions -%}
                        {{ item }},
                        {% endfor -%}]
        
        self.effects = [{% for item in effects -%}
                        {{ item }},
                        {% endfor -%}]

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.{{ resource_name }}_interfacer_to_driver_subscriber = self.create_subscription(
            {{ message_type_interfacer_to_driver }}, 
            "/{{ resource_name }}_interfacer_to_driver",
            self.{{ resource_name }}_interfacer_to_driver_callback,
            10)

        # Then sleep for a bit so that the node get the updated variables before publishing them
        time.sleep(2)

        self.{{ resource_name }}_driver_to_interfacer_publisher_ = self.create_publisher(
            {{ message_type_driver_to_interfacer }},
            "/{{ resource_name }}_driver_to_interfacer",
            10)

        self.driver_to_interfacer_tmr = self.create_timer(
            self.driver_to_interfacer_tmr_period, 
            self.driver_to_interfacer_publisher_callback)
 
    # SP updating the command variables through the interfacer
    def {{ resource_name }}_interfacer_to_driver_callback(self, data):
        {% for item in command_variables %}
        self.{{ item }} = data.{{ item }}
        self.msg_emulator_to_interfacer.got_{{ item }} = data.{{ item }}
        {%- endfor %}

    def driver_to_interfacer_publisher_callback(self):
        
        # add stuff here and there...

        self.{{ resource_name }}_driver_to_interfacer_publisher_.publish(self.msg_emulator_to_interfacer)

def main(args=None):
    rclpy.init(args=args)

    {{ resource_name }}_simulator = {{ capitalized_resource_name }}Simulator()

    rclpy.spin({{ resource_name }}_simulator)

    {{ resource_name }}_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()