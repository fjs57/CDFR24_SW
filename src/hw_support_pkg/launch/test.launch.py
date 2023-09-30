from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    can_support_node = Node(
        package="hw_support_pkg",
        executable="can_support_node",
        parameters=[
            {"interface_file"       : "can0"    },
            {"baudrate"             : 800000    },
            {"filter_addr"          : 0x0000    },
            {"filter_mask"          : 0x0000    },
            {"timeout_seconds"      : 0         },
            {"timeout_microseconds" : 500000    },
            {"log_raw_frames"       : False     }
        ],
	    arguments=['--ros-args', '--log-level', 'info']
    )

    generic_board_node = Node(
        package="hw_support_pkg",
        executable="generic_board_node",
        parameters=[
            {"board_bus_id"         : 5         },
            {"service_length"       : 6         },
            {"watchdog_timeout"     : 2000      }
        ],
	    arguments=['--ros-args', '--log-level', 'debug']
    )

    ld.add_action(can_support_node)
    ld.add_action(generic_board_node)

    return ld
