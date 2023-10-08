from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    actuator_board_node = Node(
        package="hw_support_pkg",
        executable="actuator_board_node",
        parameters=[
            {"board_bus_id"         : 6         },
            {"service_length"       : 5         },
            {"watchdog_timeout"     : 2000      }
        ],
	    arguments=['--ros-args', '--log-level', 'info']
    )

    motor_board_node = Node(
        package="hw_support_pkg",
        executable="motor_board_node",
        parameters=[
            {"board_bus_id"         : 10        },
            {"service_length"       : 5         },
            {"watchdog_timeout"     : 2000      }
        ],
	    arguments=['--ros-args', '--log-level', 'info']
    )

    distribution_board_node = Node(
        package="hw_support_pkg",
        executable="distribution_board_node",
        parameters=[
            {"board_bus_id"         : 1                             },
            {"service_length"       : 5                             },
            {"watchdog_timeout"     : 2000                          },
            {"cells_scales"         : [0.0,0.0,0.0]                 },
            {"voltages_scales"      : [0.0095, 0.00945, 0.0095, 0]  },
            {"current_scales"       : [11.0, 11.0, 11.0, 11.0]      },
            {"cells_offsets"        : [0.0,0.0,0.0]                 },
            {"voltages_offsets"     : [0.0,0.0,0.0,0.0]             },
            {"current_offsets"      : [0.0,0.0,0.0,0.0]             },
            {"encoder_ppr"          : 4096                          }
        ],
	    arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(actuator_board_node)
    ld.add_action(motor_board_node)
    ld.add_action(distribution_board_node)

    return ld
