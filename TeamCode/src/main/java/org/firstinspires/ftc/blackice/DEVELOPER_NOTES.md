Note: all this code is written by Jacob Ophoven

Definitions:
Parametric = a function that describes a path using a single variable (t)

Designed for modularity, readability, expandability, minimize redundancy, easily chaining methods, and performance.



Only why comments. no what comments. that is a sign of bad and unreadable code.

@apiNote - API note for common users.
@implNote - Implementation note for developers.


Follow style guidelines: https://www.oracle.com/java/technologies/javase/codeconventions-fileorganization.html



Heading 0 is the front side of the robot facing away from the starting wall
positive X-axis is forward for the robot with 0 heading (facing the center of the field)
positive Y-axis is strafing the robot to the left from 0 heading

This is the axis with the robot's HEADING AT 0 DEGREES:

Left is positive Y
Forward is 

^ +Y Axis (Robot Left)
│      
│ ┌──────────┐
│ │          │
│ │          │--> FRONT OF ROBOT (FORWARD)
│ │          │
│ └──────────┘
└──────────────> +X Axis (Robot Forward)
