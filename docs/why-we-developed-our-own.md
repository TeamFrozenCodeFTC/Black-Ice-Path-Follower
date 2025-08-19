---
nav_order: 2
---

# Why Did We Develop Our Own Path Follower?

1. We wanted to create a **faster** path follower that could brake as fast as `ZeroPowerBrakeMode` when needed and did not have to always coast to a stop.
   - This was because our goal for the 2024–2025 season was a five-specimen autonomous using only 312 RPM wheels and where after scoring each specimen, the robot had to turn around. 
   - To achieve this, we needed our autonomous to be as fast as possible; something we couldn’t accomplish without having our deceleration as fast as `ZeroPowerBrakeMode`.
2. We were **interested** in path following, and we wanted to intuitively **learn**, hands-on, how path following works.
3. We wanted more **customization**, modularization, and less boilerplate when building paths and routines.

It was worth creating our own path follower because it led us to add a quadratic-damping term to our translational PID. This made the system nearly five times faster at decelerating. We never would have discovered this approach without building our own follower with all the eariler prototype and iterations. Setting the goal of stopping as quickly and accurately as possible, using Zero Power Brake Mode as inspiration, pushed us to that breakthrough.
