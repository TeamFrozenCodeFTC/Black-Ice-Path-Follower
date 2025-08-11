## Why Did We Develop Our Own Path Follower?

1. We wanted to create a **faster** path follower that could brake as fast as `ZeroPowerBrakeMode` when needed and did not have to always coast to a stop.
   - This was because our goal for the 2024–2025 season was a five-specimen autonomous using 312 RPM wheels where after scoring each specimen, the robot had to turn around. 
   - To achieve this, we needed our autonomous to be as fast as possible; something we couldn’t accomplish without having our deceleration as fast as `ZeroPowerBrakeMode`.
2. We were **interested** in path following, and we wanted to intuitively **learn**, hands-on, how path following works.
3. We wanted more **customization**, modularization, and less boilerplate when building paths and routines.
