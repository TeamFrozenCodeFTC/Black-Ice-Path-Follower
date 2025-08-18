## Why Did We Develop Our Own Path Follower?

1. We wanted to create a **faster** path follower that could brake as fast as `ZeroPowerBrakeMode` when needed and did not have to always coast to a stop.
   - This was because our goal for the 2024–2025 season was a five-specimen autonomous using only 312 RPM wheels and where after scoring each specimen, the robot had to turn around. 
   - To achieve this, we needed our autonomous to be as fast as possible; something we couldn’t accomplish without having our deceleration as fast as `ZeroPowerBrakeMode`.
2. We were **interested** in path following, and we wanted to intuitively **learn**, hands-on, how path following works.
3. We wanted more **customization**, modularization, and less boilerplate when building paths and routines.

It was worth creating our own path follower because we came up with adding a quadratic-damping term to our translational PID, which allows it to be nearly 5x faster at decelerating.

We would have never knew to add a quadratic damping term if it were not for creating our own path follower and the mini versions of it and previous iterations. Using Zero Power Braking Mode to set that goal of stopping as fast and as accurate as possible turned into great.

We have been partnering with Pedro Path to implement some aspects of Black Ice into Pedro Path or even just add a separate fork of pedro path with our follower but will all of the access to the localization and tuning.

Our contributions to Pedro Path for v1.1.0 that world teams use
- BezierCurve.through
- Kinematics class
- Wrote the Docs for pedro's deceleration algorithm
- HeadingInterpolator class
- Beta Tester
- Tested their new Bezier Curve class that uses Matrices
- Lead for deceleration in later versions
