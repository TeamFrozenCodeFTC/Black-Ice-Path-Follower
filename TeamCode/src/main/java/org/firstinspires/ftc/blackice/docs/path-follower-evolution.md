<h1>
  The History and Evolution<br>
  <sub>of Our Path Follower, Black Ice</sub>
</h1>

### Prototype Versions

- [v1.0 – Wheel Encoder + IMU](#v10---wheel-encoder--imu)
- [v2.0 - Odometry Wheels + ZeroPowerBrakeMode Stop Prediction](#v20---odometry-wheels--zeropowerbrakemode-stop-prediction)
- [v3.0 - Corrective Braking Using a Quadratic-Damped PID](#v30---corrective-braking-using-a-quadratic-damped-pid)
- [v4.0 – Dynamic Lookahead Follower](#v40---dynamic-lookahead-follower)
- [v5.0 – Sophisticated Path Follower](#v50---sophisticated-follower)
  - translational, drive, heading, and deceleration profiles

The beginnings of Black Ice started in our first season, INTO THE DEEP 2024-2025, with the development of our first autonomous frameworks. Our small framework was later was fully developed into a sophisticated follower in the 2025 off-season.

## v1.0 - Wheel Encoder + IMU
###### Used in our first _ever_ competition in December, 2024.</span>

In auto, it combined a single wheel encoder to estimate linear displacement with an IMU for heading lock. In tele-op, it used the internal IMU to transform a target field-relative vector into a robot-relative vector.

Once the robot reached the target position, it would stop by setting the power to zero on [zero power brake mode](). We were limited to about 50% power since any faster would make the robot overshoot the target and could make the encoder wheel slip. Movement was basic but multi-directional with our mecanum wheels. Lacked speed and accuracy and could not follow paths.

### Field-Centric TeleOp

After we realized we could retrieve the robot's heading from the IMU, it sparked the idea of reversing the heading to transform controller input into field-centric movement. This simple concept became a foundational step in developing vector following, which is essential for path following.

**Fun Fact:** At the time we had no idea that it was even called field-centric teleOp. We used to call it "controller relative TeleOp" because it was relative to the controller's orientation if the controller was relative to the field.

## v2.0 - Odometry Wheels + ZeroPowerBrakeMode Stop Prediction
###### Intermediate version between our first and second competitions in 2024.

[Why did we develop our own Path Follower?]()

Used goBILDA pinpoint odometry to move toward a target and then relied on zero power brake mode to stop.

Using separate non-powered wheels to track distance allowed for much more speed because we could apply braking force opposite to the robot's motion and didn't have to worry about if the powered wheels slightly slipped.

So far, this prototype could only go from point to point; it could not path follow with lines or curves.

### Preventing Overshoot

This obviously would instantly overshoot the target position because it would only brake after it reached the target position. To fix this, we decided to create a program that drives the robot at different speeds, turns on zero power brake mode, and calculates the distance the robot took to brake. With those data points we found that it was in the form of `ax^2 + bx` so we used quadratic regression, in that form, to derive a function that accurately predicts the required braking distance at any speed.

We commonly found our coefficients to be around `a=0.001` and `b=0.07`.

- The `a` term accounts for friction and momentum when braking. This makes it much more accurate at high velocities when friction is more dominant than the wheels' braking force.
- the `b` term is basically the robot's braking force from the wheels. Later on, we would realize that just this term in our future predictive braking controller was just a more empirical version of the Derivative term in PIDs.

In this prototype version, it would just turn on zero power brake mode if the distance was greater than the distance remaining to the target point. This worked okay, but it could not correct while it was braking. In next prototype version we would fix this issue by turning it into a simple proportional controller.

## v3.0 - Corrective Braking Using a Quadratic-Damped PID
###### Last version used in the 2024-2025 season.

Used in our second competition in January 2025, and was the first time we called it Black-Ice.

In the previous prototype version, it would just turn on zero power brake mode if the braking distance was greater than the distance remaining to the target point. This worked okay, but it could not correct while it was braking. In this prototype version we fixed this issue by turning it into a simple proportional controller that predicts the robot's position by how much displacement it would take to brake.

Pseudo code implementation of the predictive braking controller:
```java
// Note: all of these are vector quantities with (x, y)
predictedBrakingDisplacement = a*velocity*abs(velocity) + b*velocity
predictedPositionAfterBraking = current + predictedBrakingDisplacement
error = target - predictedPositionAfterBraking
power = error * proportionalConstant
```
We later would realize that this is just a more empirical version of a PID controller with Quadratic-Damping. The `b` is just the Derivative term and the `a` is the quadratic damping.

**Fun Fact:** We originally thought our algorithm was falling apart here because it would be quite a few inches off from the target, so we tried adding Integral terms but eventually we figured out that one of our odometry pods was just defective.

TODO With this we didn't always have to stop at a specific point. We could check if the predicted braking displacement is greater than or equal to the distance remaining, and then we could just proceed to the next waypoint. This ensures it goes to the next waypoint as soon as it needs to so there is no overshoot and before the controller starts to brake.

With later modifications to this version, we also created two distinct braking  predictors for lateral and forward, but after testing, the results were not worth the extra tuning variables to manage. The predicitiveness as well as correctiveness of the controller makes it very accurate and smooth with no overshoot.

## v4.0 - Dynamic Lookahead Follower
###### Beginning of 2025 off-season.

Made follower using a lookahead based off the predicted braking displacement. Followed based of a bunch of points spread along the path.

Worked well, however, points were limited to about 1-2 inches apart or else the robot's reaction time would be slower than the robot's loop time. Looking back at this now, we realized we could have updated our follower and skipped several points in the same loop instead of waiting for the next loop for each point. However, it lacked accuracy and option of motion profiles for smooth deceleration.

## v5.0 - Sophisticated Follower
###### 2025 off-season.

This is currently the latest version.
This is a more sophisticated follower that can follow more than just points, including continuous paths such as Bézier curves and lines. It uses centripetal, translational, heading, and drive vectors (prioritized in that order) for smooth and controlled motion. Our drive vector can also follow custom velocity profiles through PIDFs with momentum compensation.


### PredictiveBrakingController = Empirical PID controller with Quadratic Damping
It was at this point that we realized our predictive braking controller was essentially an empirical form of a Proportional-Derivative (PD) controller with added quadratic damping.
```
outputPower = error - a*velocity*abs(velocity) + b*velocity
```

The term `b * velocity` acts as a derivative component, since the target is constant and the rate of change of position is `-velocity`.

Expanded form:
```
error = target - current
dTerm = -kD * velocity
quadraticDamping = -kQ * velocity * |velocity|
outputPower = error + dTerm + quadraticDamping
```
The combined damping terms `dTerm + quadraticDamping` represent the predicted overshoot due to momentum and the robot's braking constraints, and are subtracted from the error to improve control.

## So what if the difference from Pedro Path?
1. We include a quadratic damping term in our translational PID, making it significantly more aggressive, accurate, while minimizing premature deceleration for faster deceleration. Learn about why this is here.
2. Velocity Motion Profiles
3. Easier Path Creation
   - Do not have to specify the previous starting point for going from one point to another
   - .from(Pose), fromPreviousTarget() .fromCurrentPose() 
   - .currentPoseTo() allows you to create dynamic paths that go towards at certain point from the current pose. Very useful for teleOp macros.
3. Path Continuity not required

We would have never knew to add a quadratic damping term if it were not for the previous iterations. Using Zero Power Braking Mode to set that goal of stopping as fast and as accurate as possible turned into great.
Those vectors may sound similar to pedro path but the previous version of our follower were necessary for us to discover and develop the unique thing about our follower. The deceleration and aggressively smooth positional control with the help of a quadratic damping term.

Why we added a quadratic damping term?
When the robot applys negative power/braking power it isn't linear deceleration , etc

This is similar to Pedro Path but our translational doesn't just use a positional PIDs but it uses our predictive braking controller which allows us to tune our proportional much more aggressively, making our follower more accurate and much faster.
We also made constructing paths a lot less repetitive than pedro path.
We have been partnering with Pedro Path however to implement some aspects of Black Ice into Pedro Path or even just add a separate fork of pedro path with our follower but will all of the access to the localization and tuning.

Later added motion profiles with target velocities for the drive vector to smoothly slow down before the predictive braking controller

todo back-emf, lower power braking, etc Predictive EMF Braking

Modeled braking displacement with kP/kQuad terms and actively applied reverse power based on back-EMF to both slow and correct position error.

Later on we would realize that just this term in our future predictive positional controller was just a more empirical version of the Derivative term. But the `a` quadratic term allows for a much more accurate and aggressive positional controller because it is more closely related to how the robot actually brakes. A simple PID positional controller can work if you tune the deceleration to be the robot's naturally deceleration but with more aggressive PID controllers it began to too harshly correct at lower velocities and overshoot at higher velocities.

// apply constant negative power reverse to the wheel does not give accurate linear deceleration this is due the internal emf braking.

//Made separate braking displacement models for forward vs lateral movement with mecanum wheels. but after testing it works to just use the same model for both axis since it is the equivalent of halfing different PIDs for each axis which is a bit unnecessary and just requires more tuning. There is support for this tho.

Things to note: power reverse of back-EMF
Modeled braking distance with kP/kQuad terms and actively applied reverse power based on back-EMF to both slow and correct position error.

// FIXME
Things to note after testing: our model basically simulated the zero power brake mode but with correction. the zero power brake mode uses back-EMF to make the wheels stop spinning and a fact is that if you just reverse the direction of the wheels for example form +1 to -0.001 it will basically behave like zero power brake mode until it gets to lower velocities where -0.001 doesn't do much and where a power like -0.3 would slow down more at lower velocities.

//Predictive back-EMF Braking
