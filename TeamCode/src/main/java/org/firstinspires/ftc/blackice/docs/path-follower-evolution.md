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

In auto, it combined a single wheel encoder to estimate linear displacement with an IMU for heading lock. In tele-op, it used the internal IMU to transform a target, field-relative vector into a robot-relative vector.

Once the robot reached the target position, it would stop by setting the power to zero on [zero power brake mode](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html#BRAKE). We were limited to about 50% power since any faster would make the robot overshoot the target and could make the encoder wheel slip. Movement was basic but multi-directional with our mecanum wheels. Lacked speed, accuracy, and could not follow paths.

### Field-Centric TeleOp

After we realized we could retrieve the robot's heading from the IMU, it sparked the idea of reversing the heading to transform controller input into field-centric movement. This simple concept became a foundational step in developing vector following, which is essential for path following.

**Fun Fact:** At the time we had no idea that it was even called field-centric teleOp. We used to call it "controller relative TeleOp" because it was relative to the controller's orientation if the controller was relative to the field.

## v2.0 - Odometry Wheels + ZeroPowerBrakeMode Stop Prediction
###### Intermediate version between our first and second competitions in 2024.

[Why did we develop our own Path Follower?]()

With acquiring independent dead wheels, we discovered that there was no need to slowly accelerate and decelerate because they would not slip like powered wheels so we did not want to use other libaries.

Used goBILDA pinpoint odometry to move toward a target and then relied on zero power brake mode to stop.

Using separate non-powered wheels to track distance allowed for much more speed because we could apply braking force opposite to the robot's motion and didn't have to worry about if the powered wheels slightly slipped.

So far, this prototype could only go from point to point; it could not follow paths with lines or curves.

### Preventing Overshoot

This would obviously overshoot the target position instantly because it would only brake after reaching the target position. To fix this, we decided to create a program that drives the robot at different speeds, turns on zero power brake mode, and calculates the distance the robot took to brake. With those data points, we found that it was in the form of `ax^2 + bx` so we used quadratic regression, in that form, to derive a function that accurately predicts the required braking distance at any speed.

![img.png](img.png)
*Velocity to braking distance data points*

We commonly found our coefficients to be around `a=0.001` and `b=0.07`.

- The `a` term accounts for friction and momentum when braking. This makes it much more accurate at high velocities when friction is more dominant than the wheels' braking force.
- the `b` term is basically the robot's braking force from the wheels. Later on, we would realize that just this term in our future predictive braking controller was just a more empirical version of the Derivative term in PIDs.

In this prototype version, it would just turn on zero power brake mode if the distance was greater than the distance remaining to the target point. This worked okay, but it could not correct while it was braking. In next prototype version we would fix this issue by turning it into a simple proportional controller.

## v3.0 - Corrective Braking Using a Quadratic-Damped PID
###### Final version used in the 2024–2025 season, first called Black-Ice at our second competition, and also used at our state championship.

In the previous prototype version, it would just turn on zero power brake mode if the braking distance was greater than the distance remaining to the target point. This worked okay, but it could not correct while it was braking. In this prototype version we fixed this issue by turning it into a simple proportional controller that predicts the robot's position by how much displacement it would take to brake.

Pseudo code implementation of the predictive braking controller:
```java
// Note: all of these are vector quantities with (x, y)
predictedBrakingDisplacement = a*velocity*abs(velocity) + b*velocity
predictedPositionAfterBraking = current + predictedBrakingDisplacement
error = target - predictedPositionAfterBraking
power = error * proportionalConstant
```
We later would realize that this is just a more empirical version of a [PID controller with quadratic-damping](). The `b` is just the Derivative term and the `a` is the quadratic damping.

**Fun Fact:** We originally thought our algorithm was falling apart here because it would be quite a few inches off from the target, so we tried adding Integral terms but eventually we figured out that one of our odometry pods was just defective.

### Continuing Momentum At End
Another benefit was that the robot didn’t always need to stop completely. By checking whether the predicted braking distance was greater than or equal to the distance remaining, we could safely advance to the next target before overshooting. This way, the robot transitions to the next waypoint exactly when needed, avoiding overshoot and preventing the controller from braking unnecessarily.

### Separate Forward and Lateral Axis for Mecanum Wheels
In later versions, we experimented with creating separate braking predictors for the lateral and forward axes of the mecanum wheels. However, testing showed that the added complexity and tuning variables weren’t worth it. It was essentially like having separate PID controllers for each axis, which is unnecessary. The controller’s inherent predictiveness and corrective behavior already provided accurate and adaptable control.
We did, however, add an extra lateral effort multiplier, which was worthwhile since strafing requires more power than moving forward or backward.

Limitations: could only go from point to point.

# The Pursuit of Following Curves

## v4.0 - Dynamic Lookahead Follower
###### Beginning of 2025 off-season.

Made follower using a lookahead based off the predicted braking displacement. Followed based of a bunch of points spread along the path.

Worked well, however, points were limited to about 1-2 inches apart, or else the robot's reaction time would be slower than the robot's loop time. Looking back at this now, we realized we could have updated our follower and skipped several points in the same loop instead of waiting for the next loop for each point. However, it lacked accuracy and the option of motion profiles for smooth deceleration.

## v5.0 - Sophisticated Follower
###### 2025 off-season and beyond.

This is currently the latest version.
It is a more sophisticated follower that can follow more than just points, including continuous paths such as Bézier curves and lines. It uses centripetal, translational, heading, and drive vectors (prioritized in that order) for smooth and controlled motion. Our drive vector can also follow custom velocity profiles and slower deceleration through PIDFs with feedforward corrected with momentum compensation.

### PredictiveBrakingController = Empirical PID controller with Quadratic Damping
It wasn't until this point that we realized our predictive braking controller was essentially an empirical form of a Proportional-Derivative (PD) controller with added quadratic damping. 
```
outputPower = error - a*velocity*abs(velocity) - b*velocity
```

The term `b * velocity` acts as a derivative component, since the target is constant and the rate of change of position is `-velocity`.

Expanded form:
```
error = target - current
derivative = -kD * velocity
quadraticDamping = -kQ * velocity * abs(velocity)
outputPower = error + derivative + quadraticDamping
```
The combined damping terms `derivative + quadraticDamping` represent the predicted overshoot due to momentum and the robot's braking constraints, and are subtracted from the error to improve control. In our code we call `kD`: `kBraking` and `kQ`: `kFriction`
