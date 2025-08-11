# Our Key Innovation, Quadratic-Damped PID
to make use of back-EMF braking

Our key innovation about our follower is our quadratic damping PID. We include this quadratic damping term in our translational PID, making it significantly more aggressive, accurate, while minimizing premature deceleration for faster deceleration. 
You can learn about how we came up with this idea here. or our first empirical approach

Pseudo Code of PD controller with quadratic-damping
```
error = target - current
derivative = -kD * velocity
quadraticDamping = -kQ * velocity * abs(velocity)
proportional = error * proportionalConstant
outputPower = proportional + derivative + quadraticDamping
```

## How does -velocity = Derivative term in PID?
Let the positional error be:  
$ \text{error} = \text{target}_{\text{pos}} - \text{current}_{\text{pos}} $

Taking the derivative of both sides:  
$$ \frac{d}{dt}(\text{error}) = \frac{d}{dt}(\text{target}_{\text{pos}}) - \frac{d}{dt}(\text{current}_{\text{pos}}) $$

If the target position is constant, then:  
$$ \frac{d}{dt}(\text{target}_{\text{pos}}) = 0 $$

Then:  
$$ \frac{d}{dt}(\text{error}) = -\frac{d}{dt}(\text{current}_{\text{pos}}) = -\text{velocity} $$

Therefore, when the target position is fixed, the derivative of the position error is simply the **negative of the current velocity**.



PredictiveBrakingController = Empirical PID controller with Quadratic Damping
It was at this point that we realized our predictive braking controller was essentially an empirical form of a Proportional-Derivative (PD) controller with added quadratic damping.

outputPower = error - a*velocity*abs(velocity) - b*velocity
The term b * velocity acts as a derivative component, since the target is constant and the rate of change of position is -velocity.

Expanded form:

error = target - current
dTerm = -kD * velocity
quadraticDamping = -kQ * velocity * |velocity|
outputPower = error + dTerm + quadraticDamping
The combined damping terms dTerm + quadraticDamping represent the predicted overshoot due to momentum and the robot's braking constraints, and are subtracted from the error to improve control.


Why does the quadratic-damping work?
- whenever you reverse the power of the wheels, it reverses the internal back-EMF, which causes the motor to brake fast using it's own momentum even if the power is very small such as -0.00001. This means that the faster the wheel was moving in the first place, the more braking power it has. This is exactly what zero power brake mode uses to brake. In a perfect world this would be force proportional to velocity, however when the robot's powered wheels are braking and stopped, the robot does not decelerate perfectly linearly. The friction of braking actually makes the ratio of velocity to braking distance non-linear. At first we were just trying to predict zero power brake mode, and that required us to use quadratic regression to get a non-linear prediction with a quadratic-damping term. If you had a PID that was less fast and aggressive and it coasted to its target, then you wouldn't need the quadratic-damping. It is only for high-speed replication of the robot's real braking model.


We would have never knew to add a quadratic damping term if it were not for the previous iterations. Using Zero Power Braking Mode to set that goal of stopping as fast and as accurate as possible turned into great.

We have been partnering with Pedro Path however to implement some aspects of Black Ice into Pedro Path or even just add a separate fork of pedro path with our follower but will all of the access to the localization and tuning.

Later added motion profiles with target velocities for the drive vector to smoothly slow down before the predictive braking controller

todo back-emf, lower power braking, etc Predictive EMF Braking

Modeled braking displacement with kP/kQuad terms and actively applied reverse power based on back-EMF to both slow and correct position error.

// apply constant negative power reverse to the wheel does not give accurate linear deceleration this is due the internal emf braking.

//Made separate braking displacement models for forward vs lateral movement with mecanum wheels. but after testing it works to just use the same model for both axis since it is the equivalent of halfing different PIDs for each axis which is a bit unnecessary and just requires more tuning. There is support for this tho.

Things to note: power reverse of back-EMF Modeled braking distance with kP/kQuad terms and actively applied reverse power based on back-EMF to both slow and correct position error.

// FIXME Things to note after testing: our model basically simulated the zero power brake mode but with correction. the zero power brake mode uses back-EMF to make the wheels stop spinning and a fact is that if you just reverse the direction of the wheels for example form +1 to -0.001 it will basically behave like zero power brake mode until it gets to lower velocities where -0.001 doesn't do much and where a power like -0.3 would slow down more at lower velocities.

//Predictive back-EMF Braking






Why does the Quadratic-Damping work?
triggers internal back EMF and more accurately models how the robot brakes because it cannot supply a constant force when friction dominates

title with PID controller with QuadraticDamping, and then explain more empirically with predicted braking displacement


We would have never knew to add a quadratic damping term if it were not for the previous iterations. Using Zero Power Braking Mode to set that goal of stopping as fast and as accurate as possible turned into great.
Those vectors may sound similar to pedro path but the previous version of our follower were necessary for us to discover and develop the unique thing about our follower. The deceleration and aggressively smooth positional control with the help of a quadratic damping term.

## Why did we added a quadratic damping term?
What other libraries do:

Other libraries use coasting (Zero Power Float Mode) to decelerate. This causes predictable near-constant deceleration, typically around `–40 in/s²`. However, it is extremely slower than zero power brake mode which has upwards of `–250 in/s²` but it is not linear and doesn't have constant deceleration.

What Black Ice does:

To understand robot deceleration in FTC, you need to recognize two types:

- Coasting (Zero Power Float Mode) causes near-constant deceleration, typically around `–40 in/s²`.
- 
- Active Braking (e.g. applying `–0.3` power after moving forward) creates nonlinear deceleration — steep at high speeds and weaker as the robot slows. In theory, a constant braking force would yield constant deceleration, but in practice, friction and motor physics dominate. 

### Back-EMF
other libraies make use of lower the drive power to near 0 to coast to a stop and then rely on a slow but smooth translational to hold the end pose.

When motor power reverses direction, the motor uses its own momentum to brake via back EMF. This means that the braking force is force proportional to velocity.
Although this braking force is linear, friction dominates at high speeds, especially when the wheels slip slightly. This introduces nonlinearity and motivates the use of quadratic damping in control to better model and correct for the robot's real braking behavior.

internal back-EMF braking
   uses regenerative braking which gives wheel power proportional to velocity making braking distance proportional to velocity instead of velocity squared.
 Even though the wheels are applying a proportional force, friction dominates at high speeds which creates the need for quadratic damping for if the wheels slightly slip.
- 
This is largely due to internal back-EMF braking, which uses regenerative braking to generate a braking force proportional to wheel velocity. This makes braking distance scale with velocity, rather than velocity squared.
Although this braking force is linear, friction dominates at high speeds, especially when the wheels slip slightly. This introduces nonlinearity and motivates the use of quadratic damping in control to better model and correct for the robot's real braking behavior.

- and the same mechanism used in ZeroPowerBrake mode but it gives us more control and the ability to correct while braking. Even a tiny braking value like –0.0001 still stops the robot similarly from high speeds; the difference lies in how long braking takes at low speeds.

First thing you must understand is that there are two types of decelerations for FTC robots,
- **Coasting at zero power float mode** generally gives constant deceleration of around -40 inches/s/s
- However, applying a constant braking power to the wheels such as -0.3 after going forward, the deceleration becomes non-linear (or whatever you call not constant and not linear like it is a combination of linear and constant). Even a small value such as -0.0001 will still make the robot stop the same at high velocities, the only difference between the braking force is that it becomes weaker as the wheel's motion slows down.

Now if you could actually apply a constant braking force then the deceleration would be constant, however this is not possible in real physics since friction is dominant.
Whenever the wheel power reverses direction, the motor internals use something called back-EMF braking. This is this same thing as what [zero power brake mode](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html#BRAKE) uses. 

Deceleration at 60 inches/s goes from -40 coasting to -250 with back EMF braking

Two with applying reverse power to brake the motors.
On mecanum wheels with 312 rpm, coasting has about a deceleration of -40 inches/s/s, whereas zero power brake mode has a deceleration of about -150-250+ inches/s/s but

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
