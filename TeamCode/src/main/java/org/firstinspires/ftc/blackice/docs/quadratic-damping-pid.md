# Our Key Innovation, Quadratic-Damped PID

Our key innovation about our follower is our quadratic-damped PID. We include this quadratic damping term in our translational PID, making it nearly **5x faster** at decelerating, more aggressive, and accurate while minimizing premature deceleration for faster deceleration. 
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

$error = target_{pos} - current_{pos}$

Taking the derivative of both sides:  

$\frac{d}{dt}(error) = \frac{d}{dt}(target_{pos}) - \frac{d}{dt}(current_{pos})$

If the target position is constant, then:  

$\frac{d}{dt}(\text{target}_{\text{pos}}) = 0$

Then:  

$\frac{d}{dt}(error) = -\frac{d}{dt}(current_{pos}) = -velocity$

Therefore, when the target position is fixed, the derivative of the position error is simply the **negative of the current velocity**.

## Why Quadratic Damping Is Needed?
We wanted to brake as fast as zero power brake mode, so that required quadratic regression to model the realistic, non-linear braking behavior of the robot with a quadratic-damped term. We later realized that applying small amounts of reverse power to the wheels uses the same system as zero power brake mode, but allows us to correct and control the robot's position. If you use a less aggressive PID controller that simply coasts to the target, quadratic damping isn’t necessary. It’s only required when you want high speed and braking that requires replicating the robot’s real braking dynamics at high speeds.

### Why is the Braking Non-Linear?
When you reverse the motor power, the motor’s internal back-EMF also reverses. This causes the motor to brake rapidly using its own momentum, even with very small power inputs like -0.00001. Essentially, the faster the wheel was initially spinning, the stronger the braking force generated. This is exactly how the zero power brake mode brakes.

In an ideal scenario, the braking force would be directly proportional to velocity. However, when the robot’s powered wheels are braking to a stop, the deceleration is not perfectly linear. The friction involved in braking causes the relationship between velocity and braking distance to become **non-linear**.

## How much difference does the Quadratic Damping make?
Other libraries, such as Pedro Path and Roadrunner, rely on less aggressive, slower PID controllers to coast to the target position. This is a more predictable constant deceleration of around `-40in/s` or `45in` of distance to stop at `60 in/s`. However, if you use a more aggressive PID controller with quadratic-damping that makes use of back-EMF, you can get much faster deceleration of around `10 in` of braking at `60in/s`. This is nearly **5 times faster deceleration**, plus the extra time you have to accelerate instead of braking.

Pros of Black Ice
- faster
- more accurate (due to higher proportional constant)
- more aggressive

Cons
- uses more power
- less smooth

## Tuning
All you have to do is run one tuning test that runs the robot at different velocities and then sets the motor's to zero power braking mode trigger back-EMF braking. It calculates all of the velocity to braking distances and does quadratic regression in the form `ax^2+bx` to give you your two constants. `a` is the quadratic damping, `b` is the braking force or classic Derivative constant. This way of tuning ensures the optimal...
(Already accounts for loop time delay in the d term)


We have been partnering with Pedro Path to implement some aspects of Black Ice into Pedro Path or even just add a separate fork of pedro path with our follower but will all of the access to the localization and tuning.





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
 [zero power brake mode](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html#BRAKE) uses. 


