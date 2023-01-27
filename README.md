# ServoMotor
An ESP32 C++ servo motor controller class inspired by... cornstarch? :)
This class is a PIV (as opposed to PID) servo controller of my own invention which uses negative velocity feedback (the V in PIV) instead of the traditional derivative (D) term as typically found in PID controllers.

The reference to cornstarch is due to how the velocity feedback works. 
As a child I mixed water and cornstarch and discovered an odd phenomenon; the more rapidly I stirred the concoction the more it acted like a solid. I would stop and it returned into a liquid. I later learned this is called a non-Newtonian liquid (some call this "oobleck").

So how does this relate to this project? I was having difficulty getting a satisfactory response from my PID controller. No combination of values seemed to work well. Then it dawned on me that instead of using a derivative term based on error history, I should use a term based on negative velocity - kind of how the cornstarch got more rigid the harder I pushed it. The idea is that the routine should apply proportional braking force based on velocity and ease up as it approaches the target and slows down.

I was blown away with how well this works and how much tolerance the routine has for varying loads and inertia.

