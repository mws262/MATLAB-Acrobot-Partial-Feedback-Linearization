# MATLAB implementation of Partial Feedback Linearization for the acrobot
This is based on Mark Spong's paper: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=407375&tag=1
It implements two swing up controllers for a double pendulum with only a motor at the second joint (the "elbow"), also called the acrobot.
I am not associated with the paper. I've just implemented it in order to play with it.

I don't recommend this as a method to control a physical acrobot. The results are very interesting to watch, but with torque-bounded actuators, you're not going to be able to get it to work.