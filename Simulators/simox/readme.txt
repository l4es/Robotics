Simox
-----

The aim of the lightweight platform independent C++ toolbox Simox is to provide a set of
algorithms for 3D simulation of robot systems, sampling based motion planning and grasp
planning. Simox consists of three libraries (Virtual Robot, Saba and Grasp Studio) and nu-
merous examples showing how these libraries can be used to build complex tools in the
context of mobile manipulation. The library Virtual Robot can be used to define complex
robot systems, which may cover multiple robots with many degrees of freedom. The robot
structure and its visualization can be easily defined via XML files and environments with
obstacles and objects to manipulate are supported. Further, basic robot simulation compo-
nents, as Jacobian computations and generic Inverse Kinematics (IK) solvers, are offered by
the library. Beyond that, extended features like tools for analyzing the reachable workspace
for robotic manipulators or contact determination for grasping are included.
With Saba, a library for planning collision-free motions is offered, which directly incorporates
with the data provided by Virtual Robot. The algorithms cover state-of-the-art implementa-
tions of sampling-based motion planning approaches (e.g. Rapidly-exploring Random Trees)
and interfaces that allow to conveniently implement own planners. Since Saba was designed
for planning in high-dimensional configuration spaces, complex planning problems for robots
with a high number of degrees of freedom (DoF) can be solved efficiently.
Grasp Studio offers possibilities to compute the grasp quality for generic end-effector 
definitions, e.g. a humanoid hand. The implemented 6D wrench-space computations can be used
to easily (and quickly) determine the quality of an applied grasp to an object. Furthermore,
the implemented planners are able to generate grasp maps for given objects automatically.
Since complex frameworks have to incorporate with several libraries in order to provide full
functionality, several issues may arise when setting up the environment, such as dependency
problems, incompatible library versions or even non-existing ports of needed libraries for the
used operating systems. Hence, only a limited set of libraries are used by the Simox core in
order to make it compile. Extended functionality (e.g. visualization) can be turned off in
order to allow Simox compiling on most platforms. Further dependencies are encapsulated
with interfaces, making it easy to exchange e.g. the collision engine or the visualization
functionality. As a reference implementation Simox offers Coin3D/SoQt-based visualization
support.

License
-------
GNU LGPL 2 (see license.txt)

Copyright
---------
 2013 Nikolaus Vahrenkamp

Contact
-------
Nikolaus Vahrenkamp
vahrenkamp at users dot sf dot net
http://wwwiaim.ira.uka.de/users/vahrenka
http://simox.sourceforge.net
