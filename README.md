# HITL

A repository for the Hardware In The Loop (HITL) project. 

## What is Hardware in the Loop?
In engineering, hardware-in-the-loop (HITL) testing is a pivotal process in the development of products, particularly during the prototyping phase. This approach involves the simplification and modelling of core system components, laying the groundwork for the subsequent development of various systems, including control systems. Engaging with this methodology as part of a university course presented both a challenge and an opportunity to delve into the practical application of control systems.

## What was the objective?

The task at hand was the development of a control schema for a model bi-rotor helicopter, employing a state-space control strategy, with a specific focus on Linear Quadratic Regulator (LQR) techniques. The project’s objective was to engineer a controller capable of stabilizing the helicopter, guiding it through a series of predetermined manoeuvres: taking off, executing a 180° rotation, achieving stabilization, rotating back 180°, and landing.

## What are the core ideas of state-space control?

The crux of creating a controller that could reliably perform these tasks lay in the tuning of the Q and R matrices. The Q matrix is intimately tied to the cost function of the system’s states, emphasizing the importance of each state variable within the overall cost function. Adjusting the Q matrix allows for the prioritization of certain state variables over others, directing the controller’s focus towards stabilizing those specific variables. This is crucial in ensuring that the helicopter maintains desired flight characteristics and responds appropriately to control inputs.

Conversely, the R matrix is associated with the cost related to the control inputs themselves. Tuning the R matrix involves a delicate balance, as it dictates the controller’s aggressiveness in responding to deviations from the desired state. A more conservative approach, signified by higher values in the R matrix, results in less aggressive control actions, reducing the risk of overcorrection but potentially leading to slower response times.

## What were the difficulties of the project?

The iterative process of adjusting the Q and R matrices was not only a technical challenge but also an engaging exercise in applying theoretical knowledge to a tangible, real-world problem. Through this project, I gained invaluable insights into the intricacies of control system design and the critical role of parameter tuning in achieving system stability and performance. The experience of bringing a model helicopter to a controlled flight, guided by principles learned in the classroom, was immensely rewarding, underscoring the power of HITL testing in bridging the gap between theory and practice in engineering.




