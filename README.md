# Inverse Kinematics

Solve inverse kinematics problems with optimization-based approaches.

## Installation

Install the dependencies using pip:

```bash
pip install -r requirements.txt
```

## Usage
Create a `Robot` object and then call its `forward_kinematics` or `inverse_kinematics` methods.

```python
arm = Robot([
    Link([0, 0, 0]),
    Link([1, 0, 0]),
    Link([1, 0, 0]),
    Link([1, 0, 0])
])

angles = [0, 0, 1.5707, 0, 0, 0, 0, 0, 0, 0, 0, 0]
result = arm.forward_kinematics()[:3, -1]

target = [1, 2, 0]
solution = arm.inverse_kinematics(target)

arm.plot(angles, result[:3, -1])
```
Here, `arm` is initialized with a list of each link in the robot, `angles` is an input list containing the roll, pitch and yaw angles of each joint, and `target` is the target position for the end effector. `result` contains the output position of the end effector in the forward kinematics problem. `solution` contains the settings for each joint to get to the target position. Calling `Robot.plot` with the joint setttings and target location will plot them in a window.

To run the program, navigate to the project directory and run the `main.py` file:

```bash
python main.py
```
