from robot import Robot, Link

arm = Robot([
    Link([0, 0, 0]),
    Link([1, 0, 0]),
    Link([2, 0, 0]),
    Link([3, 0, 0])
])

angles = [0, 0, 1.5707, 0, 0, 0, 0, 0, 0, 0, 0, 0]
result = arm.forward_kinematics(angles)[:3, -1]
print(result)

target = [1, 2, 1]
solution = arm.inverse_kinematics(target)
print(solution)

arm.plot(solution, target)
