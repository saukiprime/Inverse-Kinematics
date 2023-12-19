import numpy as np
import matplotlib.pyplot
import scipy.optimize

# from optimizer import levenberg_marquardt

class Link():

    def __init__(self, translation: np.ndarray):
        self.translation = np.array(translation)

    def link_matrix(self, rpy):
        def translation_matrix(x, y, z):
            return np.array([
                [1, 0, 0, x], 
                [0, 1, 0, y], 
                [0, 0, 1, z], 
                [0, 0, 0, 1]
            ])

        def rpy_matrix(roll, pitch, yaw):   
            return np.array([
                    [np.cos(yaw), -np.sin(yaw), 0, 0],
                    [np.sin(yaw), np.cos(yaw), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ]) @ np.array([
                    [np.cos(pitch), 0, np.sin(pitch), 0],
                    [0, 1, 0, 0],
                    [-np.sin(pitch), 0, np.cos(pitch), 0],
                    [0, 0, 0, 1]
                ]) @ np.array([
                    [1, 0, 0, 0],
                    [0, np.cos(roll), -np.sin(roll), 0],
                    [0, np.sin(roll), np.cos(roll), 0],
                    [0, 0, 0, 1]
                ])

        result = np.eye(4)
        result = result @ translation_matrix(*self.translation)
        result = result @ rpy_matrix(*rpy)

        return result

class Robot:
    
    def __init__(self, links):
        self.links = links

    def forward_kinematics(self, joints, all_nodes=False):
        current_matrix = np.eye(4)

        if all_nodes:
            current_matrixes = []

        joints = np.array(joints).reshape(-1, 3)
        for (link, joint_parameters) in zip(self.links, joints):
            current_matrix = current_matrix @ link.link_matrix(joint_parameters)
            if all_nodes:
                current_matrixes.append(current_matrix)

        if all_nodes:
            return current_matrixes
        else:
            return current_matrix

    def inverse_kinematics(self, target_position):
        initial_position = np.array([0, 0, 0] * len(self.links))

        def optimize_function(x):
            fk = self.forward_kinematics(x)
            target_error = (fk[:3, -1] - target_position)
            return target_error

        res = scipy.optimize.least_squares(optimize_function, initial_position).x
        # res = levenberg_marquardt(optimize_function, initial_position)[0]

        return res.reshape(-1, 3)

    def plot(self, joints, end=None):
        nodes = []

        transformation_matrixes = self.forward_kinematics(joints, all_nodes=True)

        for index in range(len(self.links)):
            node = transformation_matrixes[index][:, -1]
            nodes.append(node)

        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
        max_limit = max(max([x[0] for x in nodes]), max([x[1] for x in nodes]), max([x[2] for x in nodes]))
        min_limit = min(min([x[0] for x in nodes]), min([x[1] for x in nodes]), min([x[2] for x in nodes]))
        interval = (max_limit - min_limit) / 8
        ax.set_xlim([min_limit - interval, max_limit + interval])
        ax.set_ylim([min_limit - interval, max_limit + interval])
        ax.set_zlim([min_limit - interval, max_limit + interval])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.plot([x[0] for x in nodes], [x[1] for x in nodes], [x[2] for x in nodes], linewidth=2)
        ax.scatter([x[0] for x in nodes], [x[1] for x in nodes], [x[2] for x in nodes], s=35)
        if end is not None:
            ax.scatter(end[0], end[1], end[2], c="red", s=50)

        matplotlib.pyplot.show()
