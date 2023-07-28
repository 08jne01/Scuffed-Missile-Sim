import numpy as np


class Body:
    def __init__(self, mass : float, position : np.ndarray, velocity : np.ndarray):
        self.position = np.array(position)
        self.velocity = np.array(velocity)
        self.mass = mass
        self.path = []

        self.append_path(0.0)

    def append_path(self, t):
        self.path.append(np.array((t, self.position[0], self.position[1], self.position[2])))

    def integrate(self, dt, force : np.ndarray, time : float):
        force += np.array((0.0, 0.0, -9.81 * self.mass))

        self.velocity += dt * force / self.mass
        self.position += dt * self.velocity

        if time != None:
            self.append_path(time+dt)


    def get_path(self):
        path = np.array(self.path)
        return np.transpose(path)

    def get_direction(self):
        speed = np.sqrt(self.velocity.dot(self.velocity))
        return self.velocity / speed