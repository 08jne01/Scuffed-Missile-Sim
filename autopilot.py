import numpy as np

G_LIMIT = 40
ACCEL_LIMIT = G_LIMIT * 9.81

class PID:
    def __init__(self, p : float, i : float, d : float, i_lim : float):
        self.p = p
        self.i = i
        self.d = d
        self.i_lim = i_lim

        self.integral_part = np.zeros(3)
        self.previous_error = np.zeros(3)

    def update(self, target, current, dt):
        error = target - current

        p = self.p * error
        
        self.integral_part += self.i * error * dt
        self.integral_part = np.clip(self.integral_part, -self.i_lim, self.i_lim)

        d = -self.d * (error - self.previous_error) / dt
        self.previous_error = error
        return p + self.integral_part + d


class Proportional:

    def __init__(self, pn_gain, p=1.0, i=0.0, d=0.0, i_lim=1.0):

        self.pn_gain = pn_gain
        self.relative_pos = None
        self.miss_distance = 1.0e9
        self.miss_time = 0.0
        self.time = 0.0
        self.miss_point_missile = np.array((0.0,0.0,0.0))
        self.miss_point_target = np.array((0.0,0.0,0.0))
        self.loft = False
        self.pid = PID(p, i, d, i_lim)

    def set_loft(self, loft):
        self.loft = loft

    def missed(self, miss_time):
        return self.miss_time >= miss_time

    def config(self):
        s = "ProNav: PN = {}, PID=({},{},{}) I Lim={}".format(self.pn_gain, self.pid.p, self.pid.i, self.pid.d, self.pid.i_lim)
        return s

    def get_command(self, target_pos : np.ndarray, own_pos : np.ndarray, own_vel : np.ndarray, dt : float):

        relative_pos = target_pos - own_pos
        miss_distance = np.sqrt(relative_pos.dot(relative_pos))

        if isinstance(self.relative_pos, type(None)):
            self.relative_pos = relative_pos
            return np.array([0.0,0.0,0.0])

        missile_direction = own_vel / np.sqrt(own_vel.dot(own_vel))
        radial_direction = relative_pos / np.sqrt(relative_pos.dot(relative_pos))

        if miss_distance <= self.miss_distance:
            self.miss_distance = miss_distance
            self.miss_point_missile = own_pos.copy()
            self.miss_point_target = target_pos.copy()
            self.miss_time = 0.0
        else:
            self.miss_time += dt


        #if missile_direction.dot(radial_direction) < 0.0:
         #   self.miss_time += dt
        #else:
         #   self.miss_time = 0.0

        relative_target_vel = (relative_pos - self.relative_pos) / dt
        self.relative_pos = relative_pos

        closure_rate = np.sqrt(relative_target_vel.dot(relative_target_vel))
        line_of_sight = np.cross(relative_pos, relative_target_vel) / np.dot(relative_pos, relative_pos)

        own_speed = np.sqrt(own_vel.dot(own_vel))



        g_bias_factor = 1.0

        if self.loft:
            if self.time < 2.0:
                g_bias_factor = 4.0
            elif miss_distance < 10000.0:
                g_bias_factor = 1.0
            else:
                g_bias_factor = 2.0

        g_bias = np.array((0.0, 0.0, 9.81 * g_bias_factor))
        acceleration = -self.pn_gain * closure_rate * np.cross(own_vel / own_speed, line_of_sight)


        #acceleration = self.pid.update(np.zeros(3), acceleration, dt)
        #acceleration[1] = self.parallel_y.update(0.0, acceleration[1], dt)
        #acceleration[2] = self.parallel_z.update(0.0, acceleration[2], dt)
        acceleration += g_bias


        acceleration_mag = np.sqrt(acceleration.dot(acceleration))

        self.time += dt

        if acceleration_mag <= 0.0:
            return np.array([0.0,0.0,0.0])

        actual_acceleration_mag = np.clip(acceleration_mag, 0.0, ACCEL_LIMIT)
        return acceleration * actual_acceleration_mag / acceleration_mag


class ZeroEffortMiss:

    def __init__(self, pn_gain, p=1.0, i=0.0, d=0.0, i_lim=1.0):

        self.pn_gain = pn_gain
        self.relative_pos = None
        self.miss_distance = 1.0e9
        self.miss_time = 0.0
        self.time = 0.0
        self.miss_point_missile = np.array((0.0,0.0,0.0))
        self.miss_point_target = np.array((0.0,0.0,0.0))
        self.loft = False
        self.pid = PID(p, i, d, i_lim)

    def set_loft(self, loft):
        self.loft = loft

    def missed(self, miss_time):
        return self.miss_time >= miss_time

    def config(self):
        s = "ZEM: PN = {}, PID=({},{},{}) I Lim={}".format(self.pn_gain, self.pid.p, self.pid.i, self.pid.d, self.pid.i_lim)
        return s

    def get_command(self, target_pos : np.ndarray, own_pos : np.ndarray, own_vel : np.ndarray, dt : float):

        relative_pos = target_pos - own_pos
        miss_distance = np.sqrt(relative_pos.dot(relative_pos))

        if isinstance(self.relative_pos, type(None)):
            self.relative_pos = relative_pos
            return np.array([0.0,0.0,0.0])

        missile_direction = own_vel / np.sqrt(own_vel.dot(own_vel))
        radial_direction = relative_pos / np.sqrt(relative_pos.dot(relative_pos))

        if miss_distance <= self.miss_distance:
            self.miss_distance = miss_distance
            self.miss_point_missile = own_pos.copy()
            self.miss_point_target = target_pos.copy()
            self.miss_time = 0.0
        else:
            self.miss_time += dt


        #if missile_direction.dot(radial_direction) < 0.0:
         #   self.miss_time += dt
        #else:
         #   self.miss_time = 0.0

        relative_target_vel = (relative_pos - self.relative_pos) / dt
        closure_rate = np.sqrt(relative_target_vel.dot(relative_target_vel))
        self.relative_pos = relative_pos


        tgo = miss_distance / closure_rate

        zem = relative_pos + relative_target_vel * tgo
        zem_r = zem.dot(radial_direction) * radial_direction
        zem_n = zem - zem_r
        acceleration = self.pn_gain * zem_n / (tgo * tgo)


       # proj_a = acceleration.dot(missile_direction) * missile_direction
        #acceleration = acceleration - proj_a

        g_bias_factor = 1.0

        if self.loft:
            if self.time < 2.0:
                g_bias_factor = 4.0
            elif miss_distance < 10000.0:
                g_bias_factor = 1.0
            else:
                g_bias_factor = 2.0

        g_bias = np.array((0.0, 0.0, 9.81 * g_bias_factor))
        
        #acceleration = -self.pid.update(np.zeros(3), acceleration, dt)
        #acceleration[1] = self.parallel_y.update(0.0, acceleration[1], dt)
        #acceleration[2] = self.parallel_z.update(0.0, acceleration[2], dt)

        

        acceleration += g_bias
        
        #proj_a = acceleration.dot(missile_direction)
        #acceleration -= proj_a
        
        acceleration_mag = np.sqrt(acceleration.dot(acceleration))

        self.time += dt

        if acceleration_mag <= 0.0:
            return np.array([0.0,0.0,0.0])

        actual_acceleration_mag = np.clip(acceleration_mag, 0.0, ACCEL_LIMIT)
        return acceleration * actual_acceleration_mag / acceleration_mag


class Parallel:

    def __init__(self, pn_gain, p=1.0, i=0.0, d=0.0, i_lim=1.0):

        self.pn_gain = pn_gain
        self.relative_pos = None
        self.miss_distance = 1.0e9
        self.miss_time = 0.0
        self.time = 0.0
        self.miss_point_missile = np.array((0.0,0.0,0.0))
        self.miss_point_target = np.array((0.0,0.0,0.0))
        self.loft = False
        self.pid = PID(p, i, d, i_lim)

    def set_loft(self, loft):
        self.loft = loft

    def missed(self, miss_time):
        return self.miss_time >= miss_time

    def config(self):
        s = "Parallel: PN = {}, PID=({},{},{}) I Lim={}".format(self.pn_gain, self.pid.p, self.pid.i, self.pid.d, self.pid.i_lim)
        return s

    def get_command(self, target_pos : np.ndarray, own_pos : np.ndarray, own_vel : np.ndarray, dt : float):

        relative_pos = target_pos - own_pos
        miss_distance = np.sqrt(relative_pos.dot(relative_pos))

        if isinstance(self.relative_pos, type(None)):
            self.relative_pos = relative_pos
            return np.array([0.0,0.0,0.0])

        missile_direction = own_vel / np.sqrt(own_vel.dot(own_vel))
        radial_direction = relative_pos / np.sqrt(relative_pos.dot(relative_pos))

        if miss_distance <= self.miss_distance:
            self.miss_distance = miss_distance
            self.miss_point_missile = own_pos.copy()
            self.miss_point_target = target_pos.copy()
            self.miss_time = 0.0
        else:
            self.miss_time += dt


        #if missile_direction.dot(radial_direction) < 0.0:
         #   self.miss_time += dt
        #else:
         #   self.miss_time = 0.0

        relative_target_vel = (relative_pos - self.relative_pos) / dt
        closure_rate = np.sqrt(relative_target_vel.dot(relative_target_vel))
        self.relative_pos = relative_pos


        tgo = miss_distance / closure_rate

        zem = relative_pos + relative_target_vel * tgo
        zem_dir = zem / np.sqrt(zem.dot(zem))
        n = np.cross(missile_direction, zem_dir)

        acceleration = self.pn_gain * np.cross(n, own_vel) / tgo

        


        #zem_r = zem.dot(radial_direction) * radial_direction
        #zem_n = zem - zem_r
        #acceleration = self.pn_gain * zem_n / (tgo * tgo)

        #proj_a = acceleration.dot(missile_direction) * missile_direction
        #acceleration = acceleration - proj_a

        g_bias_factor = 1.0

        if self.loft:
            if self.time < 2.0:
                g_bias_factor = 4.0
            elif miss_distance < 10000.0:
                g_bias_factor = 1.0
            else:
                g_bias_factor = 2.0

        g_bias = np.array((0.0, 0.0, 9.81 * g_bias_factor))
        #acceleration = -self.pid.update(np.zeros(3), acceleration, dt)
        #acceleration[1] = self.parallel_y.update(0.0, acceleration[1], dt)
        #acceleration[2] = self.parallel_z.update(0.0, acceleration[2], dt)

    
        acceleration += g_bias
        
        #proj_a = acceleration.dot(missile_direction)
        #acceleration -= proj_a
        
        acceleration_mag = np.sqrt(acceleration.dot(acceleration))

        self.time += dt

        if acceleration_mag <= 0.0:
            return np.array([0.0,0.0,0.0])

        actual_acceleration_mag = np.clip(acceleration_mag, 0.0, ACCEL_LIMIT)
        return acceleration * actual_acceleration_mag / acceleration_mag