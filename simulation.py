import physics_engine
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import autopilot
from matplotlib.widgets import Slider

colors = ['black','red','orange','green','blue','cyan']

def distance(r1,r2):

    r = r1 - r2
    return np.sqrt(r.dot(r)) 


class Simulation:

    def __init__(self, missile_autopilots, target_func, missile_thrust_func, velocities):

        self.target_func = target_func
        self.missile_thrust_func = missile_thrust_func
        self.missile_pos = np.array((0.0,0.0,2000.0))
        self.missile_vel = np.array((50.0,0.0,10.0))
        self.missile_mass = 250.0

        self.missile_autopilots = missile_autopilots
        self.missiles = []

        for i,autopilot in enumerate(self.missile_autopilots):

            missile_vel = self.missile_vel
            if velocities != None:
                missile_vel = velocities[i]

            self.missiles.append(physics_engine.Body(self.missile_mass, self.missile_pos, missile_vel))

        self.t = 0.0
        self.dt = 0.06
        self.n = 0
        self.min_t = 0.0
        self.max_t = 300.0

    def run(self):

        self.t = 0.0
        self.n = 0

        done = False

        while not done:
            target_position = self.target_func(self.t)

            done = True

            for autopilot,missile in zip(self.missile_autopilots, self.missiles):

                if autopilot.missed(10.0):
                    continue

                done = False

                missile_command = autopilot.get_command(target_position, missile.position, missile.velocity, self.dt)
                missile_thrust = missile.get_direction() * self.missile_thrust_func(self.t)
                missile.integrate(self.dt, self.missile_mass * missile_command + missile_thrust, self.t)

            self.t += self.dt
            self.n += 1

            if self.t > 300.0:
                break


    def get_paths(self):

        N = self.n + 1
        t = np.linspace(0.0, self.t, N)
        target_path = np.empty((N,3))

        for i,cur_t in enumerate(t):
            target_path[i] = self.target_func(cur_t)

        target_path = np.transpose(target_path)

        missile_paths = []

        for missile_autopilot,missile in zip(self.missile_autopilots, self.missiles):
            missile_path = missile.get_path()
            missile_path_obj = {
                "path" : missile_path[1:],
                "t_end" : missile_path[0][-1],
                "n" : len(missile_path[0]),
                "missile" : True,
                "miss_missile" : missile_autopilot.miss_point_missile,
                "miss_target" : missile_autopilot.miss_point_target,
                "miss_distance" : missile_autopilot.miss_distance,
                "config" : missile_autopilot.config()
            }

            missile_paths.append(missile_path_obj)

        target_path_obj = {
            "path" : target_path,
            "t_end" : self.t,
            "n" : self.n+1,
            "missile" : False,
            "config" : "Target"
        }

        return missile_paths, target_path_obj

def spiral_target_position(t : float):

    vel = np.array((-200.0,0.0,0.0))

    r = 2000.0
    gforce = 9.0
    acceleration = gforce * 9.81

    angular_velocity = np.sqrt(acceleration / r)

    dy = np.sin(t * angular_velocity) * r
    dz = np.cos(t * angular_velocity) * r

    pos = np.array((60000.0, dy, 1000.0 + dz))

    return pos + vel * t

def straight_target_angle(t : float):
    speed = 300.0
    pos = np.array((60000.0, 0.0, 4000.0))
    vel = np.array((-1.0, 1.0, 0.0)) * speed

    return pos + vel * t

def s_target_position(t : float):

    vel = np.array((-300.0, 0.0, 0.0))

    period = 60.0
    angular_freq = 2.0 * np.pi / period

    displacement = 4000.0 * np.sin(angular_freq * t)
    pos = np.array((1e5, displacement, 1e4))
    return pos + vel * t



def break_target_position(t : float):
    speed = 500.0
    gforce = 1.1
    acceleration = gforce * 9.81

    radius = speed * speed / acceleration
    angular_vel = acceleration / speed

    defending_time = 80.0
    cold_time = np.pi / angular_vel

    original_pos = np.array((1e5, 0.0, 1e4))
    defending_pos = original_pos + np.array((-speed,0.0,0.0)) * min(t,defending_time)
    if t <= defending_time:
        return defending_pos
    
    t_param = min(t - defending_time, cold_time)
    x = np.cos(-angular_vel * t_param + 3.0 * np.pi/2.0) * radius
    y = np.sin(-angular_vel * t_param + 3.0 * np.pi/2.0) * radius + radius
    turning_pos = defending_pos + np.array((x,y,0.0))

    if t <= (cold_time + defending_time):
        return turning_pos

    t_param = t - cold_time - defending_time
    defending_pos = turning_pos + np.array((speed, 0.0, 0.0)) * t_param

    return defending_pos


def missile_thrust(t : float):

    if t <= 2.9:
        return 35000.0
    else:
        return 0.0
    
def loft_vel(loft_angle, speed):
    loft_angle_rad = np.radians(loft_angle)
    x = speed * np.cos(loft_angle_rad)
    z = speed * np.sin(loft_angle_rad)
    return np.array([x,0.0,z])


def main():

    pro_nav = autopilot.Proportional(10.0)
    pro_nav.set_loft(False)
    pro_nav2 = autopilot.Proportional(3.0)
    pro_nav2.set_loft(True)
    zem = autopilot.ZeroEffortMiss(3.0, 1.0, 0.5, 0.0, 0.5)
    parallel = autopilot.Parallel(3.0, 1.0, 0.5, 0.0, 0.5)

    autopilots = [parallel,zem]
    velocities = []

    for i in range(3,7):
        autopilots.append(autopilot.Proportional(i))

    for i in autopilots:
        velocities.append(np.array([200.0, 0.0, 0.0]))

    #autopilots = []
    """
    loft_angles = [10.0, 30.0, 60.0]
    for angle in loft_angles:
        velocities.append(loft_vel(angle, 300.0))
        ap = autopilot.Proportional(3.0)
        ap.set_loft(True)
        autopilots.append(ap)

    """

    #velocities = [loft_vel(10.0, 50.0), loft_vel(10.0, 50.0), loft_vel(10.0, 50.0), loft_vel(10.0, 50.0)]
    #autopilots = [pro_nav, pro_nav2, zem, parallel]


    simulation = Simulation(autopilots, spiral_target_position, missile_thrust, velocities)
    simulation.run()
    missile_paths, target_path =  simulation.get_paths()
    fig = plt.figure()
    fig.tight_layout()
    ax = plt.axes(projection='3d')

    paths = []
    paths.append(target_path)
    paths += missile_paths

    lines = []

    for i,path in enumerate(paths):
        color = colors[i%len(colors)]
        x = path["path"][0]
        y = path["path"][1]
        z = path["path"][2]
        line = ax.plot(x,y,z, color, label=path["config"])[0]

        if "miss_missile" in path:
            p1 = path["miss_missile"] 
            p2 = path["miss_target"]

            data = np.transpose(np.array((p1,p2)))
            ax.scatter3D(data[0],data[1],data[2], c=color)
            ax.plot3D(data[0],data[1],data[2], color)

            print("[{}][{}] miss distance = {}".format(color, path["config"], path["miss_distance"]))


        lines.append(line)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend()


    
    #ax.set_xlim(0, xlim)
   # ax.set_ylim(-ylim/2.0, ylim/2.0)
    #ax.set_zlim(0.0, zlim)

    


    
    ax_time = fig.add_axes([0.25, 0.05, 0.5, 0.01])
    time_slider = Slider(
        ax=ax_time,
        label='Time Max (s)',
        valmin=0.0,
        valmax=simulation.t,
        valinit=simulation.t,
    )

    ax_time2 = fig.add_axes([0.25, 0.01, 0.5, 0.01])
    time_slider2 = Slider(
        ax=ax_time2,
        label='Time Min (s)',
        valmin=0.0,
        valmax=simulation.t,
        valinit=0.0,
    )

    simulation.min_t = 0.0
    simulation.max_t = simulation.t

    def update_lines():

        x0 = []
        x1 = []
        y0 = []
        y1 = []
        z0 = []
        z1 = []

        for line,path in zip(lines,paths):


            v_max = max(simulation.max_t, simulation.min_t)
            v_min = min(simulation.max_t, simulation.min_t)
            

            step = float(path["n"]) * v_max / path["t_end"]
            step_n = int(step)

            step_min = float(path["n"]) * v_min / path["t_end"]
            step_min_n = int(step_min)

            if step_min_n == step_n:
                step_n += 1

            r = path["path"]
            line.set_data(r[0][step_min_n:step_n], r[1][step_min_n:step_n])
            line.set_3d_properties(r[2][step_min_n:step_n])

            if path["missile"] and path["path"][0][step_min_n:step_n] != []:
                x0.append(min(path["path"][0][step_min_n:step_n]))
                x1.append(max(path["path"][0][step_min_n:step_n]))
                y0.append(min(path["path"][1][step_min_n:step_n]))
                y1.append(max(path["path"][1][step_min_n:step_n]))
                z0.append(min(path["path"][2][step_min_n:step_n]))
                z1.append(max(path["path"][2][step_min_n:step_n]))


        if x0 != []:
            xmin = min(x0)
            xmax = max(x1)
            ymin = min(y0)
            ymax = max(y1)
            zmin = min(z0)
            zmax = max(z1)

            ax.set_xlim(xmin,xmax)
            ax.set_ylim(ymin,ymax)
            ax.set_zlim(zmin,zmax)
            ax.set_box_aspect(((xmax - xmin), (ymax - ymin), (zmax - zmin)))
        #ax.update({})

        ax.autoscale_view(True)
        ax.autoscale(False)
        fig.canvas.draw_idle()
        return lines


    def update_slider_min(value):
        simulation.min_t = value
        update_lines()

    def update_slider_max(value):
        simulation.max_t = value
        update_lines()


    time_slider2.on_changed(update_slider_min)
    time_slider.on_changed(update_slider_max)

    update_lines()
    plt.show()


   # ani = animation.FuncAnimation(fig, update_lines, n, fargs=(paths, lines), interval=100)

    

if __name__ == "__main__":
    main()