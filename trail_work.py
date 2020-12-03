import sys
if "." not in sys.path:
    sys.path.append(".")

import source_world
import moth_brain_nengo
import moth_brain_nengo_new
import nengo
import pytry
import numpy as np

import matplotlib.pyplot as plt
import scipy.ndimage.filters
class SimulationFail(Exception):
    pass
class SimulationSuccess(Exception):
    pass




class trail(pytry.PlotTrial):
    def params(self):
        self.param("x",x = 0)
        self.param("y",y = -1)
        self.param("moveable_source",moveable_source = False)
        self.param("heading",heading = 0)
        self.param("minx",min_x = -2)
        self.param("miny",max_x = 2)
        self.param("miny",min_y = -2)
        self.param("maxy",max_y = 2)
        self.param("sourcex",source_x = 0)


        self.param("sourcey",source_y = 1)
        self.param("distance_array",distance_array = [])
        self.param("stop_distance",stop_distance = 0.01)
        self.param("maxtime",max_time = 100)


    def evaluate(self, p, plt):
        model = nengo.Network()
        with model:

            world = source_world.nengoSource(x=p.x, y=p.y, heading=p.heading+np.pi)
            if p.moveable_source:
                source=world.make_source_move()

            sensor=world.make_sensor()
            rotation=world.make_movement()
            pos = world.make_position()
            heading_and_vel = world.make_heading_and_velocity()


            brain = moth_brain_nengo_new.MothBrainNengo(noise=0, inhib=0, N=100)
            nengo.Connection(sensor[0], brain.inputR, transform=1, synapse=None)
            nengo.Connection(sensor[1], brain.inputL, transform=1, synapse=None)
            nengo.Connection(brain.turn, rotation, transform=10, synapse=None)

            def check_bounds(t):

                if world.x < p.min_x or world.x > p.max_x:
                    raise SimulationFail()
                if world.y < p.min_y or world.y > p.max_y:
                    raise SimulationFail()
                delta = [world.x - p.source_x, world.y - p.source_y]
                distance=np.linalg.norm(delta)
                p.distance_array.append(distance)
                if distance < p.stop_distance:
                    raise SimulationSuccess()

            nengo.Node(check_bounds)

            p_pos = nengo.Probe(pos)
            p_heading = nengo.Probe(heading_and_vel[0])
            p_velocityx = nengo.Probe(heading_and_vel[1])
            p_velocityy = nengo.Probe(heading_and_vel[2])
            p_rotation = nengo.Probe(rotation,synapse=0.3)
            p_sensor = nengo.Probe(sensor)
            if p.moveable_source:
                p_source=nengo.Probe(source)

            input_left_probe = nengo.Probe(brain.inputL)
            input_right_probe = nengo.Probe(brain.inputR)

            PBN_left_probe = nengo.Probe(brain.PBN_l_slow, synapse=0.03)  # ,synapse=0.03
            PBN_right_probe = nengo.Probe(brain.PBN_r_slow, synapse=0.03)  # ,synapse=0.03

            FF_left_probe = nengo.Probe(brain.stateL, synapse=0.03)  # ,synapse=0.03)
            FF_right_probe = nengo.Probe(brain.stateR, synapse=0.03)  # ,synapse=0.03)

            GII_left_probe = nengo.Probe(brain.giia_l, synapse=0.03)
            GII_right_probe = nengo.Probe(brain.giia_r, synapse=0.03)

            turn_probe = nengo.Probe(brain.turn, synapse=0.03)  # )

        sim = nengo.Simulator(model, progress_bar=False)
        try:
            sim.run(p.max_time, progress_bar=True)
            success = False
        except SimulationSuccess:
            success = True
        except SimulationFail:

                success = False

        return dict(path = sim.data[p_pos] ,
                    marker= (p.source_x,p.source_y),
                    start_pos=(p.x,p.y))




