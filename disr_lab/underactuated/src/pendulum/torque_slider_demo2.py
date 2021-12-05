import argparse
import numpy as np

from pydrake.all import (DiagramBuilder, Simulator)
from pydrake.examples.pendulum import PendulumPlant
from underactuated import SliderSystem
from visualizer import PendulumVisualizer

from underactuated.meshcat_rigid_body_visualizer import (
    MeshcatRigidBodyVisualizer)


from pydrake.all import (
    DiagramBuilder,
    RgbdCamera,
    RigidBodyFrame,
    RigidBodyPlant,
    RigidBodyTree,
    RungeKutta2Integrator,
    Shape,
    SignalLogger,
    Simulator,
    BasicVector,
    LeafSystem,
    PortDataType,
    AddModelInstanceFromUrdfFile,
    FloatingBaseType,
    AddFlatTerrainToWorld,
    VectorSystem,
    VectorSystem,
)

class Controller(VectorSystem):
    def __init__(self):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.value = 3
    def update(self, val):
        self.value = val
    def _DoCalcVectorOutput(self, context, unused, unused2, output):
        output[:] = self.value


builder = DiagramBuilder()
#pendulum = builder.AddSystem(PendulumPlant())



rbt = RigidBodyTree()
robot_base_frame = RigidBodyFrame(
        "robot_base_frame", rbt.world(),
        [0.0, 0, 0], [0, 0, 0])
AddModelInstanceFromUrdfFile("/home/pawel/proj/underactuated/src/pendulum/pendulum.urdf", FloatingBaseType.kFixed,
                                 robot_base_frame, rbt)

rbplant = RigidBodyPlant(rbt)
rbplant.set_name("Rigid Body Plant")

pendulum = builder.AddSystem(rbplant)
pbrv = MeshcatRigidBodyVisualizer(rbt, draw_timestep=0.01)
visualizer2 = builder.AddSystem(pbrv)
builder.Connect(pendulum.get_output_port(0), visualizer2.get_input_port(0))


parser = argparse.ArgumentParser()
parser.add_argument("-T", "--duration",
                    type=float,
                    help="Duration to run sim.",
                    default=10000.0)
args = parser.parse_args()

visualizer = builder.AddSystem(PendulumVisualizer())
builder.Connect(pendulum.get_output_port(0), visualizer.get_input_port(0))

ax = visualizer.fig.add_axes([.2, .95, .6, .025])
torque_system = builder.AddSystem(SliderSystem(ax, 'Torque', -5., 5.))


input_system = Controller();
visualizer = builder.AddSystem(input_system)

builder.Connect(input_system.get_output_port(0),
                pendulum.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
state.SetFromVector([1., 0.])

simulator.StepTo(args.duration)
