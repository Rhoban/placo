import sys
import commentjson as json
import numpy as np
import pinocchio as pin
import argparse
import matplotlib.pyplot as plt
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz
from pinocchio.visualize import MeshcatVisualizer
import time
from os.path import dirname, join, abspath

# geom.removeAllCollisionPairs()
# fp = open('collisions.json', 'r')
# collisions = json.load(fp)
# fp.close()
# for entry in collisions:
#     geom.addCollisionPair(pin.CollisionPair(entry[0], entry[1]))

parser = argparse.ArgumentParser(
    description="Compute the collisions exclusion matrix"
)
parser.add_argument(
    "-v",
    action='store_true',
    help="Visualization"
)
parser.add_argument(
    "-n",
    help="Number of runs",
    type=int,
    default=None
)
args = parser.parse_args()
visualisation = args.v

this_dir = join(dirname(str(abspath(__file__))))
mesh_dir = join(this_dir, 'sigmaban')
urdf_file = join(mesh_dir, 'robot.urdf')

print('- Loading model...')
model, col, vis = pin.buildModelsFromUrdf(
    urdf_file, mesh_dir, pin.JointModelFreeFlyer())
print(model)

print('- Building geometry...')
geom = pin.buildGeomFromUrdf(
    model, urdf_file, pin.GeometryType.COLLISION, mesh_dir)

print('- Adding collision pairs...')
geom.addAllCollisionPairs()
total_pairs = len(geom.collisionPairs)
print("Found %d collisions pairs" % total_pairs)

print('Creating data...')
model_data = model.createData()
geom_data = pin.GeometryData(geom)

q0 = pin.neutral(model)
q = q0.copy()

if visualisation:
    print('Creating viewer...')
    viz = MeshcatVisualizer(model, col, vis)
    viz.initViewer()
    viz.loadViewerModel()

# Number of DOF (7 is the floating base)
n_dof = len(q)-7
avg = 0
collisions = set()
last_update = time.time()
start = time.time()
xs = []
ys = []

print('Starting the collision loops...')

n_tests = 0
while True:
    # Setting the position to a random one
    q[7:] = model.lowerPositionLimit[7:] + \
        (model.upperPositionLimit[7:] - model.lowerPositionLimit[7:]) * \
        np.random.uniform(0, 1, n_dof)

    if visualisation:
        viz.display(q)

    pin.computeCollisions(model, model_data, geom, geom_data, q, False)

    for k in range(len(geom.collisionPairs)):
        cr = geom_data.collisionResults[k]

        if cr.isCollision():
            cp = geom.collisionPairs[k]
            pair = (cp.first, cp.second)
            if pair not in collisions:
                collisions.add(pair)

    n_tests += 1
    if args.n is not None and n_tests > args.n:
        break

    if time.time() - last_update > 5:
        last_update = time.time()
        print('- Tested %d configurations, found %d effective collision pairs (over %d, which is %.2f%%), updating collisions.json...' %
             (n_tests, len(collisions), total_pairs, (100*len(collisions)/total_pairs)))

        # Updating the collisions matrix file
        fp = open('sigmaban/collisions.json', 'w')
        json.dump(list(collisions), fp)
        fp.close()

        xs.append(time.time() - start)
        ys.append(len(collisions))
        plt.clf()
        plt.grid()
        plt.plot(xs, ys)
        plt.xlabel('Time (s)')
        plt.ylabel('Collisions detected')
        plt.pause(0.01)


