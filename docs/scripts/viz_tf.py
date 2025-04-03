import numpy as np
import pytransform3d.rotations as pr
import pytransform3d.visualizer as pv
import pytransform3d.transformations as pt

def plotPlanes(fig, poly_planes):
    for plane in poly_planes:

        norm_vec = plane[0]
        pt = plane[1]

        # norm_vec = np.array([plane[0], plane[1], plane[2]])

        # pt: point that lies on plane
        # pt = -plane[3] * norm_vec

        # Plot plane
        fig.plot_plane(normal = norm_vec,
                        point_in_plane = pt, 
                        s = 1.0,
                        c= (0, 1, 1))

        # Plot normal of plane
        fig.plot_vector(start=pt,
                        direction=norm_vec,
                        c=(1.0, 0.5, 0.0))

fig = pv.figure()
# Plot Origin
fig.plot_transform(np.eye(4))

#################
# Plot start and goal 
#################

start = np.array([8, 0, 1])
goal = np.array([3.6, 1, 1])

start_tf =  pt.transform_from(R=np.eye(3), p=start)
goal_tf =  pt.transform_from(R=np.eye(3), p=goal)

fig.plot_sphere(radius=0.05, A2B=start_tf, c=(1, 0, 0))
fig.plot_sphere(radius=0.05, A2B=goal_tf, c=(0, 1, 0))

#################
# Input planes
#################
poly1_planes = []
# poly1_planes.append((np.array([-0.144193,  -0.53487,   0.83254]),
#                     np.array([4.55, 0.75, 1.05]))) # 0
poly1_planes.append((np.array([-0.0384832, 0.00551909,  -0.999244]),
                    np.array([4.75, 0.55, 0.85]))) # 1
poly1_planes.append((np.array([-0.252432, -0.951793,  0.174262]),
                    np.array([4.85, 0.45, 0.85]))) # 2
poly1_planes.append((np.array([0.221621, 0.975133,        0]),
                    np.array([8.22162, 0.975133,        1]))) # 3
poly1_planes.append((np.array([-0.221621, -0.975133,        -0]),
                    np.array([7.77838, -0.975133,         1]))) # 4
poly1_planes.append((np.array([-0.975133,  0.221621,         0]),
                    np.array([2.62487, 1.22162,       1]))) # 5
poly1_planes.append((np.array([0.975133, -0.221621 ,       -0]),
                    np.array([8.97513, -0.221621,         1]))) # 6
poly1_planes.append((np.array([0,  0, -1]),
                    np.array([8, 0, 0]))) # 7
poly1_planes.append((np.array([-0, -0,  1]),
                    np.array([8, 0, 2]))) # 8

#################
# Plot planes
#################
plotPlanes(fig, poly1_planes)

if "__file__" in globals():
    fig.show()
else:
    fig.save_image("__open3d_rendered_image.jpg")


# Polyhedron 0:
# Plane 0: 
#   normal: -0.144193,  -0.53487,   0.83254
#   pt: 4.55, 0.75, 1.05
# Plane 1: 
#   normal: -0.0384832, 0.00551909,  -0.999244
#   pt: 4.75, 0.55, 0.85
# Plane 2: 
#   normal: -0.252432, -0.951793,  0.174262
#   pt: 4.85, 0.45, 0.85
# Plane 3: 
#   normal: 0.221621, 0.975133,        0
#   pt:  8.22162, 0.975133,        1
# Plane 4: 
#   normal: -0.221621, -0.975133,        -0
#   pt:   7.77838, -0.975133,         1
# Plane 5: 
#   normal: -0.975133,  0.221621,         0
#   pt: 2.62487, 1.22162,       1
# Plane 6: 
#   normal:  0.975133, -0.221621 ,       -0
#   pt:   8.97513, -0.221621,         1
# Plane 7: 
#   normal:  0,  0, -1
#   pt:      8, 0, 0
# Plane 8: 
#   normal: -0, -0,  1
#   pt:      8, 0, 2

# Polyhedron 1:
# Plane 0: 
#   normal:  0.395628   0.32969 -0.857195
#   pt: 3.65 0.75 0.35
# Plane 1: 
#   normal:  0.997509 0.0498755 0.0498755
#   pt: 4.35 0.55 1.05
# Plane 2: 
#   normal:   0.98773 0.0493865 -0.148159
#   pt: 4.35 0.55 0.85
# Plane 3: 
#   normal:  0.255812 -0.799412   -0.5436
#   pt:  3.75 -0.75  0.15
# Plane 4: 
#   normal: -0.894427  0.447214         0
#   pt: 2.70557 1.44721       1
# Plane 5: 
#   normal:  0.894427 -0.447214        -0
#   pt:  4.49443 0.552786        1
# Plane 6: 
#   normal: -0.447214 -0.894427         0
#   pt:   2.65279 -0.894427         1
# Plane 7: 
#   normal: 0.447214 0.894427       -0
#   pt: 4.04721 1.89443       1
# Plane 8: 
#   normal: -0  0 -1
#   pt: 3.6   1   0
# Plane 9: 
#   normal:  0 -0  1
#   pt: 3.6   1   2






# Polyhedron 0:
# Plane 0: -0.144193,  -0.53487,   0.83254,  0.183066
# Plane 1: -0.0384832, 0.00551909,  -0.999244,    1.02912
# Plane 2: -0.252432, -0.951793,  0.174262,   1.50448
# Plane 3: 0.221621, 0.975133,        0, -2.77297
# Plane 4: -0.221621, -0.975133,        -0,  0.772969
# Plane 5: -0.975133,  0.221621,         0,   2.28886
# Plane 6:  0.975133, -0.221621,        -0,  -8.80106
# Plane 7:  0,  0, -1, -0
# Plane 8: -0, -0,  1, -2

# Polyhedron 1:
# Plane 0:  0.395628   0.32969 -0.857195  -1.39129
# Plane 1:  0.997509 0.0498755 0.0498755  -4.41897
# Plane 2:   0.98773 0.0493865 -0.148159  -4.19785
# Plane 3:  0.255812 -0.799412   -0.5436  -1.47731
# Plane 4: -0.894427  0.447214         0   1.77272
# Plane 5:  0.894427 -0.447214        -0  -3.77272
# Plane 6: -0.447214 -0.894427         0  0.386362
# Plane 7: 0.447214 0.894427       -0  -3.5044
# Plane 8: -0  0 -1 -0
# Plane 9:  0 -0  1 -2

