import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as collections


def plot2dMap(axis, dataset_class):

    # obs_size = dataset_class.obstacles[0].size
    obs_x = []
    obs_y = []
    rects = []
    for obs in dataset_class.obstacles:
        obs_x.append(obs.x)
        obs_y.append(obs.y)
        rects.append(
            plt.Rectangle(
                (obs.x - obs.size[0] / 2, obs.y - obs.size[1] / 2),
                obs.size[0],
                obs.size[1],
                color="b",
            )
        )

    pc = collections.PatchCollection(rects)
    axis.add_collection(pc)
    axis.axis(dataset_class.bounds)
    return pc


def plotArm(figure, axis, arm, conf, color, width):
    # PLOTARM Plot Arm class in 3D
    #
    #   Usage: PLOTARM(arm, conf, color, width)
    #   @arm    Arm object
    #   @conf   arm configuration vector
    #   @color  color string, use plot convention, e.g. 'r' is red
    #   @width  line width

    position = arm.forwardKinematicsPosition(conf)
    # marker='-'
    axis.plot(
        position[0, :], position[1, :], position[2, :], color=color, linewidth=width
    )

    axis.plot(
        position[0, :-1],
        position[1, :-1],
        position[2, :-1],
        "k.",
        markersize=10 * width,
    )


def plotEvidenceMap2D(figure, axis, prob_grid, origin_x, origin_y, cell_size):
    # PLOTEVIDENCEMAP2D Plot 2D evidence grid map, for 2D dataset visualization
    #
    #   Usage: PLOTEVIDENCEMAP2D(prob_grid, origin_x, origin_y, cell_size)
    #   @prob_grid              evidence grid matrix
    #   @origin_x, origin_y     origin (down-left) corner of the map
    #   @cell_size              cell size

    # map display setting
    # colormap([0.3 0.3 0.3; 0.7 0.7 0.7; 1 1 1]);

    # get X-Y coordinates
    grid_rows = prob_grid.shape[0]
    grid_cols = prob_grid.shape[1]
    grid_corner_x = origin_x + (grid_cols - 1) * cell_size
    grid_corner_y = origin_y + (grid_rows - 1) * cell_size

    grid_X = np.linspace(origin_x, grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin_y, grid_corner_y, num=grid_rows)

    temp = (1 - prob_grid) * 2 + 1
    z_min = np.amin(temp)
    z_max = np.amax(temp)

    c = axis.pcolor(grid_X, grid_Y, temp, vmin=z_min, vmax=z_max)
    # figure.colorbar(c, ax=axis) # add colorbar

    axis.invert_yaxis()  # TODO: check this again! same as set(gca,'YDir','normal')
    # axis equal
    axis.axis("equal")
    axis.axis(
        [
            origin_x - cell_size / 2,
            grid_corner_x + cell_size / 2,
            origin_y - cell_size / 2,
            grid_corner_y + cell_size / 2,
        ]
    )

    return c


def plotMap3D(figure, axis, corner_idx, origin, cell_size):
    # PLOTMAP3D Plot 3D evidence grid map, for visualization of 3D dataset
    #
    #   Usage: PLOTMAP3D(corner_idx, origin, cell_size)
    #   @corner_idx   corner index [x1, x2, y1, y2, z1, z2], generated by generate3Ddataset
    #   @origin       origin (down-left) corner of the map
    #   @cell_size    cell size

    # for each obj

    # colormap([0.4, 0.4, 0.4])

    for i in range(corner_idx.shape[0]):
        x1 = corner_idx[i, 0] * cell_size + origin[0]
        x2 = corner_idx[i, 1] * cell_size + origin[0]
        y1 = corner_idx[i, 2] * cell_size + origin[1]
        y2 = corner_idx[i, 3] * cell_size + origin[1]
        z1 = corner_idx[i, 4] * cell_size + origin[2]
        z2 = corner_idx[i, 5] * cell_size + origin[2]
        axis.plot(
            [x1, x2, x1, x2], [y1, y1, y2, y2], [z1, z1, z1, z1]
        )  # ,rstride=4, cstride=4, color='r')
        axis.plot(
            [x1, x2, x1, x2], [y1, y1, y2, y2], [z2, z2, z2, z2]
        )  # ,rstride=4, cstride=4, color='r')
        axis.plot(
            [x1, x1, x1, x1], [y1, y1, y2, y2], [z1, z2, z1, z2]
        )  # ,rstride=4, cstride=4, color='r')
        axis.plot(
            [x2, x2, x2, x2], [y1, y1, y2, y2], [z1, z2, z1, z2]
        )  # ,rstride=4, cstride=4, color='r')
        axis.plot(
            [x1, x2, x1, x2], [y1, y1, y1, y1], [z1, z1, z2, z2]
        )  # ,rstride=4, cstride=4, color='r')
        axis.plot(
            [x1, x2, x1, x2], [y2, y2, y2, y2], [z1, z1, z2, z2]
        )  # ,rstride=4, cstride=4, color='r')

    axis.axis("auto")


def plotPlanarArm(figure, axis, arm, conf, color, width):
    # %PLOTPLANARARM Plot Arm class in 2D
    # %
    # %   Usage: PLOTPLANARARM(arm, conf, color, width)
    # %   @arm    Arm object
    # %   @conf   arm configuration vector
    # %   @color  color string, use plot convention, e.g. 'r' is red
    # %   @width  line width

    # TODO: check if rows arnd colums are correct
    position = arm.forwardKinematicsPosition(np.array(conf))
    print("************")
    position = position[0:2, :]
    position = np.append(np.asarray([0, 0]).reshape(2, 1), position, axis=1)

    # marker='-'
    axis.plot(position[0, :], position[1, :], color=color, linewidth=width)
    axis.plot(position[0, :], position[1, :], "k.", markersize=20)


# def plotPlanarMobile2Arms(figure, axis, marm, p, vehsize, color, width):
# 	#PLOTPLANARMOBILE2ARMS Summary of this function goes here
# 	#   Detailed explanation goes here
# 	# color  = [(r,g,b)] where all values lie between 0 and 1
# 	pose = p.pose()
# 	# vehicle corners
# 	corner1 = pose.transform_from(Point2(vehsize[0]/2, vehsize[1]/2))
# 	corner2 = pose.transform_from(Point2(-vehsize[1]/2, vehsize[2]/2))
# 	corner3 = pose.transform_from(Point2(-vehsize[1]/2, -vehsiz[2]/2))
# 	corner4 = pose.transform_from(Point2(vehsize[1]/2, -vehsize[2]/2))

# 	# vehicle base black lines
# 	x_corners = [corner1.x(), corner2.x(), corner3.x(), corner4.x(), corner1.x()]
# 	y_corners = [corner1.y(), corner2.y(), corner3.y(), corner4.y(), corner1.y()]
# 	axis.plot(x_corners, y_corners, 'k-')

# 	# arm
# 	position = marm.forwardKinematicsPosition(p)
# 	position = position[0:2, :] # Todo: check rows and columns

# 	#style = strcat(color, '-');

# 	axis.plot(position[0,0:marm.arm1.dof+1], position[1,0:marm.arm1.dof+1], color=color, linewidth=width)
# 	axis.plot(position[0,[0,marm.arm1.dof+1:end+1]], position[1,[0,marm.arm1.dof+1:end+1]], color=color, linewidth=width)

# 	axis.plot(position[0,0:marm.arm1.dof+1], position[1,0:marm.arm1.dof+1], 'k.', markersize=5);
# 	axis.plot(position[0,marm.arm1.dof+1:end], position[1,marm.arm1.dof+1:end], 'k.', markersize=5);


def plotPlannarMobileArm(figure, axis, marm, p, vehsize, color, width):
    # PLOTPLANNARMOBILEARM Summary of this function goes here
    #   Detailed explanation goes here

    pose = p.pose()
    # vehicle corners
    corner1 = pose.transform_from(Point2(vehsize[0] / 2, vehsize[1] / 2))
    corner2 = pose.transform_from(Point2(-vehsize[0] / 2, vehsize[1] / 2))
    corner3 = pose.transform_from(Point2(-vehsize[0] / 2, -vehsize[1] / 2))
    corner4 = pose.transform_from(Point2(vehsize[0] / 2, -vehsize[1] / 2))

    # vehicle base black lines
    x_corners = [corner1.x(), corner2.x(), corner3.x(), corner4.x(), corner1.x()]
    y_corners = [corner1.y(), corner2.y(), corner3.y(), corner4.y(), corner1.y()]
    axis.plot(x_corners, y_corners, "k-")

    # arm
    position = marm.forwardKinematicsPosition(p)
    position = position[0:2, :]

    axis.plot(position[0, :], position[1, :], color=color, linewidth=width)

    axis.plot(position[0, 0:end], position[1, 0:end], "k.", markersize=5)


def plotPlanarMobileBase(figure, axis, robot, pose, vehsize, color, width):
    # PLOTPLANARMOBILEBASE Summary of this function goes here
    #   Detailed explanation goes here

    # vehicle corners
    corner1 = pose.transform_from(Point2(vehsize[0] / 2, vehsize[1] / 2))
    corner2 = pose.transform_from(Point2(-vehsize[0] / 2, vehsize[1] / 2))
    corner3 = pose.transform_from(Point2(-vehsize[0] / 2, -vehsize[1] / 2))
    corner4 = pose.transform_from(Point2(vehsize[0] / 2, -vehsize[1] / 2))

    # vehicle base black lines
    x_corners = [corner1.x(), corner2.x(), corner3.x(), corner4.x(), corner1.x()]
    y_corners = [corner1.y(), corner2.y(), corner3.y(), corner4.y(), corner1.y()]
    axis.plot(x_corners, y_corners, "k-")


def plotPointRobot2D(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
    # %plotPointRobot2D Plot PointRobotModel in 2D
    # %
    # %   Usage: plotRobotModel(robot, conf, color_rgb)
    # %   @robot      PointRobotModel object
    # %   @conf       robot configuration vector
    # %   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

    # points
    body_points = conf
    r = robot.sphere_radius(0)

    theta = np.linspace(0, 2 * np.pi, num=40)
    x = r * np.cos(theta) + body_points[0]
    y = r * np.sin(theta) + body_points[1]
    axis.plot(x, y, color=color_rgb)
    # axis.plot([conf.x(), conf.x()+ r* np.cos(conf.theta())], [conf.y(), conf.y()+ r * np.sin(conf.theta())], color='b')
    # axis.plot([conf[0], conf[0]+ r* np.cos(conf[2])], [conf[1], conf[1]+ r * np.sin(conf[2])], color='b')
    if np.asarray(conf).shape[0] == 3:
        axis.plot(
            [conf[0], conf[0] + r * np.cos(conf[2])],
            [conf[1], conf[1] + r * np.sin(conf[2])],
            color="b",
        )


def plotPointRobot2D_theta(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
    # %plotPointRobot2D Plot PointRobotModel in 2D
    # %
    # %   Usage: plotRobotModel(robot, conf, color_rgb)
    # %   @robot      PointRobotModel object
    # %   @conf       robot configuration vector
    # %   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

    # points
    body_points = conf
    r = robot.sphere_radius(0)

    theta = np.linspace(0, 2 * np.pi, num=40)
    x = r * np.cos(theta) + body_points[0]
    y = r * np.sin(theta) + body_points[1]
    axis.plot(x, y, color=color_rgb)
    # axis.plot([conf.x(), conf.x()+ r* np.cos(conf.theta())], [conf.y(), conf.y()+ r * np.sin(conf.theta())], color='b')
    axis.plot(
        [conf[0], conf[0] + r * np.cos(conf[2])],
        [conf[1], conf[1] + r * np.sin(conf[2])],
        color="b",
    )


def plotSphere(figure, axis, radius, center, color):
    # Make data
    u = np.linspace(0, 2 * np.pi, 10)
    v = np.linspace(0, np.pi, 10)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]

    # Plot the surface
    axis.plot_surface(x, y, z, rstride=4, cstride=4, color=color)


def plotRobotModel(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
    # plotRobotModel Plot RobotModel class in 3D, visualize the body spheres
    #   also it can plot any child class of RobotModelm like ArmModel
    #
    #   Usage: plotRobotModel(robot, conf, color_rgb)
    #   @robot      RobotModel(or child) object
    #   @conf       robot configuration vector
    #   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

    # points
    body_points = robot.sphereCentersMat(conf)

    for i in range(robot.nr_body_spheres()):
        # TODO: check if it is body_points[:,i] or body_point[i,:]
        plotSphere(
            figure, axis, robot.sphere_radius(i), body_points[:, i], color=color_rgb
        )


def plotSignedDistanceField2D(
    figure, axis, field, origin_x, origin_y, cell_size, epsilon_dist=0
):
    # %PLOTSIGNEDDISTANCEFIELD2D plot 2D SignedDistanceField
    # %
    # %   Usage: PLOTSIGNEDDISTANCEFIELD2D(field, origin_x, origin_y, cell_size, epsilon_dist)
    # %   @field                  field matrix
    # %   @origin_x, origin_y     origin (down-left) corner of the map
    # %   @cell_size              cell size
    # %   @epsilon_dist           optional plot obstacle safety distance, default = 0

    # get X-Y coordinates
    grid_rows = field.shape[0]
    grid_cols = field.shape[1]
    grid_corner_x = origin_x + (grid_cols - 1) * cell_size
    grid_corner_y = origin_y + (grid_rows - 1) * cell_size

    grid_X = np.linspace(origin_x, grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin_y, grid_corner_y, num=grid_rows)

    z_min = np.amin(field)
    z_max = np.amax(field)
    c = axis.pcolor(grid_X, grid_Y, field, cmap="RdBu", vmin=z_min, vmax=z_max)
    figure.colorbar(c, ax=axis)  # add colorbar

    # set(gca,'YDir','normal')
    axis.invert_yaxis()  # TODO: check this again! same as set(gca,'YDir','normal')

    axis.axis("equal")
    axis.axis(
        [
            origin_x - cell_size / 2,
            grid_corner_x + cell_size / 2,
            origin_y - cell_size / 2,
            grid_corner_y + cell_size / 2,
        ]
    )

    # colorbar
    axis.set_xlabel("X/m")
    axis.set_ylabel("Y/m")
    axis.set_title("Signed Distance Field")


def plotSignedDistanceField3D(
    figure, axis, field, origin, cell_size, epsilon_dist=0, marker_size=10
):
    # %PLOTSIGNEDDISTANCEFIELD3D plot 3D SignedDistanceField
    # %
    # %   Usage: PLOTSIGNEDDISTANCEFIELD3D(field, origin, cell_size, epsilon_dist)
    # %   @field                  field 3D matrix
    # %   @origin                 origin of the map
    # %   @cell_size              cell size
    # %   @epsilon_dist           optional plot obstacle safety distance, default = 0
    # %   @marker_size            marker size, default = 0
    # % Here, note that axis = figure.gca(projection='3d')

    # get X-Y coordinates
    grid_rows = field.shape[0]
    grid_cols = field.shape[1]
    grid_z = field.shape[2]
    grid_corner_x = origin[0] + (grid_cols - 1) * cell_size
    grid_corner_y = origin[1] + (grid_rows - 1) * cell_size
    grid_corner_z = origin[2] + (grid_z - 1) * cell_size
    grid_X = np.linspace(origin[0], grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin[1], grid_corner_y, num=grid_rows)
    grid_Z = np.linspace(origin[2], grid_corner_z, num=grid_z)

    x, y, z = np.nonzero(field > epsilon_dist)
    vals = -field[x, y, z]
    axis.scatter(grid_X[x], grid_Y[y], grid_Z[z], s=marker_size , c=vals, cmap='YlGn_r')
    
    axes_lim = [
            origin[0] - cell_size / 4,
            grid_corner_x + cell_size / 2,
            origin[1] - cell_size / 2,
            grid_corner_y + cell_size / 4,
            origin[2] - cell_size / 2,
            grid_corner_z + cell_size / 4,
    ]

    #axis.axes.set_xlim3d(left=axes_lim[0], right=axes_lim[1])
    #axis.axes.set_ylim3d(bottom=axes_lim[2], top=axes_lim[3])
    #axis.axes.set_zlim3d(bottom=axes_lim[4], top=axes_lim[5]) 

    axis.set_xlabel("X/m")
    axis.set_ylabel("Y/m")
    axis.set_zlabel("Z/m")
    axis.set_title("Signed Distance Field")


def plotSphere2D(figure, axis, radius, center, color):
    # Make data
    u = np.linspace(0, 2 * np.pi, 100)

    x = radius * np.cos(u) + center[0]
    y = radius * np.sin(u) + center[1]

    # Plot the surface
    axis.plot(x, y, color=color)


def plotRobotModel2D(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
    # plotRobotModel Plot RobotModel class in 3D, visualize the body spheres
    #   also it can plot any child class of RobotModelm like ArmModel
    #
    #   Usage: plotRobotModel(robot, conf, color_rgb)
    #   @robot      RobotModel(or child) object
    #   @conf       robot configuration vector
    #   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

    # points
    body_points = robot.sphereCentersMat(conf)

    for i in range(robot.nr_body_spheres()):
        # TODO: check if it is body_points[:,i] or body_point[i,:]
        plotSphere2D(
            figure, axis, robot.sphere_radius(i), body_points[:, i], color=color_rgb
        )


def set3DPlotRange(figure, axis, dataset):
    # %SET3DPLOTRANGE Set figure axis range for 3D dataset
    # %
    # %   Usage: SET3DPLOTRANGE(dataset)
    # %   @dataset    dataset output by generate3Ddataset

    x1 = dataset.origin_x
    x2 = dataset.cols * dataset.cell_size + dataset.origin_x
    y1 = dataset.origin_y
    y2 = dataset.rows * dataset.cell_size + dataset.origin_y
    z1 = dataset.origin_z
    z2 = dataset.z * dataset.cell_size + dataset.origin_z

    axis.axis([x1, x2, y1, y2])
