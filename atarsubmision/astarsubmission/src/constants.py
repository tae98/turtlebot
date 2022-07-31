# Scaling factor 1grid = 100cm
dist_thresh = 0.01
scaling_factor = int(1 / dist_thresh)
# maximum number of action 
max_actions = 8
#angular step to calculate the orientation of robot
angular_step = 20
# time between each action
time_step = 1
total_time = 10
time_scaling = 0.5
# robots physical dimension in cm
robot_diameter = 30.6
robot_radius = robot_diameter / 2
wheel_distance = 24.4
wheel_radius = 3.3
#threshhold for goal points
goal_thresh = robot_radius
# angle range in degreee
total_angle = 360
#nodes constant value for children and parents
no_parent = -1
node_generated = 1
start_parent = -99
# dimention of the map = width *height
width, height = 12, 12
map_size = (scaling_factor * height), (scaling_factor * width)
map_center = (map_size[0] // 2), (map_size[1] // 2)
