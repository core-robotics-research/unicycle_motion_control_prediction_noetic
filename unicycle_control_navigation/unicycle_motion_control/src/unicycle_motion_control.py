"""
Unicycle Distance Gradient Motion Control
"""

import numpy as np
from scipy.integrate import solve_ivp

# For the union function to compute the truncated ice cream cone
from shapely.geometry import Polygon

from geometry_tools import circle_polygon, convexhull_polygon


####################################################################################
############################ UNICYCLE DYNAMICS #####################################
####################################################################################

def unicycle_discrete_update(position, orientation, linear_velocity, angular_velocity, dt):
  """
  Computes unicycle discrete state update assuming contant velocity 
  """
  position[0] = position[0] + linear_velocity * np.cos(orientation) * dt
  position[1] = position[1] + linear_velocity * np.sin(orientation) * dt
  orientation = orientation + angular_velocity * dt
  return position, orientation

def unicycle_discrete_update_ode(position, orientation, linear_velocity, angular_velocity, dt, **kwargs):
  
  initial_pose = np.array([position[0], position[1], orientation])

  unicycle_dynamics = lambda t, pose: np.array([linear_velocity*np.cos(pose[2]), linear_velocity*np.sin(pose[2]), angular_velocity])

  sol = solve_ivp(unicycle_dynamics, [0, dt], initial_pose, **kwargs)

  position = [sol.y[0][-1], sol.y[1][-1]]
  orientation = sol.y[2][-1]

  return position, orientation


####################################################################################
############################ UNICYCLE CONTROL FUNCTION #############################
####################################################################################

def distance_gradient_ctrl(position, orientation, goal, lingain=1.0, anggain=1.0, tol=1e-3, motion_direction = 'Bidirectional'):
    """
    Computes projected distance motion control inputs for the kinematic unicycle robot model
    toward a given goal location as in
    "Feedback Motion Prediction for Safe Unicycle Robot Navigation." arXiv preprint arXiv:2209.12648 (2022).


    Function:
    linvel, angvel = distance_gradient_ctrl(position, orientation, goal, lingain, anggain, tol, motion_direction)
    Input:
    position :  Unicycle Position [float float]
    orientation: Unicycle Orientation [float]
    goal : Goal Position [float float]
    lingain: Control Gain for Linear Motion, [float], (default, 1.0)
    anggain: Control Gain for Angular Motion, [float], (default, 1.0)
    tol: Motion Control Tolerance, [float], (default, 1e-3)
    motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')
    Output:
    linvel: Linear Velocity  (unit/s) [float]
    angvel: Angular Velocity (rad/s) [float]
    Usage:
    position = [0.0, 0.0]
    orientation = 0.0
    goal = [1.0, 0.0]
    lingain = 1.0
    anggain = 1.0
    tol = 1e-5
    motion_direction = 'Bidirectional'
    linvel, angvel = distance_gradient_ctrl(position, orientation, goal, lingain, anggain, tol, motion_direction)
    """
    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position
    tol = np.float64(tol) # Tolerance
    motion_direction = motion_direction.lower()

    if not (motion_direction == 'forward' or motion_direction == 'backward' or motion_direction == 'bidirectional'):
        raise ValueError("Motion Direction must be 'Forward', 'Backward', or 'Bidirectional'")

    # Check motion control tolerance
    if (np.linalg.norm(position-goal) < tol):
        return 0.0, 0.0
    
    if motion_direction == 'backward':
        orientation = orientation + np.pi

    vdir = np.array([np.cos(orientation), np.sin(orientation)]) # Current Direction
    ndir = np.array([-np.sin(orientation), np.cos(orientation)]) # Normal Direction

    vproj = np.dot(goal-position, vdir) # Projection onto velocity direction
    nproj = np.dot(goal-position, ndir) # Projection onto velocity normal

    linvel = lingain*vproj # Linear velocity control
    angvel = anggain*np.arctan2(nproj,vproj) # Angular velocity control


    # Turn in place with a continuous angular velocity until the robot is aligned with the goal
    if motion_direction == 'forward' or motion_direction == 'backward':
        linvel = np.maximum(linvel, 0)

    if motion_direction == 'backward':
        linvel = -1 * linvel

    return linvel, angvel


##################################################################################################################
############################ DISTANCE GRADIENT CONTROL DYNAMICS AND MOTION PREDICTION ###########################
##################################################################################################################

def dgc_vector_field(position, orientation, goal, lingain, anggain, tol, motion_direction = 'Bidirectional'):
    """
    Unicycle dynamics for the kinematic unicycle robot model controlled via projected distance control.
    """

    position = np.array(position)
    goal = np.array(goal)
    
    linear_velocity, angular_velocity = distance_gradient_ctrl(position, orientation, goal, lingain, anggain, tol, motion_direction)
    dx = linear_velocity*np.cos(orientation)
    dy = linear_velocity*np.sin(orientation)
    dtheta = angular_velocity
    dX = np.array([dx, dy ,dtheta])

    return dX


def dgc_trajectory(position, orientation, goal, tspan, lingain, anggain, tol, motion_direction = 'Bidirectional', **kwargs):

    """
    Computes the trajectory of the kinematic unicycle robot model controlled via projected distance control.


    Function:
    t, y = pd_trajectory(position, orientation, goal, tspan, lingain, anggain, tol, motion_direction, **kwargs)

    Input:
    position: Current position of the robot [float, float]
    orientation: Current orientation (angle) of the robot [float]
    goal: Goal position [float, float]
    tspan: Time span for trajectory [float, float]
    lingain: Control gain for linear motion [float]
    anggain: Control gain for angular motion [float]
    tol: Tolerance for motion control [float]
    motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')
    **kwargs: Keyword arguments for solve_ivp

    Output:
    t: Time vector [array]
    y: State vector [array]
    """
    unicycle_dynamics = lambda t, X: dgc_vector_field(X[0:2], X[2], goal, lingain, anggain, tol, motion_direction)

    position = np.array(position)
    initial_pose = np.append(position, orientation)

    sol = solve_ivp(unicycle_dynamics, tspan, initial_pose, max_step=1e-2, **kwargs)

    # position = [sol.y[0][-1], sol.y[1][-1]]
    # orientation = sol.y[2][-1]
    return sol.t, sol.y.T


def dgc_discrete_update(position, orientation, goal, duration, lingain, anggain, tol, motion_direction = 'Bidirectional', **kwargs):
    
        t, y = dgc_trajectory(position, orientation, goal, [0, duration], lingain, anggain, tol, motion_direction, **kwargs)
    
        position = [y[-1][0], y[-1][1]]
        orientation = y[-1][2]
    
        return position, orientation


def dgc_circle_motion_prediction(position, orientation, goal, motion_direction = 'Bidirectional', res = 60):
    """
    Computes the circular motion prediction for a robot model.
    orientation is not used in this function, but is included for compatibility with other motion prediction functions.

    Args:
        position: Current position of the robot [float, float]
        goal: Goal position [float, float]
        res: Resolution for circle approximation [int]

    Returns:
        P: Motion polygon vertices [array]
    """

    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position

    # subtract goal from the position
    position = position - goal

    if np.linalg.norm(position) < 1e-3:
        motionpolygon_radius = 0
    else:
        motionpolygon_radius = np.linalg.norm(position)

    # Compute the convex hull of the circle approximation
    P = circle_polygon(motionpolygon_radius, res)

    # Add each vertex to the polygon the goal
    for i in range(len(P)):
        P[i] = P[i] + goal

    return P


def dgc_boundedcone_motion_prediction(position, orientation, goal, motion_direction = 'Bidirectional', res = 60):
    """
    Computes the bounded cone motion prediction for a 2D Unicycle kinematic robot controlled via projected distance control.

    Args:        
        position: Current position of the robot [float, float]
        orientation: Current orientation (angle) of the robot [float]
        goal: Goal position [float, float]
        motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')

    Returns:
        P: Motion polygon vertices [array]
    """ 
    
    if motion_direction == 'backward':
        orientation = orientation + np.pi

    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position

    # subtract goal from the position
    position = position - goal

    
    vdir = np.array([np.cos(orientation), np.sin(orientation)]) # Current Direction
    ndir = np.array([-np.sin(orientation), np.cos(orientation)]) # Normal Direction

    vproj = np.dot(-position, vdir) # Projection onto velocity direction
    nproj = np.dot(-position, ndir) # Projection onto velocity normal

    if np.arctan2(nproj,vproj) > np.pi/2 or np.arctan2(nproj,vproj) < -np.pi/2:
        P = dgc_circle_motion_prediction(position, None, [0, 0], res)
    else:

        # Determine critical points
        R = np.linalg.norm(position)
        th = np.linspace(orientation - np.arctan2(nproj,vproj), orientation + 3*np.arctan2(nproj,vproj), res)
        arc = np.array([R*np.cos(th), R*np.sin(th)]).T
        # if arc is 3 dimensional, then reshape it to 2 dimensional
        if len(arc.shape) == 3:
            arc = arc.reshape((res, 2))
        P = np.vstack((arc, position))

    # Add each vertex to the polygon the goal
    for i in range(len(P)):
        P[i] = P[i] + goal

    return P


def dgc_icecreamcone_motion_prediction(position, orientation, goal, motion_direction = 'Bidirectional', res = 60):
    """
    Computes the ice cream cone motion prediction for a 2D Unicycle kinematic robot controlled via projected distance control.

    Args:        
        position: Current position of the robot [float, float]
        orientation: Current orientation (angle) of the robot [float]
        goal: Goal position [float, float]
        motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')

    Returns:
        P: Motion polygon vertices [array]
    """

    if motion_direction == 'backward':
        orientation = orientation + np.pi

    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position

    # subtract goal from the position
    position = position - goal

    
    vdir = np.array([np.cos(orientation), np.sin(orientation)]) # Current Direction
    ndir = np.array([-np.sin(orientation), np.cos(orientation)]) # Normal Direction

    vproj = np.dot(-position, vdir) # Projection onto velocity direction
    nproj = np.dot(-position, ndir) # Projection onto velocity normal

    if np.arctan2(nproj,vproj) > np.pi/2 or np.arctan2(nproj,vproj) < -np.pi/2:
        P = dgc_circle_motion_prediction(position, None, [0, 0], res)
    else:
        # Determine critical points
        R = np.linalg.norm(position)*np.abs(np.sin(np.arctan2(nproj,vproj)))
        if R < 1e-3:
            R = 1e-3
        ice = circle_polygon(R, res)
        cone = position
        P = np.vstack((ice, cone))

    P = convexhull_polygon(P)

    # Add each vertex to the polygon the goal
    for i in range(len(P)):
        P[i] = P[i] + goal

    return P


def dgc_truncatedicecreamcone_motion_prediction(position, orientation, goal, motion_direction = 'Bidirectional', res = 60):
    """
    Computes the truncated ice cream cone motion prediction for a 2D Unicycle kinematic robot controlled via projected distance control.

    Args:        
        position: Current position of the robot [float, float]
        orientation: Current orientation (angle) of the robot [float]
        goal: Goal position [float, float]
        motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')

    Returns:
        P: Motion polygon vertices [array]
    """

    if motion_direction == 'backward':
        orientation = orientation + np.pi

    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position

    # subtract goal from the position
    position = position - goal

    
    vdir = np.array([np.cos(orientation), np.sin(orientation)]) # Current Direction
    ndir = np.array([-np.sin(orientation), np.cos(orientation)]) # Normal Direction

    vproj = np.dot(-position, vdir) # Projection onto velocity direction
    nproj = np.dot(-position, ndir) # Projection onto velocity normal

    if np.arctan2(nproj,vproj) > np.pi/2 or np.arctan2(nproj,vproj) < -np.pi/2:
        P = dgc_circle_motion_prediction(position, None, [0, 0], res)
    else:
        # Determine critical points
        XCone = np.zeros((3,2))
        XCone[1,:] = position.reshape(1,2)
        XCone[2,:] = position.reshape(1,2) + np.linalg.norm(position)*np.cos(np.arctan2(nproj,vproj))*np.array([np.cos(orientation), np.sin(orientation)])

        ice_radius = np.linalg.norm(position)*np.abs(np.sin(np.arctan2(nproj,vproj)))
        if ice_radius < 1e-3:
            ice_radius = 1e-3
        XCircle = circle_polygon(ice_radius, res)

        # Take the set union of the XCircle and XCone using shapely
        # ShapelyDeprecationWarning: The 'cascaded_union()' function is deprecated.
        # use union() instead
        # P = cascaded_union([Polygon(XCircle), Polygon(XCone)])
        Union = Polygon(XCircle).union(Polygon(XCone))
        P = np.array(Union.exterior.coords)
        
    # Add each vertex to the polygon the goal
    for i in range(len(P)):
        P[i] = P[i] + goal

    return P