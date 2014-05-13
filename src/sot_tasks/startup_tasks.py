from sot_tasks import createEqualityTask, createJointLimitsTask
from sot_ros_api.utilities.sot import pop, push
from sot_ros_api.sot_robot.prologue import robot, solver

from .meta_task_selfcollision_avoidance import *

'''
add self-collision at one point
exclude all configuration for self-collision into external file
'''

# Basic stack
def basicStack():
    push(taskJL)
    solver.addContact(taskBASE)
    push(taskSC)

taskBASE = createEqualityTask('baseContact','base_joint',10)
taskJL = createJointLimitsTask(200)
taskSC = getSelfCollisionTask()


