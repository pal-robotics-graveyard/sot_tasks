from sot_ros_api.sot_robot.prologue import robot, solver
from dynamic_graph import plug

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from sot_tasks.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier
import yaml

'**********************************'
'initialize yaml file'
'**********************************'
from rospkg import RosPack
rp = RosPack()
folder_path = rp.get_path('sot_tasks')
file_path = str(folder_path)+'/config/'+'self_collision_full.yaml'
file_stream = file(file_path, 'r')
yaml_parser = yaml.load(file_stream)

'**********************************'
'read out yaml file '
'**********************************'
collision_objects = yaml_parser['collision_objects']
collision_pairs = yaml_parser['collision_pairs']

'**********************************'
'init entfcl and all pointers for possible collision pairs'
''' This is a design choice here, that the complete collision matrix
based on ALL collision_objects will be initialized.
Later, only the necessary collision pairs are plugged in the taskDAMP
This is valid, because the DG only computes the pointers which are actually
requested by some other entities.'''
'**********************************'

entFCL = DynamicGraphFCL('entFCL')
for collision in collision_objects:
	entFCL.addCollisionJoint(collision)
        robot.dynamic.createOpPoint(collision, collision)
	robot.dynamic.signal(collision).recompute(0)
entFCL.finalizeSignals()
# enable a rostopic publisher for capusle information
#entFCL.enableCapsuleTopic(True)

# plug the forward kinematics into the input signals
for collision in collision_objects:
	plug(robot.dynamic.signal(collision), entFCL.signal(collision))

# cross init signals/matrix
for i in range(len(collision_objects)):
	for j in range(len(collision_objects)):
		if not(collision_objects[i] == collision_objects[j]):
			signal_name = str(collision_objects[i])+str(collision_objects[j])
			entFCL.signal(signal_name).recompute(0)

'**********************************'
'init oppoint modifiers'
''' this is going to be necessary for each active moving collision point (namely link1)
    for the collision avoidance task we need a jacobian at the closest point 
rather than at the link-origin. This being sad, the entfcl bring out oppoint_<name> signals
which are relative points of the closest point pairs w.r.t the link origin
dyanamic op point modifier takes this relative point as a transformation and outputs the according jacobian

Note that we only need link1->link2 for the jacobian, we don't need the counter part jacobian link2->link1
 '''
'**********************************'
dynOP_collection = []
idx = 1
for pair in collision_pairs:
	link1 = pair['link1']
	link2 = pair['link2']
	dynOP = DynamicOppointModifier('dynop'+str(idx))
	plug(robot.dynamic.signal(link1), dynOP.positionIN)
	plug(robot.dynamic.signal('J'+link1), dynOP.jacobianIN)
	plug(entFCL.signal('oppoint_'+link1+link2), dynOP.transformationSig)
	dynOP_collection.append(dynOP)
	idx +=1

'**********************************'
'init taskDAMP for actual collision avoidance	'
'''  for each collision pair we init the following signals
	- p1_<link1link2> = considered as the moving part for which the jacobian has to be computed
	- p2_<link1link2> = considered as the static part which has to be avoided
	- ds_<link1link2> = security distance between p1 and p2 which is not allowed to get violated
	- di_<link1link2> = influence distance on which the inequality becomes part of the active search set
'''
'**********************************'

taskDAMP = MetaTaskDynamicVelocityDamping('selfcollision')
for pair in collision_pairs:
	taskDAMP.task.setAvoidingObjectPair(pair['link1'], pair['link2'])
#create all the signals
taskDAMP.task.finalizeSignals()
taskDAMP.task.controlGain.value = 1

# optional: expose closest points as a ROS topic to visualize in RVIZ marker node
for idx in range(len(collision_pairs)):
	link1 = collision_pairs[idx]['link1']
	link2 = collision_pairs[idx]['link2']
	ds = collision_pairs[idx]['ds']
	di = collision_pairs[idx]['di']
	dynOP = dynOP_collection[idx]

	plug(entFCL.signal(link1+link2), taskDAMP.task.signal('p1_'+link1+link2))
	plug(entFCL.signal(link2+link1), taskDAMP.task.signal('p2_'+link1+link2))
	plug(dynOP.jacobian, taskDAMP.task.signal('jVel_'+link1+link2) )
	taskDAMP.task.signal('ds_'+link1+link2).value = ds
	taskDAMP.task.signal('di_'+link1+link2).value = di

def getSelfCollisionTask():
    return taskDAMP

__all__ = ['getSelfCollisionTask']
