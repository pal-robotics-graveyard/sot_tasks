from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint, FeatureVisualPoint

from dynamic_graph.sot_tasks.feature_grasping_point import *

class MetaTaskGraspingPoint(MetaTaskVisualPoint):

    def createFeatures(self):
        self.feature    = FeatureGraspingPoint('feature'+self.name)
        self.featureDes = FeatureVisualPoint('feature'+self.name+'_ref')
        self.feature.selec.value = '11'

    def __init__(self,name,dyn,opPoint,opPointRef='right-wrist'):
	MetaTaskVisualPoint.__init__(self, name, dyn, opPoint, opPointRef)
