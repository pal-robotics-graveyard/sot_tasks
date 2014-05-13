from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import *
from dynamic_graph.sot.core.meta_tasks_kine import *

class MetaTaskIneqKine6d(MetaTaskKine6d):
    def createTask(self):
        self.task = TaskInequality('inequalitytask'+self.name)
        
    def createFeatures(self):
        self.feature    = FeaturePoint6d('ineqfeature'+self.name)
        self.featureDes = FeaturePoint6d('ineqfeature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')
