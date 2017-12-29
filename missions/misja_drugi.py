import executive_engine_api as api
#import numpy as np
#import time

def runMission():
  
  api.activateBehavior('SELF_LOCALIZE_BY_ODOMETRY')
  api.activateBehavior('FOLLOW_UAV', droneID=1)

