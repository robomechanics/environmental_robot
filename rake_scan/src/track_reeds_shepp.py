import numpy as np
import scipy
import reeds_shepp
import pybullet as p

class track_reeds_shepp(object):
    def __init__(self,rho = 3, step_size = 0.05,
            minTrackDistance = 0.2,
            kp = np.array([5,5]),
            kd = np.array([2,2]),
            goalReachCriteria = 0.2,
            maxThrottle=1):
        # params for path generation
        self.rho = rho
        self.step_size = step_size
        # params for path tracking
        self.minTrackDistance = minTrackDistance
        self.kp = kp
        self.kd = kd
        self.goalReachCriteria = goalReachCriteria
        # id for plotting
        self.pathLines = []
        self.trackPointPlotID = -1
        self.maxThrottle=maxThrottle
    def setGoal(self,robotState,goalState,physicsClientId=None,terrain=None):
        goalState = np.array(goalState)
        if len(goalState.shape) < 2:
            goalState = goalState.unsqueeze(0)
        self.path = np.array([])
        lastPoint = robotState
        for i in range(goalState.shape[0]):
            path = reeds_shepp.path_sample(lastPoint,goalState[i,:],self.rho,self.step_size)
            lastPoint = goalState[i,:]
            self.path = torch.cat((self.path,torch.tensor(path)),dim=0)
        self.path[:,-1] = 1
        if not terrain is None:
            z = scipy.interpolate.interpn((terrain.gridX[0,:],terrain.gridY[:,0]),
                                        terrain.gridZ.transpose(),
                                        self.path[:,:2])
            self.path[:,-1] = torch.tensor(z)+0.25
        self.resetTracking()
        if not physicsClientId is None:
            for i in range(self.path.shape[0]-1):
                plotParams = {'lineFromXYZ': self.path[i,:].tolist(),
                            'lineToXYZ': self.path[i+1,:].tolist(),
                            'lineColorRGB': [1,0,0], 'lineWidth': 2, 'physicsClientId': physicsClientId}
                if len(self.pathLines) > i:
                    p.addUserDebugLine(**plotParams,replaceItemUniqueId=self.pathLines[i])
                else:
                    self.pathLines.append(p.addUserDebugLine(**plotParams))
            for i in range(self.path.shape[0]-1,len(self.pathLines)):
                p.addUserDebugLine([0,0,0],[0,0,0],physicsClientId=physicsClientId,replaceItemUniqueId=self.pathLines[i])
            self.trackPointPlotID=p.addUserDebugLine([0,0,0],[0,0,0],replaceItemUniqueId=self.trackPointPlotID,physicsClientId=physicsClientId)
    def resetTracking(self):
        self.trackIndex = 0
        self.trackError = torch.tensor([0,0])
        self.atGoal = False
    def track(self,robotState,physicsClientId=None):
        robotState = robotState.to(self.path.device)
        dist2Traj = torch.norm(self.path[:,0:2]-robotState[0:2].unsqueeze(0),dim=1)
        forwardDist2Traj = (self.path[:,0]-robotState[0])*torch.cos(robotState[2]) + (self.path[:,1]-robotState[1])*torch.sin(robotState[2])
        closestIndex = torch.argmin(dist2Traj)
        windowLow = max(closestIndex,self.trackIndex)
        forwardDistCriteria = forwardDist2Traj.abs()[windowLow:] > self.minTrackDistance
        forwardDistCriteria[-1] = True
        self.trackIndex = torch.arange(windowLow,forwardDist2Traj.shape[0])[forwardDistCriteria].min()
        trackPoint = self.path[self.trackIndex,:]
        if not physicsClientId is None:
            self.trackPointPlotID=p.addUserDebugLine([trackPoint[0],trackPoint[1],trackPoint[2]],
                                                [trackPoint[0],trackPoint[1],trackPoint[2]+10],
                                                lineColorRGB=[1,1,0],lineWidth=5,
                                                replaceItemUniqueId=self.trackPointPlotID,
                                                physicsClientId=physicsClientId)
        if dist2Traj[-1] < self.goalReachCriteria:
            self.atGoal = True
        return self.calcTrackAction(robotState,trackPoint)
    def calcTrackAction(self,robotState,trackPoint):
        relX = torch.cos(robotState[2])*(trackPoint[0]-robotState[0]) + torch.sin(robotState[2])*(trackPoint[1]-robotState[1])
        relY = -torch.sin(robotState[2])*(trackPoint[0]-robotState[0]) + torch.cos(robotState[2])*(trackPoint[1]-robotState[1])
        error = torch.tensor([relX,relY])
        dError = error - self.trackError
        action = self.kp*error + self.kd*dError
        self.trackError = error
        action[0] = torch.clip(action[0],-self.maxThrottle,self.maxThrottle)
        action[1] = torch.clip(action[1],-1,1)
        return action.tolist()
