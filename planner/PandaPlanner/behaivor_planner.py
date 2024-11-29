
import numpy as np

class BehaivorPlanner:
    def __init__(self, RoutePlanner) -> None:
        '''
        param:
        RoutePlanner: 全局路径规划器
        '''
        self.RoutePlanner = RoutePlanner
        self.ChangeLeft  = False # 左换道布尔量
        self.ChangeRight = False # 右换道布尔量
        self.WaitChangeLeft = 0 # 左换道等待时间
        self.WaitChangeRight = 0 # 右换道等待时间
        self.MaxWaitTime = 5 # 最大等待时间

    def LaneChangeAble(self,Current):
        '''
        function: 检查当前道路的换道可行性
        '''
        if Current in self.RoutePlanner.ForbiddenPath: # 如果当前道路在禁止换道路段内，则左右换道均为false
            self.ChangeLeft = False
            self.ChangeRight = False
            return
        Left = self.RoutePlanner.road_info.discretelanes[Current].leftlane
        Right = self.RoutePlanner.road_info.discretelanes[Current].rightlane
        if Left == '' and Right != '': # 如果左侧无路段且右侧有路段，则右换道为True，否则为False； 只有最左侧车道时，右换道才为真
            self.ChangeRight = True
        else:
            self.ChangeRight = False
        if Left != '': # 如果左侧非空，则左换道为True，否则为False
            self.ChangeLeft = True
        else:
            self.ChangeLeft = False

    def CalcBehaivorTest(self,CurrentLane,LaneChangeSpace,observation,InCurve):
        '''
        function: 计算车辆的换道行为， 左换道为 -1， 右换道为 1
        '''
        self.LaneChangeAble(CurrentLane)
        LeftSpace = LaneChangeSpace[0]
        LeftSpeed = LaneChangeSpace[1]
        CurSpace  = LaneChangeSpace[2]
        CurSpeed  = LaneChangeSpace[3]
        RightSpace = LaneChangeSpace[4]
        RightSpeed = LaneChangeSpace[5]


        print("LeftSpace:", LeftSpace, "LeftSpeed:", LeftSpeed)
        print("rightSpace:", RightSpace, "RightSpeed:", RightSpeed)
        print("CurSpace:", CurSpace, "CurSpeed:", CurSpeed,"CurLane:",CurrentLane,"lane Change able:",self.ChangeLeft,self.ChangeRight)
        if CurSpace < 50 and InCurve == 0:
            # 如果前方空间不足50m，且无强制换道指令（全局路径换道），则进入决策过程
            if self.ChangeLeft and (LeftSpeed - CurSpeed > 5/3.6 or (LeftSpace - CurSpace > 7 and LeftSpeed > CurSpeed - 1)) and observation.object_info['pedestrian'] == {}:
                print("-------------Wait For LaneChange Left---------------------",self.WaitChangeLeft)
                # 如果左侧车道速度比当前车道快5km/h 或 空间比当前车道长超过7m 并且速度接近 且 无行人干扰
                self.WaitChangeLeft += 1 # 左等待时间增加
                if self.WaitChangeLeft >= self.MaxWaitTime: # 如果到达最大等待时间 则返回左换道指令
                    print("------------- LaneChange Left---------------------")
                    return -1
            elif self.ChangeRight and (RightSpeed - CurSpeed > 5/3.6 or (RightSpace - CurSpace > 7 and RightSpeed > CurSpeed - 1)) and observation.object_info['pedestrian'] == {}:
                # 如果右侧车道速度比当前车道快5km/h 或 空间比当前车道长超过7m 并且速度接近 且 无行人干扰
                self.WaitChangeRight += 1 # 右等待时间增加
                print("-------------Wait For LaneChange Right---------------------",self.WaitChangeRight)
                if self.WaitChangeRight >= self.MaxWaitTime: # 如果到达最大等待时间
                    print("------------- LaneChange Right---------------------")
                    return 1
            else:
                print("-------------Lane Keeping---------------------")
                return 0
        else: # 不满足换道决策条件，清空换道意图
            print("-------------Lane Keeping---------------------")
            self.WaitChangeLeft = 0
            self.WaitChangeRight = 0
            return 0
        return 0
