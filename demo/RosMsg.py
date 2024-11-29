import math
import rospy
from perception_msgs.msg import PerceptionObjects, PerceptionLocalization, TrafficLightDetection
from control_msgs.msg import ControlCmd   
from device_msgs.msg import taskpublish  
from device_msgs.msg import participantTrajectories
import time
import numpy as np
from common_msgs.msg import Header
# from perception_msgs.msg import RoadMarkers
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
# from planner.PandaPlanner.SloveMap.RoutePlanner import
from visualization_msgs.msg import Marker,MarkerArray
from tf.transformations import quaternion_from_euler
from planner.PandaPlanner.SloveMap.RoutePlanner import tmerc2utmPoint

class RosMsg():
    def __init__(self):
        self.ego = [0, 0, 0, 0, 0, 0, 2, 5, 0, 0]  # 主车，[frame_number, lon, lat, v, a, yaw, width, length, utmx, utmy]
        self.obs = {
            'vehicle': {},
            'bicycle': {},
            'pedestrian': {},
            'construction': {},
            'tunnel': {},
            'stopline': {},
            'parking': {},
            'cone': {},
            'crossing': {},
            'others': {}
        }                           # 从车, {type : {id : {lon, lat, v, a, yaw, width, length}}}
        self.trajectory = []        # 全局路经，[[lon, lat, angle(deg,east-0/north-90), speed_limit(km/h), status(0start/1finishi/2normal/3necessary)], []]
        self.last_ego = []          # 上一帧主车信息
        self.light = []             # 红绿灯
        self.marker = []            # 路标信息，[[id, xabs, yabs, type(type ：1-直行  2-直行左转 3-直行右转 4-直行左转右转 5-左转 6-右转)],[]]
        rospy.init_node("msg_node", anonymous=True)
        rospy.Subscriber("cicv_location", PerceptionLocalization, self.UpdateEgo)
        rospy.Subscriber("tpperception", PerceptionObjects, self.UpdateObs)
        rospy.Subscriber("test_trajectory_req", participantTrajectories, self.UpdateTrajectory)
        self.pub = rospy.Publisher("cicv_control_cmd", ControlCmd, queue_size=1)
        rospy.Subscriber("tftrafficlight", TrafficLightDetection, self.UpdateLight)
        print(len(self.trajectory))
        # rospy.Subscriber("/fusion/road_markings", RoadMarkers, self.UpdateMarker)

        self.Publisher = rospy.Publisher('/Path', Path, queue_size=10)
        self.EgoPub = rospy.Publisher('/Ego', Marker, queue_size=10)
        self.ObsPub = rospy.Publisher('/Obs', MarkerArray, queue_size=10)
        self.rate = rospy.Rate(10)

    def UpdateEgo(self, data):
        """更新主车数据"""
        self.last_ego = self.ego
        self.ego = [data.frame_unmber, data.longitude, data.latitude,
                    math.sqrt(data.velocity_x ** 2 + data.velocity_y ** 2),
                    math.sqrt(data.accel_x ** 2 + data.accel_y ** 2),
                    data.yaw, 2, 5, data.position_x, data.position_y]

    def UpdateObs(self, data):
        """更新从车数据"""
        self.obs = {'vehicle': {}, 'bicycle': {}, 'pedestrian': {},
                    'construction': {}, 'tunnel': {}, 'stopline': {}, 'parking': {},
                    'cone': {}, 'crossing': {}, 'others': {}}
        # time match
        # now_time = time.time()# * 1000
        # if np.abs(now_time - data.header.time_stamp) > 2:
        #     print('obs time stamp not match')
        # else:
        for one_obs in data.objs:
            temp_type = ''
            if one_obs.type == 0 or one_obs.type == 1:      # vehicle
                temp_type = 'vehicle'
            elif one_obs.type == 3:                         # bicycle
                temp_type = 'bicycle'
            elif one_obs.type == 2:                         # pedestrian
                temp_type = 'pedestrian'
            elif one_obs.type == 17:                        # construction
                temp_type = 'construction'
            elif one_obs.type == 19:                        # stopline
                temp_type = 'stopline'
            elif one_obs.type == 20:                        # parking
                temp_type = 'parking'
            elif one_obs.type == 18:                        # tunnel
                temp_type = 'tunnel'
            elif one_obs.type == 5:                         # parking
                temp_type = 'cone'
            elif one_obs.type == 21:                        # crossing
                temp_type = 'crossing'
            else:                                           # others
                temp_type = 'others'
            if temp_type in self.obs.keys():
                self.obs[temp_type].update({str(one_obs.id) : {}})
                self.obs[temp_type][str(one_obs.id)].update({'x' : one_obs.xabs})
                self.obs[temp_type][str(one_obs.id)].update({'y': one_obs.yabs})
                self.obs[temp_type][str(one_obs.id)].update({'v': one_obs.speed})
                self.obs[temp_type][str(one_obs.id)].update({'a': 0})
                self.obs[temp_type][str(one_obs.id)].update({'yaw': one_obs.heading})
                self.obs[temp_type][str(one_obs.id)].update({'width': one_obs.width})
                self.obs[temp_type][str(one_obs.id)].update({'length': one_obs.length})
                self.obs[temp_type][str(one_obs.id)].update({'time': data.header.time_stamp})

    def UpdateTrajectory(self, data):
        """更新全局轨迹数据"""
        for one_pont in data.value:
            self.trajectory.append([one_pont.longitude, one_pont.latitude, one_pont.courseAngle, one_pont.speed, one_pont.siteType])

    def PubControl(self, action):
        """发布控制结果"""
        result = ControlCmd()
        result.acceleration = action[0]
        result.wheel_angle = action[1]
        result.gear = 3
        result.speed = action[2]
        result.light = action[3]
        self.pub.publish(result)

    def UpdateLight(self, data):
        """更新红绿灯"""
        if data.contain_lights == 1:
            for i in data.traffic_light:
                temp_light = {'shape' : i.shape, 'light' : i.color}
                self.light.append(temp_light)
        else:
            self.light = []

    def UpdateMarker(self, data):
        """更新标志"""
        for i in data.roadmarker:
            temp_marker = [i.id, i.xabs, i.yabs, i.type]
            self.marker.append(temp_marker)

    def CreatePath(self,PlanningPath):
        PathMsg = Path()
        PathMsg.header.frame_id = 'map'
        # PathMsg.header.frame_id = 'base_link'
        PathMsg.header.stamp = rospy.Time.now()
        for i in range(len(PlanningPath)):
            pose = PoseStamped()
            # x,y = tmerc2utmPoint(PlanningPath[i])
            x,y = PlanningPath[i][0],PlanningPath[i][1]
            pose.pose.position.x = x
            pose.pose.position.y = y
            PathMsg.poses.append(pose)
        return PathMsg
    def PublishPlanningPath(self,PlanningPath):
        Path = self.CreatePath(PlanningPath)
        # print("Path pub")
        self.Publisher.publish(Path)

    def GetMarker(self):
        veh = Marker()
        veh.header.frame_id = "map"
        # veh.header.frame_id = "base_link"
        veh.header.stamp = rospy.Time.now()
        veh.type = Marker.CUBE
        veh.color.r = 0.0
        veh.color.g = 0.0
        veh.color.b = 0.0
        veh.color.a = 1.0
        veh.scale.x = 5.0
        veh.scale.y = 2.0
        veh.scale.z = 1.0
        return veh
    def EgoPublish(self,observation,LaneChange):
        veh = self.GetMarker()
        if LaneChange == 0:
            veh.color.b = 1.0
        else:
            veh.color.r = 1.0
        veh.pose.position.x = observation.ego_info.x
        veh.pose.position.y = observation.ego_info.y
        veh.pose.position.z = 0
        quat = quaternion_from_euler(0, 0, np.deg2rad(observation.ego_info.yaw))
        veh.pose.orientation = Quaternion(*quat)
        self.EgoPub.publish(veh)

    def obsPub(self,observation):
        ObsArray = MarkerArray()
        i = 0
        for type,data in observation.object_info.items():
            if type != 'vehicle' and type != 'bicycle': continue
            for id,value in data.items():
                obs = self.GetMarker()
                obs.pose.position.x = value.x
                obs.pose.position.y = value.y
                obs.color.g = 1.0
                obs.pose.position.z = 0
                quat = quaternion_from_euler(0, 0, np.deg2rad(value.yaw))
                obs.pose.orientation = Quaternion(*quat)
                obs.id = i
                ObsArray.markers.append(obs)
                i += 1
        self.ObsPub.publish(ObsArray)




