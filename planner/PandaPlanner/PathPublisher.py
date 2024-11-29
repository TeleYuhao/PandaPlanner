import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher:
    def __init__(self):
        # rospy.init_node('PathPublisher',anonymous=True)
        self.Publisher = rospy.Publisher('/Path',Path,queue_size=10)
        self.rate = rospy.Rate(10)
    def CreatePath(self,PlanningPath):
        PathMsg = Path()
        PathMsg.header.frame_id = 'map'
        PathMsg.header.stamp = rospy.Time.now()
        for i in range(len(PlanningPath)):
            pose = PoseStamped()
            pose.pose.position.x = PlanningPath[i][0]
            pose.pose.position.y = PlanningPath[i][1]
            PathMsg.poses.append(pose)
        return PathMsg
    def Publish(self,PlanningPath):
        Path = self.CreatePath(PlanningPath)
        print("Path pub")
        self.Publisher.publish(Path)
            # self.rate.sleep()

if __name__ == '__main__':
    try:
        PathPub = PathPublisher()
        PathPub.Publish()
    except rospy.ROSInterruptException: pass

