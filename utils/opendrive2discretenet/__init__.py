from lxml import etree
from utils.opendrive2discretenet.opendriveparser.parser import parse_opendrive as parse_opendrive_xml
from utils.opendrive2discretenet.network import Network

def parse_opendrive(path_opendrive: str) -> None:
    """
    解析opendrive路网的信息，存储到self.replay_info.road_info。
    """
    with open(path_opendrive, 'r', encoding='utf-8') as fh:
        root = etree.parse(fh).getroot()
    
    # 返回OpenDrive类的实例对象（经过parser.py解析）
    openDriveXml = parse_opendrive_xml(root)

    # 将OpenDrive类对象进一步解析为参数化 的Network类对象，以备后续转化为DiscreteNetwork路网并可视化
    loadedRoadNetwork = Network()
    loadedRoadNetwork.load_opendrive(openDriveXml)

    """将解析完成的Network类对象转换为DiscreteNetwork路网，其中使用的只有路网中各车道两侧边界的散点坐标
        车道边界点通过线性插值的方式得到，坐标点储存在<DiscreteNetwork.discretelanes.left_vertices/right_vertices> -> List"""
    open_drive_info = loadedRoadNetwork.export_discrete_network(
        filter_types=["driving","biking", "onRamp", "offRamp", "exit", "entry", "sidewalk"])  # -> <class> DiscreteNetwork
    

    ## 获取车道详情
    open_drive_info.getLaneDetail('88.0.-1.-1')
    
    ## 根据位置获取车道 若返回为none, 则输入经纬度不在车道内
    # start = time.time()
    open_drive_info.getLaneId(108.89699297,34.37364864)
    # print(time.time() - start)

    return open_drive_info

def main():
    path_opendrive = "/home/wanji/Downloads/onsite-structured-test-0830-borrowandstopline/onsite-structured-test/scenario/replay/crossing_752_7_0/crossing_752_7_0.xodr"
    road_info = parse_opendrive(path_opendrive)
    print(road_info.discretelanes)

if __name__ == '__main__':
    main()
