# -*- coding: utf-8 -*-

import numpy

 


def encode_road_section_lane_width_id(roadId, sectionId, laneId, widthId):
    """

    Args:
      roadId:
      sectionId:
      laneId:
      widthId:

    Returns:

    """
    return ".".join([str(roadId), str(sectionId), str(laneId), str(widthId)])


def decode_road_section_lane_width_id(encodedString: str):
    """

    Args:
      encodedString:

    Returns:

    """

    parts = encodedString.split(".")

    if len(parts) != 4:
        raise Exception()

    return (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))


def allCloseToZero(array):
    """Tests if all elements of array are close to zero.

    Args:
      array:

    Returns:

    """

    return numpy.allclose(array, numpy.zeros(numpy.shape(array)))


def isPointIn(point, rangelist):
    # 判断是否在外包矩形内，如果不在，直接返回false
    lnglist = []
    latlist = []
    for i in range(len(rangelist) - 1):
        lnglist.append(rangelist[i][0])
        latlist.append(rangelist[i][1])
    # print(lnglist, latlist)
    maxlng = max(lnglist)
    minlng = min(lnglist)
    maxlat = max(latlist)
    minlat = min(latlist)
    # print(maxlng, minlng, maxlat, minlat)
    if (point[0] > maxlng or point[0] < minlng or
            point[1] > maxlat or point[1] < minlat):  # 超出多边形边界
        return False
    count = 0
    point1 = rangelist[0]
    for i in range(1, len(rangelist)):
        point2 = rangelist[i]
        # 点与多边形顶点重合
        if (point[0] == point1[0] and point[1] == point1[1]) or (point[0] == point2[0] and point[1] == point2[1]):
            # print("在顶点上")
            return True
        # 判断线段两端点是否在射线两侧 不在肯定不相交 射线（-∞，lat）（lng,lat）
        if (point1[1] < point[1] <= point2[1]) or (point1[1] >= point[1] > point2[1]):
            # 求线段与射线交点 再和lat比较
            point12lng = point2[0] - (point2[1] - point[1]) * (point2[0] - point1[0]) / (point2[1] - point1[1])
            # print(point12lng)
            # 点在多边形边上
            if point12lng == point[0]:
                # print("点在多边形边上")
                return True
            if point12lng < point[0]:
                count += 1
        point1 = point2
    # print(count)
    if count % 2 == 0:
        return False
    else:
        return True
