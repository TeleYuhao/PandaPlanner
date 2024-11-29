# -*- coding: utf-8 -*-

'''openD路网的散点形式，以DiscreteNetwork类进行描述'''
import numpy as np
import warnings
from typing import *
from utils.opendrive2discretenet.utils import isPointIn


class DiscreteLane:
    '''离散化的车道对象，左右边界通过散点形式给出
    '''
    def __init__(
        self,
        parametric_lane_group,
        left_vertices: np.ndarray,
        center_vertices: np.ndarray,
        right_vertices: np.ndarray,
        length: np.float64,
        curvature:np.float64,
        lane_id,
        predecessor=None,
        successor=None,
        leftlane=None,
        rightlane=None,
        left_color = None,
        left_type = None,
        right_color = None,
        right_type = None,
    ):
        self._parametric_lane_group = parametric_lane_group
        self._left_vertices = left_vertices
        self._center_vertices = center_vertices
        self._right_vertices = right_vertices
        self._lane_id = lane_id
        self._predecessor = predecessor
        self._successor = successor
        self._rightlane = leftlane
        self._rightlane = rightlane
        self._length = length
        self._curvature = curvature
        self._left_color = left_color
        self._left_type = left_type
        self._right_color = right_color
        self._right_type = right_type

    @property
    def lane_id(self) -> int:
        return self._lane_id

    @lane_id.setter
    def lane_id(self, id_: int):
        self._lane_id = id_

    @property
    def left_vertices(self) -> np.ndarray:
        return self._left_vertices

    @left_vertices.setter
    def left_vertices(self, polyline: np.ndarray):
        self._left_vertices = polyline

    @property
    def center_vertices(self) -> np.ndarray:
        return self._center_vertices

    @center_vertices.setter
    def center_vertices(self, polyline: np.ndarray):
        self._center_vertices = polyline

    @property
    def right_vertices(self) -> np.ndarray:
        return self._right_vertices

    @right_vertices.setter
    def right_vertices(self, polyline: np.ndarray):
        self._right_vertices = polyline

    @property
    def predecessor(self) -> list:
        return self._predecessor

    @predecessor.setter
    def predecessor(self, predecessor: list):
        self._predecessor = predecessor

    @property
    def successor(self) -> list:
        return self._successor

    @successor.setter
    def successor(self, successor: list):
        self._successor = successor

    @property
    def leftlane(self) -> list:
        return self._leftlane

    @leftlane.setter
    def leftlane(self, leftlane: list):
        self._leftlane = leftlane

    @property
    def rightlane(self) -> list:
        return self._rightlane

    @rightlane.setter
    def rightlane(self, rightlane: list):
        self._rightlane = rightlane

    @property
    def length(self) -> np.float64:
        return self._length

    @length.setter
    def length(self, length: np.float64):
        self._length = length

    @property
    def curvature(self) -> np.float64:
        return self._curvature

    @curvature.setter
    def curvature(self, curvature: np.float64):
        self._curvature = curvature

    @property
    def left_color(self) -> int:
        return self._left_color

    @left_color.setter
    def left_color(self, left_color: int):
        self._left_color = left_color

    @property
    def left_type(self) -> int:
        return self._left_type

    @left_type.setter
    def left_type(self, left_type: int):
        self._left_type = left_type

    @property
    def right_color(self) -> int:
        return self._right_color

    @right_color.setter
    def right_color(self, right_color: int):
        self._right_color = right_color


    @property
    def right_type(self) -> int:
        return self._right_type

    @right_type.setter
    def right_type(self, right_type: int):
        self._right_type = right_type

    def getPolygon(self) -> np.array:
        result = np.array([])
        result = np.append(result,self.right_vertices)
        result = np.append(result,self.left_vertices[::-1])
        result = np.append(result,self.right_vertices[0])
        result = result.reshape(-1,2)
        return result



class DiscreteNetwork:
    '''离散化的OpenDRIVE路网对象
    '''
    def __init__(self) -> None:
        self._discretelanes: Dict[int, DiscreteLane] = {}

    @property
    def discretelanes(self) -> List[DiscreteLane]:
        return list(self._discretelanes.values())

    def add_discretelane(self, lane: DiscreteLane):
        assert isinstance(lane, DiscreteLane), 'provided lane is not of ' \
            'type DiscreteLane! type = {}'.format(type(lane))
        if lane.lane_id in self._discretelanes.keys():
            warnings.warn('This lane already exited in network!')
            return False
        else:
            self._discretelanes[lane.lane_id] = lane
            return True
        
    def getLaneDetail(self,ID):
        for i in self.discretelanes:
            if i.lane_id == ID:
                return i
            
    def getLaneId(self,lng,lat):
        for i in self.discretelanes:
            if isPointIn([lng,lat],i.getPolygon()):
                return i.lane_id
        return None
            
    

