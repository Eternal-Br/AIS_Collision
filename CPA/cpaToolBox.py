#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author: lph time:2019/5/18
import numpy as np
from math import *


class CPA:
    """
    :arg:
        Ref_Ship: Reference Vessel Object, Tar_Ship: Target Vessel Object

    :Instance attributes:
        self.Ref_Lon、self.Tar_Lon
        self.Ref_Lat、self.Tar_Lat
        self.Ref_Cog、self.Tar_Cog
        self.Ref_Sog、self.Tar_Sog

        self.dcpa, self.tcpa = self.cpa()

    :Instance methods:
        self.distance() -> the distance between Ref_Ship and Tar_Ship
        self.mid_point() -> the middle point between Ref_Ship and Tar_Ship
        self.bearing() -> initial bearing
        self.relative_speed() -> the relative speed() between Ref_Ship and Tar_Ship
        self.collision_risk_index() -> CRI(Collision Risk Index)
        self.cpa() -> self.dcpa, self.tcpa

    reference:
        <http://www.movable-type.co.uk/scripts/latlong.html>
        <https://cloud.tencent.com/developer/article/1055335>
        <A Novel Method for Risk Assessment and Simulation of Collision Avoidance for Vessels based on AIS>
    """
    EARTH_RADIUS = 6378.1
    n_mile = 1.852

    def __init__(self, Ref_Ship, Tar_Ship):
        self.Ref_Lon = Ref_Ship.LON
        self.Ref_Lat = Ref_Ship.LAT
        self.Tar_Lon = Tar_Ship.LON
        self.Tar_Lat = Tar_Ship.LAT
        self.Ref_Cog = Ref_Ship.COG
        self.Tar_Cog = Tar_Ship.COG

        self.Ref_Sog, self.Tar_Sog = Ref_Ship.SOG, Tar_Ship.SOG
        # self.Ref_LENGTH, self.Ref_WIDTH = Ref_Ship.LENGTH, Ref_Ship.WIDTH
        # self.Tar_LENGTH, self.Tar_WIDTH = Tar_Ship.LENGTH, Tar_Ship.WIDTH

        # 航向差delta_course
        self.delta_course = self.Tar_Cog - self.Ref_Cog

        # 速比speed_ratio
        self.speed_ratio = self.Ref_Sog / self.Tar_Sog

        # 相对方位relative bearing
        self.relative_bearing = self.bearing() - degrees(self.Ref_Cog)

        # distance for latest actions(nm)
        self.dla = 1
        self.dla2 = self.dla + self.safe_encounter_distance()

        self.dcpa, self.tcpa = self.cpa()

    def haversine(self, theta):
        return pow(sin(theta / 2), 2)

    def distance(self):
        """
        Use Haversine formula To Calculate The Distance Between Tar_Ship and Ref_Ship On The Sphere
        :return: The distance between Tar_Ship and Ref_Ship

        :unit: nautical mile(海里)
        """
        diff_lon = np.fabs(self.Tar_Lon - self.Ref_Lon)
        diff_lat = np.fabs(self.Tar_Lat - self.Ref_Lat)
        h = self.haversine(diff_lat) + np.cos(self.Tar_Lat) * np.cos(self.Ref_Lat) * self.haversine(diff_lon)
        distance = 2 * CPA.EARTH_RADIUS * asin(np.sqrt(h))
        return distance / CPA.n_mile

    def mid_point(self):
        """
        mid_point(): Mid point between two latitude and longitude
        :return: Mid Point(mid_lat, mid_lon)
        :unit: degree(°)
        """
        diff_lon = self.Tar_Lon - self.Ref_Lon
        Bx = cos(self.Tar_Lat) * cos(diff_lon)
        By = cos(self.Tar_Lat) * sin(diff_lon)
        x = sqrt((cos(self.Ref_Lat) + Bx) * (cos(self.Ref_Lat) + Bx) + pow(By, 2))
        y = sin(self.Ref_Lat) + sin(self.Tar_Lat)
        mid_lat = 180 / pi * np.arctan2(y, x)
        mid_lon = 180 / pi * (self.Ref_Lon + np.arctan2(By, cos(self.Ref_Lat) + Bx))
        return mid_lat, mid_lon

    def bearing(self):
        diff_lon = self.Tar_Lon - self.Ref_Lon

        x = sin(diff_lon) * cos(self.Tar_Lat)
        y = (cos(self.Ref_Lat) * sin(self.Tar_Lat)) - (sin(self.Ref_Lat) * cos(self.Tar_Lat) * cos(diff_lon))

        inital_bearing = np.arctan2(x, y)
        inital_bearing = np.rad2deg(inital_bearing)
        compass_bearing = (inital_bearing + 360) % 360
        return compass_bearing

    def relative_speed(self):
        """
            两船之间的相对速度
        :return: relative speed

        :unit: knot(节)
        """
        alpha = self.Tar_Cog - self.Ref_Cog
        temp = 2 * self.Tar_Sog * self.Ref_Sog * cos(alpha)
        relative_speed = sqrt(pow(self.Tar_Sog, 2) + pow(self.Ref_Sog, 2) - temp)
        return relative_speed

    def safe_distance(self):
        """
            minimum safe pass distance
        :return:
            safe distance of approach

        :unit: nautical mile(海里)
        """
        bearing = self.bearing()

        if 0 <= bearing < 112.5:
            sad = 1.1 - (0.2 * bearing) / 180
        elif 112.5 <= bearing < 180:
            sad = 1.0 - (0.4 * bearing) / 180
        elif 180 <= bearing < 247.5:
            sad = 1.0 - (0.4 * (360 - bearing)) / 180
        else:
            sad = 1.1 - (0.2 * (360 - bearing)) / 180

        return sad

    def safe_encounter_distance(self):
        """
            ship safe encounter distance
        :return:
            2 * safe distance

        :unit: nautical mile(海里)
        """
        return 2 * self.safe_distance()

    def time_arrival_collision(self):
        """
            the time of arrival to the collision

        :unit: hour
        """
        if self.dcpa <= self.dla:
            tac = sqrt(pow(self.dla, 2) - pow(self.dcpa, 2)) / self.relative_speed()
        else:
            tac = (self.dcpa - self.dla) / self.relative_speed()
        return tac

    def time_arrival_destination(self):
        """
            the time of arrival to the destination

        :unit: hour
        """
        tad = sqrt(pow(8, 2) - pow(self.dcpa, 2)) / self.relative_speed()
        return tad

    def func_dcpa(self):
        if self.dcpa <= self.safe_distance():
            return 1
        elif self.safe_distance() < self.dcpa <= self.safe_encounter_distance():
            sub_safe = self.safe_encounter_distance() - self.safe_distance()
            plus_safe = self.safe_distance() + self.safe_encounter_distance()
            return 0.5 - 0.5 * sin((pi / sub_safe) * (self.dcpa - plus_safe / 2))
        else:
            return 0

    def func_distance(self):
        if self.distance() <= self.dla:
            return 1
        elif self.dla < self.distance() <= self.dla2:
            sub_dla = self.dla2 - self.dla
            plus_dla = self.dla + self.dla2
            return 0.5 - 0.5 * sin((pi / sub_dla) * (self.distance() - plus_dla / 2))
        else:
            return 0

    def func_tcpa(self):
        if self.tcpa <= self.time_arrival_collision():
            return 1
        elif self.time_arrival_collision() < self.tcpa <= self.time_arrival_destination():
            return abs((self.time_arrival_destination() - self.tcpa) / self.time_arrival_destination())
        else:
            return 0

    def func_bearing(self):
        theta = radians(self.bearing() - 19)
        return 0.5 * (cos(theta) + sqrt(440/289 + pow(cos(theta), 2))) - 5/17

    def func_speed_ratio(self):
        k = self.speed_ratio
        part = 2 / (k * sqrt(pow(k, 2) + 1 + 2 * k * sin(abs(self.delta_course))))
        return 1 / (1 + part)

    def collision_risk_index(self, weight_dcpa=0.4535, weight_tcpa=0.3604, weight_dis=0.1481, weight_bearing=0.0527, weight_speed_ratio=0.0393):
        """
            calculation of collision risk index
        :param weight_dcpa: the weight of dcpa
        :param weight_tcpa: the weight of tcpa
        :param weight_dis:  the weight of distance
        :param weight_bearing: the weight of bearing
        :param weight_speed_ratio: the weight of speed ratio
        :return: The CRI value between two encounter ship
        """
        dcpa, tcpa = weight_dcpa * self.func_dcpa(), weight_tcpa * self.func_tcpa()
        dis, bearing = weight_dis * self.func_distance(), weight_bearing * self.func_bearing()
        speed_ratio = weight_speed_ratio * self.func_speed_ratio()
        cri = dcpa + tcpa + dis + bearing + speed_ratio
        return cri

    def cpa(self):
        """
            The Method of Calculate DCPA and TCPA
        :return: (DCPA, TCPA)

        :unit: nautical mile(海里), hour(小时)
        """
        alpha = self.Ref_Cog - self.Tar_Cog
        if alpha > pi:
            alpha -= 2*pi
        elif alpha < -pi:
            alpha += 2*pi

        # 舷角Q
        x = pow(self.relative_speed(), 2) + pow(self.Ref_Sog, 2) - pow(self.Tar_Sog, 2)
        y = 2 * self.relative_speed() * self.Ref_Sog
        Q = np.arccos(x / y)

        # 两船之间的相对航向Relative_Course
        if alpha > 0:
            Relative_Course = self.Ref_Cog + Q
        else:
            Relative_Course = self.Ref_Cog - Q

        # 相对舷角Bearing
        Bearing = radians(self.bearing()) - Relative_Course

        # 计算Tar_Ship和Ref_Ship之间的DCPA和TCPA
        DCPA = abs(self.distance() * np.sin(Bearing))
        TCPA = abs(self.distance() * np.cos(Bearing) / self.relative_speed())

        return DCPA, TCPA
