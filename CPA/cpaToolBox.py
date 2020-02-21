#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author: lph time:2019/5/18
import numpy as np
from math import *


class CPA:
    """
    1. The Distance between Target Ship and Reference Ship
        get_distance_hav(*args):
    args = (lat1, lon1, lat2, lon2)

    2. The Mid Point between Target Ship and Reference Ship
        min_point(*args)
    args = (lat1, lon1, lat2, lon2)

    3. The DCPA and TCPA between Target Ship and Reference Ship
        cpa(*args):
    args = (Tar_Ship, Ref_Ship)
    Tar_Ship: Target Vessel Object, Ref_Ship: Reference Vessel Object
    """
    EARTH_RADIUS = 6378.1
    n_mile = 1.852

    def __init__(self, Tar_Ship, Ref_Ship):
        self.Tar_Lat = Tar_Ship.LAT
        self.Tar_Lon = Tar_Ship.LON
        self.Ref_Lat = Ref_Ship.LAT
        self.Ref_Lon = Ref_Ship.LON
        self.Tar_Cog = Tar_Ship.COG
        self.Ref_Cog = Ref_Ship.COG

        self.Ref_Sog, self.Tar_Sog = Ref_Ship.SOG, Tar_Ship.SOG
        self.Ref_LENGTH, self.Ref_WIDTH = Ref_Ship.LENGTH, Ref_Ship.WIDTH
        self.Tar_LENGTH, self.Tar_WIDTH = Tar_Ship.LENGTH, Tar_Ship.WIDTH

        # 相对方位relative bearing
        self.relative_bearing = self.bearing() - self.Ref_Cog

    def haversine(self, theta):
        return pow(sin(theta / 2), 2)

    def distance(self):
        """
        Use Haversine formula To Calculate The Distance Between Tar_Ship and Ref_Ship On The Sphere
        return: The distance between Tar_Ship and Ref_Ship
        Unit: nm
        """
        diff_lon = np.fabs(self.Tar_Lon - self.Ref_Lon)
        diff_lat = np.fabs(self.Tar_Lat - self.Ref_Lat)
        h = self.haversine(diff_lat) + np.cos(self.Tar_Lat) * np.cos(self.Ref_Lat) * self.haversine(diff_lon)
        distance = 2 * CPA.EARTH_RADIUS * asin(np.sqrt(h))
        return distance / CPA.n_mile

    def mid_point(self):
        """
        mid_point(): Mid point between two latitude and longitude
        return: Mid Point(mid_lat, mid_lon)
        Unit: degree(°)
        """
        diff_lon = self.Ref_Lon - self.Tar_Lon
        Bx = cos(self.Ref_Lat) * cos(diff_lon)
        By = cos(self.Ref_Lat) * sin(diff_lon)
        x = sqrt((cos(self.Tar_Lat) + Bx) * (cos(self.Tar_Lat) + Bx) + pow(By, 2))
        y = sin(self.Tar_Lat) + sin(self.Ref_Lat)
        mid_lat = 180 / pi * np.arctan2(y, x)
        mid_lon = 180 / pi * (self.Tar_Lon + np.arctan2(By, cos(self.Tar_Lat) + Bx))
        return mid_lat, mid_lon

    def bearing(self):
        diff_lon = self.Ref_Lon - self.Tar_Lon

        x = sin(diff_lon) * cos(self.Ref_Lat)
        y = (cos(self.Tar_Lat) * sin(self.Ref_Lat)) - (sin(self.Tar_Lat) * cos(self.Ref_Lat) * cos(diff_lon))

        inital_bearing = np.arctan2(x, y)
        inital_bearing = np.rad2deg(inital_bearing)
        compass_bearing = (inital_bearing + 360) % 360
        return compass_bearing

    def relative_speed(self):
        """
            两船之间的相对速度
        :return: relative speed
        """
        alpha = self.Tar_Cog - self.Ref_Cog
        temp = 2 * self.Tar_Sog * self.Ref_Sog * cos(alpha)
        relative_speed = sqrt(pow(self.Tar_Sog, 2) + pow(self.Ref_Sog, 2) - temp)
        return relative_speed

    def cpa(self):
        """
            The Method of Calculate DCPA and TCPA
            return: (DCPA, TCPA)
        """
        alpha = self.Tar_Cog - self.Ref_Cog
        if alpha > pi:
            alpha -= 2*pi
        elif alpha < -pi:
            alpha += 2*pi

        # 舷角Q
        x = pow(self.relative_speed(), 2) + pow(self.Tar_Sog, 2) - pow(self.Ref_Sog, 2)
        y = 2 * self.relative_speed() * self.Tar_Sog
        Q = np.arccos(x / y)

        # 两船之间的相对航向Relative_Course
        if alpha > 0:
            Relative_Course = self.Tar_Cog + Q
        else:
            Relative_Course = self.Tar_Cog - Q

        # 相对舷角Bearing
        Bearing = radians(self.bearing()) - Relative_Course

        # 计算Tar_Ship和Ref_Ship之间的DCPA和TCPA
        DCPA = self.distance() * np.sin(Bearing)
        TCPA = self.distance() * np.cos(Bearing) / self.relative_speed()

        return DCPA, TCPA * 60
