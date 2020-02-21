#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author: lph time:2019/5/13
import numpy as np

from math import pi, sqrt, cos, sin, log10 as lg, log as ln
from shapely.geometry import Polygon
from Area.Grid import Grid
"""

    MMSI, NAME, TIME, LON, LAT, COG, SOG, LENGTH, WIDTH

"""

class Vessel(Grid):

    EARTH_RADIUS = 6378137

    def __init__(self, area_id, args, *, gridlon_, gridlat_, grid_delta):
        self.MMSI = args[0]
        self.NAME = args[1]
        self.TIME = args[2]
        self.LON = np.radians(float(args[3]))
        self.LAT = np.radians(float(args[4]))
        self.COG = np.radians(float(args[5]))
        self.SOG = float(args[6])
        self.LENGTH = float(args[7])
        self.WIDTH = float(args[8])
        Grid.__init__(
            self,
            area_id,
            gridlon_=gridlon_,
            gridlat_=gridlat_,
            grid_delta=grid_delta
        )

    def fujiDomain(self):
        """
            fuji椭圆船舶领域
        :return:
            Polygon(fujiDomain)
        """
        lon, lat = self.LON, self.LAT
        length = (self.LENGTH / Vessel.EARTH_RADIUS) * (180 / np.pi)

        # 椭圆的半长轴semi_major, 椭圆的半短轴semi_minor
        semi_major, semi_minor = 3 * length, 7 * length

        cog = self.COG
        theta = np.linspace(0, 2 * np.pi, 100)

        # ellipse shape is (2, 100)
        ellipse = np.array([semi_major * np.cos(theta), semi_minor * np.sin(theta)])
        # two dimension Rotation matrix
        rot = np.array([[np.cos(cog), np.sin(cog)], [-np.sin(cog), np.cos(cog)]])

        for i in range(ellipse.shape[1]):
            ellipse[:, i] = np.dot(rot, ellipse[:, i])

        poly = np.column_stack([np.degrees(lon) + ellipse[0, :], np.degrees(lat) + ellipse[1, :]])
        # poly = [np.degrees(lon) + ellipse[0, :], np.degrees(lat) + ellipse[1, :]]
        return Polygon(poly)

    def fqsd(self, k_shape=2, r_static=0.5):
        """
            Fuzzy Quaternion Ship Domain (模糊四元数船舶领域)
        :return: Polygon(fqsd)
        """
        r0 = 0.5
        if self.SOG != 0.0:
            k_ad = 10 ** (0.359 * lg(self.SOG) + 0.0952)
            k_dt = 10 ** (0.541 * lg(self.SOG) - 0.0795)
        else:
            k_ad, k_dt = 0.0, 0.0

        R_fore = (1 + 1.34 * sqrt((k_ad) ** 2 + (k_dt / 2) ** 2)) * self.LENGTH
        R_aft = (1 + 0.67 * sqrt((k_ad) ** 2 + (k_dt / 2) ** 2)) * self.LENGTH
        R_starb = (0.2 + k_dt) * self.LENGTH
        R_port = (0.2 + 0.75 * k_dt) * self.LENGTH

        R = np.array((R_fore, R_aft, R_starb, R_port))
        r_fuzzy = ((ln(1 / r_static)) / (ln(1 / r0))) ** (1 / k_shape)
        R_fuzzy = r_fuzzy * R

        x_max = R_fuzzy[0]
        x_min = -R_fuzzy[1]
        x_serises_plus = np.linspace(0, x_max, 80)
        x_serises_minus = np.linspace(x_min, -0.01, 80)

        y_1 = R_fuzzy[2] * pow((1 - (x_serises_plus / R_fuzzy[0]) ** k_shape), 1 / k_shape)
        y_4 = R_fuzzy[3] * -pow((1 - (x_serises_plus / R_fuzzy[0]) ** k_shape), 1 / k_shape)
        y_2 = R_fuzzy[2] * pow((1 - (-x_serises_minus / R_fuzzy[1]) ** k_shape), 1 / k_shape)
        y_3 = R_fuzzy[3] * -pow((1 - (-x_serises_minus / R_fuzzy[1]) ** k_shape), 1 / k_shape)

        # stack the column
        curve1 = np.column_stack((x_serises_plus, y_1))
        curve2 = np.column_stack((x_serises_minus, y_2))
        curve3 = np.column_stack((x_serises_minus, y_3))
        curve4 = np.column_stack((x_serises_plus, y_4))
        # Primacy connection
        curve4 = np.flipud(curve4)
        curve3 = np.flipud(curve3)
        # cat the data series to one curve.
        curves = np.row_stack((curve1, curve4, curve3, curve2)).T

        t_rot = self.COG - pi / 2
        R_rot = np.array([[cos(t_rot), sin(t_rot)], [-sin(t_rot), cos(t_rot)]])
        for i in range(curves.shape[1]):
            curves[:, i] = np.dot(R_rot, curves[:, i])
        # Translation to position
        curves = curves / 111000
        curves[0, :] += np.degrees(self.LON)
        curves[1, :] += np.degrees(self.LAT)

        curves = np.column_stack(curves)
        return Polygon(curves)