#!/usr/bin/env python
# -*- coding:utf-8 -*-
# author: lph time:2019/5/5
import csv
import time

import pandas as pd
from pymongo import MongoClient
from datetime import datetime
from ShipDataQuery.ComplexEncoder import ComplexEncoder

"""
    研究区域:
            (Longitude, Latitude)
        Left Corner:    Right Corner:
        (120°, 30°)     (125°, 35°)

    网格精度:
            0.5° * 0.5°
    
    Polygon:
            [
                [
                    [120, 30], 
                    [125, 30],
                    [125, 35],
                    [120, 35],
                    [120, 30]
                ]
            ]
    
    导出形式:
            ChinaCoastalData.json
                
    将位于上述网格区域内的所有船舶经、纬度从mongodb数据库中筛选出来
"""

# 使用mongodb登录
client = MongoClient('localhost', 27017)
# 数据库database: ais_motor
db = client.ais6
# 集合collection: test_motor
collection = db.tracks1_

# 开始时间
start = time.time()

# 使用aggregate()方法
results = collection.aggregate([{
    "$match": {
        "update_time": {
            "$gte": datetime(2016, 1, 1),
            "$lte": datetime(2016, 1, 31)
        },
        "location": {"$geoWithin": {"$geometry": {
            "type": "Polygon",
            "coordinates": [[
                [122.1, 30.6],
                [122.6, 30.6],
                [122.6, 31.2],
                [122.1, 31.2],
                [122.1, 30.6]
            ]]
        }}}
    }
}])

# 使用.csv格式存储Polygon中的船舶AIS数据
with open('../DataProcess/2016-Meta-Data/ais6-2016-01.csv', 'w') as f_write:
    datas = csv.writer(f_write)
    Header = ["MMSI", "NAME", "TIME", "LON", "LAT", "COG", "SOG", "LENGTH", "WIDTH"]
    datas.writerow(Header)
    for result in results:
        datas.writerow([
            result["mmsi"],
            result["name"],
            result["update_time"],
            result["location"]["coordinates"][0],
            result["location"]["coordinates"][1],
            result["course"],
            result["speed"],
            result["length"],
            result["width"]
        ])

# 结束时间
end = time.time()
# 总用时
total = end - start

print("-------------------------------------")
print("*********The Work is done!***********")
print("The total Time is {}!".format(total))
# print("The query has {} documents!".format(len(ship_info)))
print("-------------------------------------")
