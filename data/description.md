# Field Description Documentation

## order.csv

| Field Name                          | Description                                            |
|------------------------------------ |--------------------------------------------------------|
| order_id                            | Unique order ID                                        |
| region_id                           | Region ID                                              |
| pax_num                             | Number of passengers                                   |
| order_time                          | Order creation time                                    |
| available_pickup_station_list       | List of available pickup stations (1 station now)      |
| available_pickup_walkingtime_list   | Walking times to each pickup station (0 now)           |
| available_dropoff_station_list      | List of available drop-off stations (1 station now)    |
| available_dropoff_walkingtime_list  | Walking times to each drop-off station (0 now)         |
| status                              | Order status (0 new order; 1 accepted; 2 picked up; 3 finished; 4 canceled)                  |
| vehicle_id                          | Assigned vehicle ID                                    |
| pick_up_time                        | Actual pickup time                                     |
| drop_off_time                       | Actual drop-off time                                   |
| pick_up_early                       | Pickup earliness (1 if true)                           |
| drop_off_early                      | Drop-off earliness (1 if true)                         |

---

## segment.csv

| Field Name   | Description                |
|--------------|----------------------------|
| id           | Unique segment ID          |
| from_station | Starting station ID        |
| to_station   | Ending station ID          |
| seg_dist     | Segment distance (meters)  |
| seg_time     | Segment duration (seconds) |

---

## station.csv

| Field Name    | Description        |
|---------------|--------------------|
| station_id    | Station ID         |
| station_name  | Station name       |
| station_lon   | Station longitude  |
| station_lat   | Station latitude   |

---

## vehicle.csv

| Field Name        | Description               |
|-------------------|---------------------------|
| vehicle_id        | Vehicle ID                |
| vehicle_num       | Vehicle license plate     |
| vehicle_capacity  | Vehicle capacity (number of passengers) |
| vehicle_speed     | Vehicle speed (km/h) |

---

## veh_arrive_station_log.csv

| Field Name   | Description        |
|--------------|--------------------|
| id           | Log ID             |
| vehicle_id   | Vehicle ID         |
| station_id   | Station ID         |
| arrive_time  | Arrival time       |

---

## veh_location_log.csv

| Field Name   | Description        |
|--------------|--------------------|
| id           | Log ID             |
| vehicle_id   | Vehicle ID         |
| lat          | Latitude           |
| lon          | Longitude          |
| ts           | Timestamp          |

---


**Note:**  
All longitude and latitude coordinates in these tables use the **bd09ll** coordinate system (Baidu Map).  
If you need to convert them to the **WGS84** coordinate system (commonly used in GPS and international maps), you must do so manually.

Below is a Python script for converting bd09ll to WGS84:

```python
import math

x_pi = math.pi * 3000.0 / 180.0
a = 6378245.0 
ee = 0.00669342162296594323 

def out_of_china(lat, lon):
    return lon < 72.004 or lon > 137.8347 or lat < 0.8293 or lat > 55.8271

def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

# BD-09LL -> GCJ-02
def bd09_to_gcj02(bd_lon, bd_lat):
    x = bd_lon - 0.0065
    y = bd_lat - 0.006
    z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * x_pi)
    theta = math.atan2(y, x) - 0.000003 * math.cos(x * x_pi)
    gcj_lon = z * math.cos(theta)
    gcj_lat = z * math.sin(theta)
    return gcj_lon, gcj_lat

# GCJ-02 -> WGS84
def gcj02_to_wgs84(gcj_lon, gcj_lat):
    if out_of_china(gcj_lat, gcj_lon):
        return gcj_lon, gcj_lat

    dlat = transform_lat(gcj_lon - 105.0, gcj_lat - 35.0)
    dlon = transform_lon(gcj_lon - 105.0, gcj_lat - 35.0)
    rad_lat = gcj_lat / 180.0 * math.pi
    magic = math.sin(rad_lat)
    magic = 1 - ee * magic * magic
    sqrt_magic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrt_magic) * math.pi)
    dlon = (dlon * 180.0) / (a / sqrt_magic * math.cos(rad_lat) * math.pi)
    wgs_lat = gcj_lat - dlat
    wgs_lon = gcj_lon - dlon
    return wgs_lon, wgs_lat

# BD-09LL -> WGS84
def bd09_to_wgs84(bd_lon, bd_lat):
    gcj_lon, gcj_lat = bd09_to_gcj02(bd_lon, bd_lat)
    return gcj02_to_wgs84(gcj_lon, gcj_lat)

# Usage:
# wgs_lon, wgs_lat = bd09_to_wgs84(bd_lon, bd_lat)
```
