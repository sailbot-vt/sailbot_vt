def test_lookup_table(apparent_wind_angle):
    wind_angles = [0,  45, 90, 135, 180, 225, 270, 315, 360]
    positions =   [80, 60, 40, 20,  10,  20,  40,  60,  80]
        
    # positions =   [80, 60, 40, 20,  10,  20,  40,  60,  80]
    # wind_angles = [0,  45, 90, 135, 180, 225, 270, 315, 360]
    # positions = [85, 37.5, 10, 10, 37.5, 85]
    # wind_angles = [0, 50, 100, 260, 310, 360]

    left = max(filter(lambda pos: pos <= float(apparent_wind_angle), wind_angles))
    right = min(filter(lambda pos: pos >= float(apparent_wind_angle), wind_angles))

    left = wind_angles.index(left)
    right = wind_angles.index(right)
    
    mast_angle = 0
    if (left == right):
        for i in range(len(positions)):
            if float(apparent_wind_angle) == wind_angles[i]:
                mast_angle = positions[i]
    else:
        slope = (positions[right] - positions[left])/(wind_angles[right] - wind_angles[left])
        mast_angle = slope * (float(apparent_wind_angle) - wind_angles[left]) + positions[left]
    
    return mast_angle

import pyproj
def test_thing(cur_lat_lon, des_lat_lon):
    cur_lat, cur_lon = cur_lat_lon
    des_lat, des_lon = des_lat_lon
    azimuth_heading, _, _ = pyproj.Geod(ellps='WGS84').inv(cur_lon, cur_lat, des_lon, des_lat)
    heading = (-azimuth_heading+ 90) % 360
    return heading

# print(test_lookup_table(340))
print(test_thing((0, 0), (0.00015, 0.00015)))