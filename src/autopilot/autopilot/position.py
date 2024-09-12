import navpy
import utm
import numpy as np
from pygeodesy.ellipsoidalKarney import LatLon
import pygeodesy

class Position:
    # TODO better documentation
    """
    A position that describes a point on the earth which is stored internally as its longitude and latitude.
    Whenever a function such as get_local_coordinates is called, this class has to perform conversions which may or may not be expensive to compute.
    This can be fixed by caching results from conversions (so maybe TODO). 
    Really this caching doesn't matter anymore as we have pretty much scrapped utm so conversions can't really be cached in the same way
    
    """
    
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
    
    def set_longitude_latitude(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
    
    def set_utm(self, easting, northing, zone, hemisphere):
        """
        If hemisphere is "N" then we are talking about the northern hemisphere and if the hemisphere is "S" we are talking about the southern hemisphere.  
        """
        if hemisphere != "N" and hemisphere != "S": raise Exception("Incorrect Arguments Passed for set_utm")
        
        is_northern = True if "N" else False  
        self.latitude, self.longitude = utm.to_latlon(easting, northing, zone, northern=is_northern)
    
    
    def set_local_coordinates(self, local_x, local_y, reference_longitude, reference_latitude):
        self.latitude, self.longitude, _ = navpy.ned2lla([local_y, local_x, 0], reference_latitude, reference_longitude, 0)
    
    def get_utm(self) -> np.ndarray:
        latlong = LatLon(self.latitude, -1 * self.longitude)
        utm_coord = pygeodesy.utm.toUtm8(latlong)
        return utm_coord.easting, utm_coord.northing
    
    def get_lon_lat(self) -> np.ndarray:
        return np.array([self.longitude, self.latitude])
    
    def get_lat_lon(self) -> np.ndarray:
        return np.array([self.latitude, self.longitude])
    
    def get_local_coordinates(self, reference_lon_lat: np.ndarray) -> np.ndarray:
        reference_latitude = reference_lon_lat[1]
        reference_longitude = reference_lon_lat[0]
        local_y, local_x, _ = navpy.lla2ned(self.latitude, self.longitude, 0, reference_latitude, reference_longitude, 0)
        return np.array([local_x, local_y])
