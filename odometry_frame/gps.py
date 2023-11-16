from ublox_gps import UbloxGps
import serial
import math
import numpy



class GPS:
    """
    How the gps modules are set up (right now)
            1
        /|
        /   | 0.5m (vDist)
    /      |      
    2-------.-------3
            1m 
        (hDist)

    The variables vDist and hDist can be changed to represent different configurations
    The period is the point that the program is calculating
    """
    def __init__(self, gpsOnePortName=None, gpsTwoPortName=None, gpsThreePortName=None, vDist = 0.5, hDist = 0.5):
        self.vDist = vDist
        self.hDist = hDist
        self.gpsOnePort = serial.Serial(gpsOnePortName) if gpsOnePortName == None else None
        self.gpsTwoPort = serial.Serial(gpsTwoPortName) if gpsTwoPortName == None else None
        self.gpsThreePort = serial.Serial(gpsThreePortName) if gpsThreePortName == None else None
    
        
        assert self.gpsOnePort != None and self.gpsTwoPort != None and self.gpsThreePort != None

        try:
            self.gpsOne = UbloxGps(self.gpsOnePort)
            self.gpsTwo = UbloxGps(self.gpsTwoPort)
            self.gpsThree = UbloxGps(self.gpsThreePort)
        
        except Exception as e: # Try to close all the gps modules, print any errors
            try: self.gpsOne.close()
            except Exception as n: print(n)
            try: self.gpsTwo.close()
            except Exception as n: print(n)
            try: self.gpsThree.close()
            except Exception as n: print(n)
            raise e
    
    def pereodic(self):
        pointA = self.gpsOne.geo_coords()
        pointB = self.gpsTwo.geo_coords()
        pointC = self.gpsThree.geo_coords()
        robot_coords =  self._trilaterate(pointA, pointB, pointC)
        robot_heading = self._calculate_heading(robot_coords, pointA)

        return robot_coords, robot_heading


    def _trilaterate(self, pointA, pointB, pointC):
        # https://gis.stackexchange.com/questions/66/trilateration-using-3-latitude-longitude-points-and-3-distances

        #assuming elevation = 0 
        earthR = 6371
        distA = self.vDist
        distB = self.hDist/2
        distC = self.hDist/2

        #using authalic sphere
        #if using an ellipsoid this step is slightly different
        #Convert geodetic Lat/Long to ECEF xyz
        #   1. Convert Lat/Long to radians
        #   2. Convert Lat/Long(radians) to ECEF
        xA = earthR *(math.cos(math.radians(pointA.lat)) * math.cos(math.radians(pointA.lon)))
        yA = earthR *(math.cos(math.radians(pointA.lat)) * math.sin(math.radians(pointA.lon)))
        zA = earthR *(math.sin(math.radians(pointA.lat)))

        xB = earthR *(math.cos(math.radians(pointB.lat)) * math.cos(math.radians(pointB.lon)))
        yB = earthR *(math.cos(math.radians(pointB.lat)) * math.sin(math.radians(pointB.lon)))
        zB = earthR *(math.sin(math.radians(pointB.lat)))

        xC = earthR *(math.cos(math.radians(pointC.lat)) * math.cos(math.radians(pointC.lon)))
        yC = earthR *(math.cos(math.radians(pointC.lat)) * math.sin(math.radians(pointC.lon)))
        zC = earthR *(math.sin(math.radians(pointC.lat)))

        P1 = numpy.array([xA, yA, zA])
        P2 = numpy.array([xB, yB, zB])
        P3 = numpy.array([xC, yC, zC])

        #from wikipedia
        #transform to get circle 1 at origin
        #transform to get circle 2 on x axis
        ex = (P2 - P1)/(numpy.linalg.norm(P2 - P1))
        i = numpy.dot(ex, P3 - P1)
        ey = (P3 - P1 - i*ex)/(numpy.linalg.norm(P3 - P1 - i*ex))
        ez = numpy.cross(ex,ey)
        d = numpy.linalg.norm(P2 - P1)
        j = numpy.dot(ey, P3 - P1)

        #from wikipedia
        #plug and chug using above values
        x = (pow(distA,2) - pow(distB,2) + pow(d,2))/(2*d)
        y = ((pow(distA,2) - pow(distC,2) + pow(i,2) + pow(j,2))/(2*j)) - ((i/j)*x)

        # only one case shown here
        z = numpy.sqrt(pow(distA,2) - pow(x,2) - pow(y,2))

        #triPt is an array with ECEF x,y,z of trilateration point
        triPt = P1 + x*ex + y*ey + z*ez

        #convert back to lat/long from ECEF
        #convert to degrees
        lat = math.degrees(math.asin(triPt[2] / earthR))
        lon = math.degrees(math.atan2(triPt[1],triPt[0]))

        return lat, lon

    def _calculate_heading(self, midpoint, front):
        # Calculate the distance between two points (haversine algorithm)
    
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [midpoint.lat, midpoint.lon, front.lat, front.lon])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        # Radius of the Earth in kilometers (mean value)
        R = 6371.0
        
        # Calculate the distance
        distance = R * c

        delta_lon = front.lon - midpoint.lon
        y = math.sin(delta_lon) * math.cos(front.lat)
        x = math.cos(midpoint.lat) * math.sin(front.lat) - math.sin(midpoint.lat) * math.cos(front.lat) * math.cos(delta_lon)
        bearing = math.atan2(y, x)
        
        # Convert the bearing from radians to degrees
        bearing = math.degrees(bearing)
        
        # Adjust the heading to be in the range [0, 360)
        heading = (bearing + 360) % 360
        
        return heading
        
