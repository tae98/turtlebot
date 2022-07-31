import cv2
import numpy as np
#constant values intialized
from src import constants


class Map:
    def __init__(self, clearance):
        #initialized values
        self.height = constants.map_size[0]
        self.width = constants.map_size[1]
        self.thresh = int(constants.robot_radius) + clearance
        self.scaling = constants.scaling_factor
        self.black = (0, 0, 0)
        #define widht and height rectangle infront of the line
        self.rectangle_width = int(self.scaling * 1)
        self.rectangle_height  = int(self.scaling*0.08)
        # define coordante for obstacle infront of the line 
        self.rectangleCordinates = np.array(np.array([(2.08000, 2.71000)]
                                      ,)*self.scaling, dtype=np.int32)
        # define values for the cylinder obstacle
        self.cylinderRadius = int(self.scaling * 0.15)
        # coordinates of the cylinder obstacle
        self.cylinderPos = np.array(
                                np.array([(0.413804,2.7),
                                          (1.113804, 2.7),
                                          (2.899000, 2.7),
                                          (3.340000, 2.7)],
                                )*self.scaling,
                                dtype=np.int32)
        #parking space corner coordinates
        self.parkingCoordinate = np.array(
                                np.array([(1.693134, 0.421303),
                                          (2.193452, 0.998265),
                                          (1.435601, 1.657695),
                                          (1.320525, 1.543221),
                                          (1.963959,0.983919),
                                          (1.676326,0.634233),
                                          (1.037460,1.194636),
                                          (0.947392,1.084816)],
                                )*self.scaling,
                                dtype=np.int32)
        # empty world initialization
        self.mapImage = self.drawObstacles()
        # get the drawn map with obstacle
        self.mapObstacle = self.erodeImage()
        

    def drawCylinder(self, img, thresh=0):
        for center in self.cylinderPos:
            # create cylinder obstacle
            cv2.circle(img, (center[0], center[1]), self.cylinderRadius + thresh, self.black, -1)
    
    def drawParking(self, img, thresh=0):
        cv2.drawContours(img,[self.parkingCoordinate],-1,self.black,-1)
        #thresh times 2 because it needs to have same thickness on both side from the middle
        cv2.drawContours(img,[self.parkingCoordinate],-1,self.black, thresh*2)
    
            

    def drawRectangle(self, img, thresh=0):
         #draw rectangle infront of the line
        for corner in self.rectangleCordinates:
            top_corner = ((corner[0] - thresh)-self.rectangle_width//2), (corner[1] - self.rectangle_height//2 -thresh)
            bottom_corner = (corner[0] + self.rectangle_width//2 + thresh), (corner[1] + self.rectangle_height//2 + thresh)
            cv2.rectangle(img, top_corner, bottom_corner, self.black, -1)

    def erodeImage(self):
        #get the map image in scale with robots radius and clearnace value
        eroded_img = self.mapImage.copy()
        self.drawRectangle(eroded_img, self.thresh)
        self.drawCylinder(eroded_img, self.thresh)
        self.drawParking(eroded_img, self.thresh)
        return eroded_img

    def drawObstacles(self):
        #draw map using cv2
        self.mapImage = cv2.imread('world Image/map.png')
        if True or self.mapImage is None:
            self.mapImage = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # backgound color of the map
            self.mapImage.fill(255)
            # obstacle drawing
            self.drawCylinder(self.mapImage)
            self.drawRectangle(self.mapImage)
            self.drawParking(self.mapImage)
            #saving png file to respective directory
            cv2.imwrite('world Image/map.png', self.mapImage)

        return self.mapImage

    def getMapPosition(self, point, theta=0):
        #converting in term of map with map orientation and coordinate value
        x, y = point
        theta = theta // constants.angular_step
        # Scaling coordiantes to fit map
        x, y = int(self.scaling * x),  self.height-int(self.scaling * y)
        return list((y, x, theta))
