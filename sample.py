from pandas import DataFrame, read_csv
import numpy as np
import matplotlib.pyplot as plt
import math

# import only specific columns required from the csv file
colms = ['recordingId', 'trackId', 'frame', 'trackLifetime', 'xCenter', 'yCenter',
                'heading', 'width', 'length', 'xVelocity', 'yVelocity', 'xAcceleration',
                'yAcceleration', 'lonVelocity', 'latVelocity', 'lonAcceleration', 'latAcceleration']
# file path
file_path = r'E:\UniversityOfWindsor\Thesis\RoundD-Dataset\data\00_tracks.csv'
# import data from the csv file
data_array = read_csv(file_path, sep=',').values
# print(data_array)
df = DataFrame(data_array, columns=colms)
# print('Dataframe for All table!')
# print(df)
df['detectionList'] = ''
veh_id = 25

veh_frames = np.unique(df.loc[df['trackId'] == veh_id]['frame']).tolist()
veh_frames = [int(x) for x in veh_frames]
# print(veh_frames)
# Only vehicles which are present in frame, veh_frames will be available in 
dataframe = df.loc[np.in1d(df['frame'], veh_frames)]
# print(df)
# print(dataframe)
ego_veh_df = dataframe.loc[lambda df: df['trackId'] == veh_id]
print(ego_veh_df)
# print(df)
# df.to_csv('E:\\UniversityOfWindsor\\Thesis\\Code\\test_matplotlib\\from_code.csv')
# --------------------------------------------------


# -----Options-----
# WINDOW_SIZE = (1200, 700) 
plt.figure(figsize=(12,7)) # Width x Height in pixels
NUM_RAYS = 120 # Must be between 1 and 360
RAY_LENGTH = 150 # Length of each Ray
RAY_SPREAD = 120 # Value of Ray Spread in degree
RAY_SPREAD_INIT = int((180 - RAY_SPREAD)/2)
RAY_SPREAD_END = int((180 + RAY_SPREAD)/2)
# NO_OF_OBJECTS = 2 # Number of Vehicles
#------------------

mx,my = tuple((133.5737,-137.1928))
lastClosestPoint = (0, 0)
rays = []
cars = []
walls = []

class Ray:
    def __init__(self, x, y, angle, length):
        self.x = x
        self.y = y
        self.length = length
        self.dir = (length*math.cos(angle), length*math.sin(angle))

    def update(self, mx, my):
        self.x = mx
        self.y = my

    def checkCollision(self, wall):
        x1 = wall.start_pos[0]
        y1 = wall.start_pos[1]
        x2 = wall.end_pos[0]
        y2 = wall.end_pos[1]

        x3 = self.x
        y3 = self.y
        x4 = self.x + self.dir[0]
        y4 = self.y + self.dir[1]
    
        # Using line-line intersection formula to get intersection point of ray and wall
        # Where (x1, y1), (x2, y2) are the ray pos and (x3, y3), (x4, y4) are the wall pos
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
        if denominator == 0:
            return None
        
        t = numerator / denominator
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

        if 1 > t > 0 and 1> u > 0: # if two line segments
        # if 1 > t > 0 and u > 0:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            collidePos = [x, y]
            return collidePos

for i in range(RAY_SPREAD_INIT, RAY_SPREAD_END, int(RAY_SPREAD/NUM_RAYS)):
    rays.append(Ray(mx, my, math.radians(i), RAY_LENGTH))

def drawRays(df, veh_frames, rays, walls, color = 'white'):
    global lastClosestPoint
    for veh_frame in veh_frames:
        vehListPerFrame = []
        # Obtain the number of objects for each frame
        print(veh_frame)
        vehListPerFrame = np.unique(df.loc[df['frame'] == veh_frame]['trackId']).tolist()
        vehListPerFrame = [int(x) for x in vehListPerFrame]
        vehListPerFrame.remove(veh_id)
        # print(vehListPerFrame)
        noOfVeh =  len(vehListPerFrame)
        # Print no of Vehicles for each frame
        print(noOfVeh)
        # Each vehicles df is obtained here
        vehListPerFramedf = df.loc[df['frame'] == veh_frame]
        vehListPerFramedf = vehListPerFramedf.loc[vehListPerFramedf['trackId'] != veh_id]
        # print(vehListPerFramedf)
        cars = modifyVehValues(vehListPerFramedf, noOfVeh)
        for ray in rays:
            ray.update(ego_veh_df.iloc[veh_frame][4],ego_veh_df.iloc[veh_frame][5])
            closest = RAY_LENGTH
            closestPoint = None
            for car in cars:
                walls = getVehicleWalls(car)
                for wall in walls:
                    wall.draw()
                    intersectPoint = ray.checkCollision(wall)
                    if intersectPoint is not None:
                        # Get distance between ray source and intersect point
                        ray_dx = ray.x - intersectPoint[0]
                        ray_dy = ray.y - intersectPoint[1]
                        # If the intersect point is closer than the previous closest intersect point, it becomes the closest intersect point
                        distance = math.sqrt(ray_dx**2 + ray_dy**2)
                        if (distance < closest):
                            closest = distance
                            closestPoint = intersectPoint
                # print(closest)
            
            if closestPoint is not None:
                plt.plot([ray.x,closestPoint[0]],[ray.y,closestPoint[1]],'k-')
                # pygame.draw.line(display, color, (ray.x, ray.y), closestPoint)
                # print(closestPoint)
                # if SOLID_RAYS:
                #     pygame.draw.polygon(display, color, [(mx, my), closestPoint, lastClosestPoint])
                lastClosestPoint = closestPoint
            else:
                plt.plot([ray.x, ray.x+ray.dir[0]], [ray.y, ray.y+ray.dir[1]],'k-')
                # pygame.draw.line(display, color, (ray.x, ray.y), (ray.x+ray.dir[0], ray.y+ray.dir[1]))
        # print(closestPoint)
        # print(car.endpoints)
        plt.pause(0.1)
        plt.cla()

class Wall:
    def __init__(self, start_pos, end_pos, color = 'white'):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.color = color
        self.slope_x = end_pos[0] - start_pos[0]
        self.slope_y = end_pos[1] - start_pos[1]
        if self.slope_x == 0:
            self.slope = 0
        else:
            self.slope = self.slope_y / self.slope_x
        self.length = math.sqrt(self.slope_x**2 + self.slope_y**2)

    def draw(self):
        plt.plot([self.start_pos[0],self.end_pos[0]],[self.start_pos[1],self.end_pos[1]],'k-')
        # pygame.draw.line(display, self.color, self.start_pos, self.end_pos, 3)

class Car:
    def __init__(self, x, y, h, w, l):
        self.x_center = x
        self.y_center = y
        self.heading = h
        self.width = w
        self.length = l
        ax = self.x_center - self.width/2
        ay = self.y_center - self.length/2
        bx = self.x_center + self.width/2
        by = self.y_center - self.length/2
        cx = self.x_center + self.width/2
        cy = self.y_center + self.length/2
        dx = self.x_center - self.width/2
        dy = self.y_center + self.length/2
        self.endpoints = [ax, ay, bx, by, cx, cy, dx, dy]

def modifyVehValues(df, nOfV):
    cars.clear()
    for i in range(nOfV):
        cars.append(Car(df.iloc[i][4], df.iloc[i][5], df.iloc[i][6], df.iloc[i][7], df.iloc[i][8]))
    return cars 

def getVehicleWalls(car):
    walls.clear()
    # for car in cars:
    start_x = car.endpoints[0]
    start_y = car.endpoints[1]
    end_x = car.endpoints[2]
    end_y = car.endpoints[3]
    walls.append(Wall((start_x, start_y), (end_x, end_y)))
    start_x = car.endpoints[2]
    start_y = car.endpoints[3]
    end_x = car.endpoints[4]
    end_y = car.endpoints[5]
    walls.append(Wall((start_x, start_y), (end_x, end_y)))
    start_x = car.endpoints[4]
    start_y = car.endpoints[5]
    end_x = car.endpoints[6]
    end_y = car.endpoints[7]
    walls.append(Wall((start_x, start_y), (end_x, end_y)))
    start_x = car.endpoints[6]
    start_y = car.endpoints[7]
    end_x = car.endpoints[0]
    end_y = car.endpoints[1]
    walls.append(Wall((start_x, start_y), (end_x, end_y)))
    return walls

def draw():
    drawRays(dataframe, veh_frames, [ray for ray in rays], [wall for wall in walls])

draw()