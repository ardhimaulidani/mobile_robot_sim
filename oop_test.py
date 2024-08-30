import sys
import yaml
import pygame
import pygame.gfxdraw

import numpy as np
from os.path import dirname, join

from include.Utility.utility import *

# Button Class
class Button:
    def __init__(self, icon_file, x, y, width, height):
        self.rect = pygame.Rect(x, y, width, height)
        self.icon = pygame.image.load(icon_file)
        self.icon = pygame.transform.scale(self.icon, (width, height))
        self.rect = self.icon.get_rect()
        self.rect.center = (x,y)

    def draw(self, screen):
        screen.blit(self.icon, self.rect)

    def is_hovered(self, pos):
        return self.rect.collidepoint(pos)

# Slider Class
class Slider:
    def __init__(self, x, y, width, height, min_value=0, max_value=100):
        self.rect = pygame.Rect(x, y, width, height)
        self.min_value = min_value
        self.max_value = max_value
        self.value = min_value
        self.dragging = False

    def draw(self, screen):
        pygame.draw.rect(screen, GRAY, self.rect)
        handle_x = self.rect.x + (self.value / self.max_value) * self.rect.width
        pygame.draw.circle(screen, BLUE, (int(handle_x), self.rect.centery), self.rect.height // 2)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            mouse_x = event.pos[0]
            self.value = max(self.min_value, min(self.max_value, 
                                (mouse_x - self.rect.x) / self.rect.width * self.max_value))

# Robot Class
class RobotOmni:
    def __init__(self, robot_config):
        # Open YAML Config File
        current_dir = dirname(__file__)
        file_path = join(current_dir, robot_config)

        with open(file_path, 'r') as file:
            robot = yaml.safe_load(file)
            
            self.widthRobot  = robot["body"]["width"]
            self.heightRobot = robot["body"]["height"]
            self.lengthRobot = robot["body"]["length"]
            self.radiusWheel = np.float32(robot["wheel"]["rw"])
            self.widthWheel = np.float32(robot["wheel"]["width"])
            
            self.dn = np.array([val for (key,val) in robot["axle_length"].items()], dtype='f8')
            self.bn = np.array([np.radians(val) for (key,val) in robot["axle_angle"].items()], dtype='f8')
            self.yn = np.array([np.radians(val) for (key,val) in robot["wheel_angle"].items()], dtype='f8')

    def draw(self, screen, pose):
        # Robot Body Point Reference
        _pointRef = np.array([[pose[0]-self.widthRobot*0.5, pose[1]-self.lengthRobot*0.5],
                              [pose[0]+self.widthRobot*0.5, pose[1]-self.lengthRobot*0.5],
                              [pose[0]+self.widthRobot*0.5, pose[1]+self.lengthRobot*0.5],
                              [pose[0]-self.widthRobot*0.5, pose[1]+self.lengthRobot*0.5]])

        for i in range(len(self.dn)):
            _wheelAngleRef = np.array([[pose[0] + (self.dn[i] * np.cos(self.bn[i]) - self.radiusWheel * np.sin(self.yn[i] + self.bn[i])), pose[1] + (self.dn[i] * -np.sin(self.bn[i]) - self.radiusWheel * np.cos(self.yn[i] + self.bn[i]))],
                                       [pose[0] + (self.dn[i] * np.cos(self.bn[i]) + self.radiusWheel * np.sin(self.yn[i] + self.bn[i])), pose[1] + (self.dn[i] * -np.sin(self.bn[i]) + self.radiusWheel * np.cos(self.yn[i] + self.bn[i]))]])
    
            _wheelRef = np.array([[_wheelAngleRef[0,0] + self.widthWheel * 0.5 * -(_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*self.radiusWheel), _wheelAngleRef[0,1] + self.widthWheel * 0.5 * (_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*self.radiusWheel)],
                                  [_wheelAngleRef[0,0] + self.widthWheel * 0.5 * (_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*self.radiusWheel), _wheelAngleRef[0,1] + self.widthWheel * 0.5 * -(_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*self.radiusWheel)],
                                  [_wheelAngleRef[1,0] + self.widthWheel * 0.5 * (_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*self.radiusWheel), _wheelAngleRef[1,1] + self.widthWheel * 0.5 * -(_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*self.radiusWheel)],
                                  [_wheelAngleRef[1,0] + self.widthWheel * 0.5 * -(_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*self.radiusWheel), _wheelAngleRef[1,1] + self.widthWheel * 0.5 * (_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*self.radiusWheel)]])
            # Draw Wheel Lines
            pygame.gfxdraw.filled_polygon(screen, Utility.Rt_2DCenterRef(_wheelRef, pose), RED)
            pygame.gfxdraw.aapolygon(screen, Utility.Rt_2DCenterRef(_wheelRef, pose), RED)

        # Draw Robot Body Lines
        pygame.draw.aalines(screen, BLACK, True, Utility.Rt_2DCenterRef(_pointRef, pose), 3)

# Map Screen Class
class MapScreen:
    def __init__(self, bgScreen, map=None):
        # Set up the map display
        self.widthMapScreen = bgScreen[0]
        self.heightMapScreen = bgScreen[1]
        self._mapScreen = pygame.display.set_mode((self.widthMapScreen, self.heightMapScreen), pygame.SRCALPHA)
        
        # Create a mask surface with transparency
        self._maskScreen = pygame.Surface((self.widthMapScreen, self.heightMapScreen), pygame.SRCALPHA)
        self._maskScreen.fill(TRANSPARENT)
        # Draw Grid
        if map is not None:
            grid = [[0 for _ in range(map.width)] for _ in range(map.height)]
        
        # Define Animation Timeframe
        self.ticks = 0
        
        # Create Object
        self.mobileRobot = RobotOmni("config\\drive\\fmlx_rover.yaml")
        
        # Get Mission 
        # TO:DO Only parse when there are input from button
        self.bodyPose, self.bodyVel = Utility.parse_csv("mission\\test.csv")
        
    def draw(self, map=None):
        # Define Map Screen Bg Color
        self._mapScreen.fill(WHITE)
        
        # Get Map Border Point
        _ratio = 0.95
        self._pointRef = np.array([[(1-_ratio) * self.widthMapScreen, (1-_ratio) * self.heightMapScreen],
                                   [(1-_ratio) * self.widthMapScreen, (_ratio) * self.heightMapScreen],
                                   [(_ratio) * self.widthMapScreen, (_ratio) * self.heightMapScreen],
                                   [(_ratio) * self.widthMapScreen, (1-_ratio) * self.heightMapScreen]])
        # Draw Map Border
        pygame.draw.aalines(self._mapScreen, BLACK, 1, self._pointRef)
    
    def run(self):
        # Draw Mobile Robot
        self.mobileRobot.draw(self._mapScreen, self.bodyPose[0])
        
        # Draw Mask
        pygame.draw.polygon(self._maskScreen, WHITE, self._pointRef)
        
        # # Clip the drawing by blitting the mask onto the drawing surface
        self._mapScreen.blit(self._maskScreen, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)

    def update(self, speed):
        # Update Trajectory
        if len(self.bodyPose) > 1:
            self.bodyPose = self.bodyPose[1:]
        
        # Update Ticks
        self.ticks += 1*speed
        pass

# Main GUI Application Class
class Visualization:
    def __init__(self, viz_config):
        # Initialize Pygame
        pygame.init()
        
        # Open YAML Config File
        current_dir = dirname(__file__)
        file_path = join(current_dir, viz_config)
        
        # Set Constant
        with open(file_path, 'r') as file:
            viz = yaml.safe_load(file)
            widthScreen  = viz["screen"]["width"]
            heightScreen = viz["screen"]["height"]
            
            WheelViewScale = np.float32(viz["wheel_view"]["scale"])/100
            if WheelViewScale > 3 : WheelViewScale = 3
            WorldViewScale = np.float32(viz["world_view"]["scale"])/100
            if WorldViewScale > 3 : WorldViewScale = 3
            
        # Constants Color
        global BLACK,WHITE,GRAY,ORANGE,TEAL,RED,TRANSPARENT
        BLACK  = (0, 0, 0)
        WHITE  = (255,255,255)
        GRAY   = (150, 150, 150)
        ORANGE = (255,165,0)
        TEAL   = (42, 157, 244)
        RED    = (196, 30, 58)
        TRANSPARENT = (0, 0, 0, 0)
        
        # Set up the background display
        self._bgScreen = pygame.display.set_mode((widthScreen, heightScreen))
        
        # Set up the map display
        self._mapScreen = MapScreen((widthScreen, heightScreen))
        
        # Set up the wheel frame display
        # self._wheelScreen = pygame.display.set_mode((widthScreen, heightScreen))
        
        # Set Application Caption 
        pygame.display.set_caption('Mobile Robot Simulator')
        
        # Animation Loop
        self.stateAnimation = True
        self.clock = pygame.time.Clock()
        
        self.font = pygame.font.Font(None, 36)
        
        # Create Object
        self.plusButton = Button("icons\\plus.png", 500, 500, 32, 32)
        self.minusButton = Button("icons\\minus.png", 500, 550, 32, 32)
        self.closeButton = Button("icons\\close.png", 500, 600, 32, 32)
        self.moreButton = Button("icons\\more.png", 500, 650, 32, 32)
        self.slider = Slider(300, 100, 200, 20)

    
    def run(self):
        while self.stateAnimation:
            self.handle_events()
            self.bgDraw()
            
            # Update State
            self.update()
            
            # Draw All Buttons
            self.plusButton.draw(self._bgScreen)
            self.minusButton.draw(self._bgScreen)
            self.closeButton.draw(self._bgScreen)
            self.moreButton.draw(self._bgScreen)
            
            # Draw Map Screen
            self._mapScreen.draw()
            
            # Run Robot
            self._mapScreen.run()
            
            pygame.display.flip()
            self.clock.tick(60)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.stateAnimation = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    if self.closeButton.is_hovered(event.pos):
                        print("Close Button clicked!")
                    if self.moreButton.is_hovered(event.pos):
                        print("Options Button clicked!")
                        
            self.slider.handle_event(event)

    def handleZoom(self, event):
        if event.button == 1:
            if self.plusButton.is_hovered(event.pos):
                zoomLevel *= 1.25
                print("Zoom Plus Button clicked!")
            if self.minusButton.is_hovered(event.pos):
                zoomLevel /= 1.25
                print("Zoom Minus Button clicked!")       
    
    # def handleFiles(self, event, trajectory):
    #     if event.button == 1:
    #         if self.moreButton.is_hovered(event.pos):
    #             print("Options Button clicked!")
        
    def update(self):
        # Update All Screen Available
        self._mapScreen.update(1)

    def bgDraw(self):
        self._bgScreen.fill(WHITE)

    def quit(self):
        pygame.quit()
        sys.exit()

# Running the Application
if __name__ == "__main__":
    app = Visualization("config\\viz\\viz.yaml")
    app.run()
    app.quit()
