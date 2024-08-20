import pygame
import pygame.gfxdraw
import numpy as np
import yaml
from os.path import dirname, join
from csv_parse import csv

# Color
black  = (0, 0, 0)
orange = (255,165,0)
teal   = (42, 157, 244)

def Rt_2D(th):
    return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])

def Rt_2DCenterRef(reference, pose):
    # Transform Body -> Back to Origin (0,0) -> Rotate -> Translate Back to World Frame
    _rotated = np.zeros_like(reference)
    _rotated = reference - np.array([pose[0], pose[1]])
    for n in range(len(reference)): _rotated[n] = (Rt_2D(pose[2]) @ _rotated[n].reshape(-1,1)).reshape(1,-1)
    _rotated += np.array([pose[0], pose[1]])
    return _rotated
    
def show_mouse_coordinate(screen, res, tcolor, font, fontsize):
    # Get World Frame Coordinate
    x, y = pygame.mouse.get_pos()
    # create a font object
    font = pygame.font.Font(font, fontsize)
    # create a text surface object,
    show = (f"World Pose ->  X: {x} Y: {y}")
    text = font.render(show, True, tcolor, None)
    # create a rectangular object for the text surface object
    textRect = text.get_rect()
    _, __, w, h = textRect
    # set the center of the rectangular object.
    textRect.center = (res[0]+w*0.5, res[1])
    screen.blit(text, textRect)
    
def draw_arrow(screen, color, start, end, trirad, thickness=2):
    pygame.draw.line(screen, color, start, end, thickness)
    rotation = (np.arctan2(start[1] - end[1], end[0] - start[0])) + np.pi/2
    pygame.draw.polygon(screen, color, ((end[0] + trirad * np.sin(rotation),
                                        end[1] + trirad * np.cos(rotation)),
                                        (end[0] + trirad * np.sin(rotation - np.radians(120)),
                                        end[1] + trirad * np.cos(rotation - np.radians(120))),
                                        (end[0] + trirad * np.sin(rotation + np.radians(120)),
                                        end[1] + trirad * np.cos(rotation + np.radians(120)))))

def draw_robot(screen, color, pose, size):
    # Robot Body Point Reference
    _pointRef = np.array([[pose[0]-size[0]*0.5, pose[1]-size[1]*0.5],
                          [pose[0]+size[0]*0.5, pose[1]-size[1]*0.5],
                          [pose[0]+size[0]*0.5, pose[1]+size[1]*0.5],
                          [pose[0]-size[0]*0.5, pose[1]+size[1]*0.5]])

    # Robot Wheel Point Reference
    _pointRef = np.array([[pose[0]-size[0]*0.5, pose[1]-size[1]*0.5],
                          [pose[0]+size[0]*0.5, pose[1]-size[1]*0.5],
                          [pose[0]+size[0]*0.5, pose[1]+size[1]*0.5],
                          [pose[0]-size[0]*0.5, pose[1]+size[1]*0.5]])
    
    for i in range(len(dn)):
        _wheelAngleRef = np.array([[pose[0] + (dn[i] * np.cos(bn[i]) - wheelRadius * np.sin(yn[i] + bn[i])), pose[1] + (dn[i] * -np.sin(bn[i]) - wheelRadius * np.cos(yn[i] + bn[i]))],
                                   [pose[0] + (dn[i] * np.cos(bn[i]) + wheelRadius * np.sin(yn[i] + bn[i])), pose[1] + (dn[i] * -np.sin(bn[i]) + wheelRadius * np.cos(yn[i] + bn[i]))]])
 
        _wheelRef = np.array([[_wheelAngleRef[0,0] + wheelWidth * 0.5 * -(_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*wheelRadius), _wheelAngleRef[0,1] + wheelWidth * 0.5 * (_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*wheelRadius)],
                              [_wheelAngleRef[0,0] + wheelWidth * 0.5 * (_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*wheelRadius), _wheelAngleRef[0,1] + wheelWidth * 0.5 * -(_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*wheelRadius)],
                              [_wheelAngleRef[1,0] + wheelWidth * 0.5 * (_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*wheelRadius), _wheelAngleRef[1,1] + wheelWidth * 0.5 * -(_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*wheelRadius)],
                              [_wheelAngleRef[1,0] + wheelWidth * 0.5 * -(_wheelAngleRef[1,1]-_wheelAngleRef[0,1])/(2*wheelRadius), _wheelAngleRef[1,1] + wheelWidth * 0.5 * (_wheelAngleRef[1,0]-_wheelAngleRef[0,0])/(2*wheelRadius)]])
        # Draw Wheel Lines
        pygame.gfxdraw.filled_polygon(screen, Rt_2DCenterRef(_wheelRef, pose), color)
        pygame.gfxdraw.aapolygon(screen, Rt_2DCenterRef(_wheelRef, pose), color)

    # Draw Robot Body Lines
    pygame.draw.aalines(screen, color, True, Rt_2DCenterRef(_pointRef, pose), 3)

def parse_csv():
    # Load data from CSV
    result = csv.load_csv("mission\\test.csv", delim = ",")
    
    # Split the data (ENC and GFC)
    pose = []
    vel = []

    for k in range(1,len(result)):
        # Append GFC array
        pose.append([np.float64(result[k,0]), np.float64(result[k,1]), np.float64(result[k,4])])
        vel.append([np.float64(result[k,2]), np.float64(result[k,3]), np.float64(result[k,5])])
            
    # Convert to NumPy
    pose = np.array(pose)
    vel = np.array(vel)
    
    return pose
    
# Open YAML Config File
current_dir = dirname(__file__)
file_path = join(current_dir, "config\\drive\\fmlx_rover.yaml")
with open(file_path, 'r') as file:
    robot = yaml.safe_load(file)
    
    robotWidth = robot["body"]["width"]
    robotHeight= robot["body"]["height"]
    robotLength = robot["body"]["length"]
    wheelRadius = np.float32(robot["wheel"]["rw"])
    wheelWidth = np.float32(robot["wheel"]["width"])
    
    dn = np.array([val for (key,val) in robot["axle_length"].items()], dtype='f8')
    bn = np.array([np.radians(val) for (key,val) in robot["axle_angle"].items()], dtype='f8')
    yn = np.array([np.radians(val) for (key,val) in robot["wheel_angle"].items()], dtype='f8')

# Set Screen Size    
file_path = join(current_dir, "config\\viz\\viz.yaml")
with open(file_path, 'r') as file:
    viz = yaml.safe_load(file)
    widthScreen  = viz["screen"]["width"]
    heightScreen = viz["screen"]["height"]
    quiverLength = np.float32(viz["wheel_view"]["quiver_length"])
    WheelViewScale = np.float32(viz["wheel_view"]["scale"])/100
    if WheelViewScale > 3 : WheelViewScale = 3
    WorldViewScale = np.float32(viz["world_view"]["scale"])/100
    if WorldViewScale > 3 : WorldViewScale = 3
    
# Set Robot Wheel View Screen
widthWheelViewScreen = widthScreen/3
heightWheelViewScreen = heightScreen
center_x_WheelView = widthWheelViewScreen * 2 + (widthWheelViewScreen / 2)
center_y_WheelView = heightWheelViewScreen / 2

body_viz = list([(center_x_WheelView - robotWidth*0.5*WheelViewScale, center_y_WheelView - robotLength*0.5*WheelViewScale), 
                 (center_x_WheelView + robotWidth*0.5*WheelViewScale, center_y_WheelView - robotLength*0.5*WheelViewScale),
                 (center_x_WheelView + robotWidth*0.5*WheelViewScale, center_y_WheelView + robotLength*0.5*WheelViewScale), 
                 (center_x_WheelView - robotWidth*0.5*WheelViewScale, center_y_WheelView + robotLength*0.5*WheelViewScale)])

wheelContact_viz = np.array([(center_x_WheelView + dn[i]*np.cos(bn[i])*WheelViewScale, center_y_WheelView - dn[i]*np.sin(bn[i])*WheelViewScale) for i in range(len(dn))])

wheelTraction_viz = np.array([(wheelContact_viz[i][0] + wheelRadius*np.sin(yn[i] + bn[i])*WheelViewScale, wheelContact_viz[i][1] + wheelRadius*np.cos(yn[i] + bn[i])*WheelViewScale) for i in range(len(dn))])

wheelAngle_viz=[]
for i in range(len(dn)):
    wheel = list(((center_x_WheelView + (dn[i] * np.cos(bn[i]) - wheelRadius * np.sin(yn[i] + bn[i]))*WheelViewScale, center_y_WheelView + (dn[i] * -np.sin(bn[i]) - wheelRadius * np.cos(yn[i] + bn[i]))*WheelViewScale),
                  (center_x_WheelView + (dn[i] * np.cos(bn[i]) + wheelRadius * np.sin(yn[i] + bn[i]))*WheelViewScale, center_y_WheelView + (dn[i] * -np.sin(bn[i]) + wheelRadius * np.cos(yn[i] + bn[i]))*WheelViewScale)))
    wheelAngle_viz.append(wheel)
    

# Initialize PyGame
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((widthScreen, heightScreen))
pygame.display.set_caption('Robot Simulator')
# Animation Loop
animationState = True

#--------------------------------------------------------------------------- TEST CODE START ---------------------------------------------------------------------------
traj = parse_csv()

# Normalize to Quadrant I
traj = np.array([0, heightScreen, 0]) - traj


# set ticks
ticks = 0
#---------------------------------------------------------------------------- TEST CODE END ----------------------------------------------------------------------------
while animationState:
    # Track User Interaction
    for event in pygame.event.get():
        # Close Event
        if event.type == pygame.QUIT:
            animationState = False
            
    # Set up
    screen.fill((255, 255, 255))

    # Show World Coordinate Text
    fontSize = int(10*WorldViewScale)
    show_mouse_coordinate(screen, ((fontSize*WorldViewScale), heightScreen-(fontSize*WorldViewScale)), (0,0,0), 'freesansbold.ttf', fontSize)
    # draw_robot(screen, black, (widthScreen/2, heightScreen/2, np.radians(90)), (robotWidth,robotLength))
    
    
#--------------------------------------------------------------------------- TEST CODE START ---------------------------------------------------------------------------
    draw_robot(screen, black, traj[0], (robotWidth,robotLength))
    if len(traj) > 1 and ticks%10 == 0:
        traj = traj[1:]
    ticks += 2
#---------------------------------------------------------------------------- TEST CODE END ----------------------------------------------------------------------------

    # Draw Body Lines
    # pygame.draw.lines(screen, line_color, True, body_viz, width=7)
    
    # # Draw Body Center Point
    # pygame.draw.circle(screen, orange, (center_x_WheelView, center_y_WheelView), 5, width=0)
    
    # for line in wheelAngle_viz:
    #     pygame.draw.line(screen, line_color, line[0], line[1], width=3)

    # # Draw Wheel Contact Dot Points
    # i = 0
    # for dot in wheelContact_viz:  
    #     pygame.draw.circle(screen, orange, dot, 5, width=0)
    #     draw_arrow(screen, orange, dot, wheelTraction_viz[i], 13, 7)
    #     i += 1


    # Update Screen
    pygame.display.flip()
    
    # print(ts)
    clock.tick(60)
