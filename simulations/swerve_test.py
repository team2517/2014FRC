import math

import pygame
from pygame.locals import *

#Test line for commit

FONT = None

class wheelVector:
    def __init__(self):
        x       =0.0
        y       =0.0
        mag     =0.0
        tarTheta=0.0
        curTheta=0.0
        turnVel =0.0

class robot_(object):

    FL=0
    FR=1
    BR=2
    BL=3

    def __init__(self,joystick):
        self.joystick=joystick
        
        self.robot_box = pygame.Rect((0,0),(200,200))
        self.robot_box.center = 150,450

        self.wheel=(wheelVector(),wheelVector(),wheelVector(),wheelVector())

        #debug displays

        #self.wheel motor values, not in self.displays due to use in other parts than self.draw()
        self.wheel_FL_mag = var_display('FL',(175,50))
        self.wheel_FR_mag = var_display('FR',(175,75))
        self.wheel_BL_mag = var_display('BL',(175,100))
        self.wheel_BR_mag = var_display('BR',(175,125))
        self.wheel_FL_ang = var_display('FL',(300,50))
        self.wheel_FR_ang = var_display('FR',(300,75))
        self.wheel_BL_ang = var_display('BL',(300,100))
        self.wheel_BR_ang = var_display('BR',(300,125))
        
        self.displays = {
                          'angle':var_display('AGL',(50,50)),
                          'magnitude':var_display('MAG',(50,75)),
                          'vx':var_display('VX',(50,100)),
                          'vy':var_display('VY',(50,125)),
                          'z':var_display('ROT',(50,150))
        }


    def calculate_move_vector(self):
        #get joystick movement and output to self.m_vector angle in radians and magnitude.
        
        self.x = self.joystick.get_axis(0)
        self.y = self.joystick.get_axis(1)
        self.z = self.joystick.get_axis(2)
        vx,vy = self.x,self.y #incase we need to math on these, use intermediate values
        distance = math.hypot(vx,vy)
        if distance >1:
            distance = 1.0
        elif distance <-1:
            distance = -1.0
        try:
            angle = math.atan2(vy,vx)
            angle = angle * -1.0
            if angle < 0:
                angle = angle + (2*math.pi)
        except Exception as e:
            angle = math.pi/2

        if distance < 0.01:
            angle = 0
        
        self.displays['angle'].set(math.degrees(angle))
        self.displays['magnitude'].set(distance)
        self.displays['vx'].set(vx)
        self.displays['vy'].set(vy)

        self.m_vector=(angle,distance,(vx*100,vy*100))

    def calculate_wheel_values(self):
        self.displays['z'].set(self.z)
            

        leftx = self.x
        lefty = self.y
        rightx =self.z
        if self.x==0 and self.y == 0:
            print "joys low!! x: %s y: %s z: %s "%(self.x,self.y,self.z)
            return
        
        ##TODO: get leftx/y ect ect
        self.wheel[self.FL].x = 0.707 * rightx
        self.wheel[self.FL].y = 0.707 * rightx
        self.wheel[self.FR].x = 0.707 * rightx
        self.wheel[self.FR].y = -0.707 * rightx
        self.wheel[self.BL].x = -0.707 * rightx
        self.wheel[self.BL].y = -0.707 * rightx
        self.wheel[self.BR].x = -0.707 * rightx
        self.wheel[self.BR].y = 0.707 * rightx


        for wh in self.wheel:
            wh.x += leftx
            wh.y += lefty
            wh.mag = math.sqrt(math.pow(wh.x, 2.0)+math.pow(wh.y,2.0))

        for wh in self.wheel:
            if wh.mag > 1.0:
                mag_devide = wh.mag
                for j in range(len(self.wheel)):
                    self.wheel[j].mag = self.wheel[j].mag / mag_devide

        for wh in self.wheel:
            if wh.y == 0 or wh.x == 0:
                print "joys low!! x: %s y: %s z: %s "%(self.x,self.y,self.z)
                print "wh.x %s, wh.y %s"%(wh.x,wh.y)
                wh.x =0
                wh.y=0
                wh.tarTheta=0
                wh.mag=0
                continue
            wh.tarTheta = math.atan(wh.y/wh.x)
            if wh.x < 0:
                wh.tarTheta += math.pi
        for i in range(len(self.wheel)):
            #print "move self.wheel %i at angle %f at speed %f"%(i,self.wheel[i].tarTheta,self.wheel[i].mag)
            print "move self.wheel %i over %f x and %f y"%(i, self.wheel[i].x, self.wheel[i].y)
        self.wheel_FL_mag.set(self.wheel[self.FL].mag)
        self.wheel_FR_mag.set(self.wheel[self.FR].mag)
        self.wheel_BL_mag.set(self.wheel[self.BL].mag)
        self.wheel_BR_mag.set(self.wheel[self.BR].mag)

        self.wheel_FL_ang.set(self.wheel[self.FL].tarTheta)
        self.wheel_FR_ang.set(self.wheel[self.FR].tarTheta)
        self.wheel_BL_ang.set(self.wheel[self.BL].tarTheta)
        self.wheel_BR_ang.set(self.wheel[self.BR].tarTheta)

    def draw(self,screen):
        #draw robot "base" first, layer other things over it
        pygame.draw.rect(screen,(0,255,75),self.robot_box,2)

        #get new movement vectors to draw
        self.calculate_move_vector()
        self.calculate_wheel_values()

        pygame.draw.arc(screen, (255,255,255), self.robot_box, 0, self.m_vector[0], 1)
        vector_end = (self.m_vector[2][0]+self.robot_box.center[0]),(self.m_vector[2][1]+self.robot_box.center[1])
        pygame.draw.line(screen, (255,0,0),self.robot_box.center,vector_end,1)
        #rot line
        rot_line_start = self.robot_box.midtop[0], self.robot_box.midtop[1]-10 #10 pixels above
        rot_line_end   = rot_line_start[0]+(self.z*100),rot_line_start[1]
        pygame.draw.line(screen,(255,0,255),rot_line_start,rot_line_end,4)

        pygame.draw.line(screen, (125, 0, 125), (200, 100), (200 + self.wheel[self.FL].x, 100 + self.wheel[self.FL].y))

        self.wheel_FL_mag.draw(screen)
        self.wheel_FR_mag.draw(screen)
        self.wheel_BL_mag.draw(screen)
        self.wheel_BR_mag.draw(screen)

        self.wheel_FL_ang.draw(screen)
        self.wheel_FR_ang.draw(screen)
        self.wheel_BL_ang.draw(screen)
        self.wheel_BR_ang.draw(screen)

        for key,disp in self.displays.items():
            disp.draw(screen)


class var_display(object):
    def __init__(self,name,cpos):
        self.center = cpos
        self.val = 0
        self.name = name

    def set(self,val):
        self.val = val
    def draw(self,screen):
        surf = FONT.render('%s:%0.3f'%(self.name,self.val),True,(255,255,255))
        rect = surf.get_rect()
        rect.center = self.center
        screen.blit(surf,rect)

def main():
    pygame.init()
    pygame.joystick.init()
    global FONT
    FONT = pygame.font.SysFont("Courier", 20)
    
    screen = pygame.display.set_mode((800,600),pygame.SRCALPHA)

    clock=pygame.time.Clock()
    j = pygame.joystick.Joystick(0)
    j.init()

    robot = robot_(j)
    

    while True:
        timedelta = clock.tick(30)
        events = pygame.event.get()
        for event in events:
            if event.type == QUIT:
                pygame.quit()
                return 
            elif event.type == MOUSEBUTTONDOWN and event.button==1:
                print event.pos
            
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    return
        screen.fill((0,0,0))
        robot.draw(screen)

        pygame.display.flip()

if __name__ == '__main__':
    main()
