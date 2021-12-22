# -*- coding: utf-8 -*-
"""
Way Point navigtion

(c) S. Bertrand
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# robot
x0 = -20
y0 = -20
theta0 = np.pi/4.0
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=1, random=True)


# position control loop: gain and timer
kpPos = 0.8
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 6
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)



# list of way points list of [x coord, y coord]
WPlist = [[x0,y0], [x0+1, y0+1]] # on ajoute ces points pour lancer le robot

#threshold for change to next WP
epsilonWP = 0.1
# init WPManager
WPManager = rob.WPManager(WPlist, epsilonWP)


t0 = 0.0
tf = 10000.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0
firstIter = True

histoPol = [[0,0,0]]# historique polution
vect = []
newWP = []
maxpol = []
mouv = 0
posfin = []
new1 = [1,1]
m1 = 0
x2=0
# loop on simulation time
for t in simu.t: 
   
    
    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        potentialValue = pot.value([robot.x, robot.y]) # do not change 
        if m1==0 : # mission 1 debute
            if (np.sqrt(pow(WPManager.xr-robot.x, 2) + pow(WPManager.yr - robot.y, 2)) < WPManager.epsilonWP ) :                
                vecx = WPManager.WPList[0][0] - WPManager.xr    
                vecy = WPManager.WPList[0][1] - WPManager.yr
                vect = [vecx , vecy] # vecteur de deplacement actuel
                histoPol.append([robot.x, robot.y, pot.value([robot.x, robot.y])])
                
                if len(posfin) == 5 : # on effectue au aximum 5 virages avant d'avoir completement encerclé l'origine de la pollution
                    maxpol = [ ( posfin[len(posfin)-1][0] + posfin[len(posfin)-3][0] ) /2 , ( posfin[len(posfin)-1][1] + posfin[len(posfin)-3][1] ) /2 ]
                    WPlist=[]
                    WPManager.WPList=[]
                    WPManager.WPList.append(maxpol)
                    WPlist.append(maxpol)
                    fin1 = 1
                    if (np.sqrt(pow(maxpol[0]-robot.x, 2) + pow(maxpol[1] - robot.y, 2)) < WPManager.epsilonWP ) : # on es arrivé à la source
                        vect = [0,0]
                        Maxpol = pot.value([robot.x, robot.y])
                        print("polution max = ", Maxpol, " à la position ", maxpol)
                        m1 = 1 # marque la fin de la mission 1
                        
                if (histoPol[len(histoPol)-2][2]) > (pot.value([robot.x, robot.y])) : # -2 car on vient d'ajouter une valeur dans histopol
                    mouv = 1
                    vect = new1
                    new1 = [-vect[1], vect[0]] # new1 orthogonal à vect. il part à gauche
                    newWP = [ (WPlist[len(WPlist)-1][0] + new1[0])  , (WPlist[len(WPlist)-1][1] + new1[1]) ] 
                    WPlist.append(newWP)
                    WPManager.WPList.append(newWP)
                    WPManager.switchToNextWP()
                    
                else :
                    if mouv == 1 :
                        posfin.append(WPlist[len(WPlist)-1])
                        mouv = 0
                    newWP = [ (WPlist[len(WPlist)-1][0] + vect[0])  , (WPlist[len(WPlist)-1][1] + vect[1]) ] # -2 pour au passage revenir au point d'avant quand on a commencé à s'éloigner de la source
                    WPlist.append(newWP)
                    WPManager.WPList.append(newWP)
                    WPManager.switchToNextWP()   
        else :
            # mission 2 debute
            if (np.sqrt(pow(WPManager.xr-robot.x, 2) + pow(WPManager.yr - robot.y, 2)) < WPManager.epsilonWP ) :
                new1 = [-3,0] # déplacement 3 par 3 pour aller plus vite. 
                if( pot.value([robot.x, robot.y]) > 290 ):
                    newWP = [ newWP[0] + new1[0]  , newWP[1] + new1[1] ]
                    WPlist.append(newWP)
                    WPManager.WPList.append(newWP) 
                    start_cart = [newWP[0], newWP[1]] # point de depart du cercle
                    WPManager.switchToNextWP()
                    dist = np.sqrt(pow(start_cart[0]-maxpol[0], 2) + pow(start_cart[1]-maxpol[1], 2))
                    x = maxpol[0]-dist
                else :       
                    if x == maxpol[0]+dist: # on est arrivé à la fin du demi cercle du haut, on commence la partie basse
                        ys2 = -np.sqrt(abs(pow(dist,2)-pow(x2-maxpol[0], 2))) # partie basse du cerle
                        y2 = ys2 + maxpol[1]
                        newWP = [x2, y2]
                        WPlist.append(newWP)
                        WPManager.WPList.append(newWP)
                        WPManager.switchToNextWP()
                        x2=x2-1
                        if (np.sqrt(pow(start_cart[0]-robot.x, 2) + pow(start_cart[1] - robot.y, 2)) < WPManager.epsilonWP ) :
                            break
                    else:
                        dist = np.sqrt(abs(pow(start_cart[0]-maxpol[0], 2) + pow(start_cart[1]-maxpol[1], 2))) # distance entre le point de debut de cartographie et la source
                        x=x+1 # incrementation du x pour calculer le y associé
                        ys1 = np.sqrt(abs(pow(dist,2)-pow(x-maxpol[0], 2))) # partie haute du cercle
                        y1 = ys1 + maxpol[1]
                        newWP = [x, y1]
                        WPlist.append(newWP)
                        WPManager.WPList.append(newWP)
                        WPManager.switchToNextWP()
                        x2 = maxpol[0]+dist # valeur de x2 pour commencer le deuxieme demi-cercle
                    
                    
                    

                    
                
                
        # velocity control input
        Vr = kpPos*np.sqrt(pow(WPManager.xr-robot.x, 2) + pow(WPManager.yr - robot.y, 2)) 
        
        
        # reference orientation
        thetar = np.arctan2(WPManager.yr-robot.y, WPManager.xr-robot.x)
        
        
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)        
        
      
        
    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input        
        omegar = kpOrient*(thetar-robot.theta)
    
    
    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)    
    
    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
    
    


    #print(WPManager.xr)
    #print(WPlist[0][0])  
    
# end of loop on simulation time
    
    
    
    


# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

simu.plotXYTheta(2)
#simu.plotVOmega(3)

simu.plotPotential(4)



simu.plotPotential3D(5)


# show plots
#plt.show()





# # Animation *********************************
# fig = plt.figure()
# ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
# ax.grid()
# ax.set_xlabel('x (m)')
# ax.set_ylabel('y (m)')

# robotBody, = ax.plot([], [], 'o-', lw=2)
# robotDirection, = ax.plot([], [], '-', lw=1, color='k')
# wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
# time_template = 'time = %.1fs'
# time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
# potential_template = 'potential = %.1f'
# potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
# WPArea, = ax.plot([], [], ':', lw=1, color='b')

# thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
# xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
# yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

# def initAnimation():
#     robotDirection.set_data([], [])
#     robotBody.set_data([], [])
#     wayPoint.set_data([], [])
#     WPArea.set_data([], [])
#     robotBody.set_color('r')
#     robotBody.set_markersize(20)    
#     time_text.set_text('')
#     potential_text.set_text('')
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  
    
# def animate(i):  
#     robotBody.set_data(simu.x[i], simu.y[i])          
#     wayPoint.set_data(simu.xr[i], simu.yr[i])
#     WPArea.set_data(simu.xr[i]+xWPArea.transpose(), simu.yr[i]+yWPArea.transpose())    
#     thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
#     thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
#     robotDirection.set_data(thisx, thisy)
#     time_text.set_text(time_template%(i*simu.dt))
#     potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea

# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
#     interval=4, blit=True, init_func=initAnimation, repeat=False)
# #interval=25

# #ani.save('robot.mp4', fps=15)

