# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import vrep
import numpy as py
import matplotlib.pyplot as plt
import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    vrep.simxGetObjectHandle(clientID, "Revolute_joint", vrep.simx_opmode_blocking)
    #las = vrep.simxGetObjectHandle(clientID, "LaserScannerLaser_2D", vrep.simx_opmode_blocking)
#    returnCode, detectionState, detectedPoint, detectedObjectHandle, NormalVector =
    #vrep.simxReadProximitySensor(clientID, "LaserScannerLaser_2D", vrep.simx_opmode_streaming)
    ret_x, val_x = vrep.simxGetFloatSignal(clientID, "signal_x", vrep.simx_opmode_streaming)
    ret_y, val_y = vrep.simxGetFloatSignal(clientID, "signal_y", vrep.simx_opmode_streaming)

    while True:
        ret_x, val_x = vrep.simxGetFloatSignal(clientID, "signal_x", vrep.simx_opmode_buffer)
        ret_y, val_y = vrep.simxGetFloatSignal(clientID, "signal_y", vrep.simx_opmode_buffer)
        print(val_x)
        print(val_y)

        plt.scatter(val_x, val_y, color='red')
        plt.pause(0.05)

        plt.show()
        #plt.plot([val])

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(5)

    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot)
    #plt.plot([1, 2, 3, 4])
    plt.show()

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Stop the simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
