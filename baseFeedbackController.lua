function sysCall_init()
         
    vehicleReference=sim.getObjectHandle('youBot_vehicleReference')
    vehicleTarget=sim.getObjectHandle('youBot_vehicleTargetPosition')
    sim.setObjectPosition(vehicleTarget,sim.handle_parent,{0,0,0})
    sim.setObjectOrientation(vehicleTarget,sim.handle_parent,{0,0,0})
    sim.setObjectParent(vehicleTarget,-1,true)
    wheelJoints={-1,-1,-1,-1}
    wheelJoints[1]=sim.getObjectHandle('rollingJoint_fl')
    wheelJoints[2]=sim.getObjectHandle('rollingJoint_rl')
    wheelJoints[3]=sim.getObjectHandle('rollingJoint_rr')
    wheelJoints[4]=sim.getObjectHandle('rollingJoint_fr')
    prevX=0
    prevY=0
    prevZ=0
    
end

function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    posError=sim.getObjectPosition(vehicleTarget,vehicleReference)
    OrientError=sim.getObjectOrientation(vehicleTarget,vehicleReference)

    currX=posError[2]*15
    currY=posError[1]*15 
    currZ=-OrientError[3]*8.8
    
    dx=currX-prevX
    dy=currY-prevY
    dz=currZ-prevZ
    
    currX=prevX+dx
    currY=prevY+dy
    currZ=prevZ+dz
    
    sim.setJointTargetVelocity(wheelJoints[1],-currX-currY-currZ)
    sim.setJointTargetVelocity(wheelJoints[2],-currX+currY-currZ)
    sim.setJointTargetVelocity(wheelJoints[3],-currX-currY+currZ)
    sim.setJointTargetVelocity(wheelJoints[4],-currX+currY+currZ)
    
    prevX=currX
    prevY=currY
    prevZ=currZ
end 

