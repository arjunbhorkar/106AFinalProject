gripperTarget=sim.getObjectHandle('youBot_gripperPositionTarget')
gripperTip=sim.getObjectHandle('youBot_gripperPositionTip')
vehicleReference=sim.getObjectHandle('youBot_vehicleReference')
vehicleTarget=sim.getObjectHandle('youBot_vehicleTargetPosition')
vT = sim.getObjectHandle('youBot_vT')
vH = sim.getObjectHandle('youb')
armJoints={-1,-1,-1,-1,-1}
for i=0,4,1 do
    armJoints[i+1]=sim.getObjectHandle('youBotArmJoint'..i)
end
ik1=sim.getIkGroupHandle('ik1')
ik2=sim.getIkGroupHandle('ik2')
gripperCommunicationTube=sim.tubeOpen(0,'youBotGripperState'..sim.getNameSuffix(nil),1)
pickup1={0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,0*math.pi/180}
reddrop = sim.getObjectHandle('redD')
bluedrop = sim.getObjectHandle('blueD')
greendrop = sim.getObjectHandle('greenD')
platform={0,28.47*math.pi/180,55.09*math.pi/180,78.32*math.pi/180,0*math.pi/180}

ikSpeed={0.2,0.2,0.2,0.2}
ikAccel={0.1,0.1,0.1,0.1}
ikJerk={0.1,0.1,0.1,0.1}
fkSpeed={1,1,1,1,1}
fkAccel={0.6,0.6,0.6,0.6,0.6}
fkJerk={1,1,1,1,1}


visualizePath=function(path)
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,3,0,-1,99999,{0.2,0.2,0.2})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/3
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*3+1],path[(i-1)*3+2],initPos[3],path[i*3+1],path[i*3+2],initPos[3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

setIkMode=function(withOrientation)
    sim.setThreadAutomaticSwitch(false)
    if (ikMode==false) then
        sim.setObjectPosition(gripperTarget,-1,sim.getObjectPosition(gripperTip,-1))
    end
    sim.setExplicitHandling(ik1,0)
    sim.setExplicitHandling(ik2,0)
    for i=1,5,1 do
        sim.setJointMode(armJoints[i],sim.jointmode_ik,1)
    end
    ikMode=true
    sim.setThreadAutomaticSwitch(true)
end

setFkMode=function()
    sim.setThreadAutomaticSwitch(false)
    sim.setExplicitHandling(ik1,1)
    sim.setExplicitHandling(ik2,1)
    for i=1,5,1 do
        sim.setJointMode(armJoints[i],sim.jointmode_force,0)
    end
    ikMode=false
    sim.setThreadAutomaticSwitch(true)
end

openGripper=function()
    sim.tubeWrite(gripperCommunicationTube,sim.packInt32Table({1}))
    sim.wait(0.8)
end

closeGripper=function()
    sim.tubeWrite(gripperCommunicationTube,sim.packInt32Table({0}))
    sim.wait(0.8)
end

dropToPlatform=function(platform)
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,{0.3,0.3,0.3,0.3,0.3},fkJerk,platform,nil)
    sim.wait(2)
    openGripper()
end

pickFromPlatform=function(platform)
    setFkMode()
    openGripper()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,{0.3,0.3,0.3,0.3,0.3},fkJerk,platform,nil)
    sim.wait(2)
    closeGripper()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickup1,nil)
    openGripper()
end

pathAndTraverse=function(x, y, a)
    sim.setObjectPosition(vT,-1,{x,y - 0.25,0})
    sim.setObjectOrientation(vT,-1,{0,0,a})
    
    initPos=sim.getObjectPosition(vehicleTarget,-1)
    initOrient=sim.getObjectOrientation(vehicleTarget,-1)
    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('2d',simOMPL.StateSpaceType.pose2d,vehicleTarget,{-2,-2},{2,2},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getCollectionHandle('Robot'),sim.getObjectHandle('block')})
    print({sim.getCollectionHandle('Robot'),sim.handle_all})
    startpos=sim.getObjectPosition(vehicleReference,-1)
    startorient=sim.getObjectOrientation(vehicleReference,-1)
    startpose={startpos[1],startpos[2],startorient[3]}
    simOMPL.setStartState(t,startpose)
    goalpos=sim.getObjectPosition(vT,-1)
    goalorient=sim.getObjectOrientation(vT,-1)
    goalpose={goalpos[1],goalpos[2],goalorient[3]}
    simOMPL.setGoalState(t,goalpose)
    r,path=simOMPL.compute(t,4,-1,800)
    visualizePath(path)
    for i=1,#path-3,3 do
        pos={path[i],path[i+1],initPos[3]}
        orient={initOrient[1],initOrient[2],path[i+2]}
        sim.setObjectPosition(vehicleTarget,-1,pos)
        sim.setObjectOrientation(vehicleTarget,-1,orient)
        sim.switchThread()
    end
    repeat
        sim.switchThread()
        p1=sim.getObjectPosition(vehicleTarget,-1)
        p2=sim.getObjectPosition(vehicleReference,-1)
        p={p2[1]-p1[1],p2[2]-p1[2]}
        pError=math.sqrt(p[1]*p[1]+p[2]*p[2])
        oError=math.abs(sim.getObjectOrientation(vehicleReference,vehicleTarget)[3])
    until (pError<0.001)and(oError<0.1*math.pi/180)
end

pickupAndTransport=function(x, y, a, gotsec, x1, y1, a1, code)
    sim.setObjectParent(gripperTarget,vehicleReference,true)
    setFkMode()
    openGripper()
    
    pathAndTraverse(x, y, 0)

    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickup1,nil)
    print("got1")
    setIkMode(true)
    p=sim.getObjectPosition(gripperTarget,-1)
    print("got2")
    p[1]=x
    p[2]=y
    p[3]=0.035
    p[3]=p[3]+0.05
    m=sim.buildMatrix(p,{0,0,a+math.pi/2})
    q = sim.getQuaternionFromMatrix(m)
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,q,nil)
    print("got3")
    sim.wait(5)
    p[3]=p[3]-0.05
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,q,nil)
    closeGripper()
    p[3]=p[3]+0.05
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,q,nil)
    print("got4")
    dropToPlatform(platform)
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickup1,nil)
    
    if gotsec == 1 then
        
        pathAndTraverse(x1, y, 0)
        
        print(a)
        
        sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickup1,nil)
        print("got1")
        setIkMode(true)
        p=sim.getObjectPosition(gripperTarget,-1)
        print("got2")
        p[1]=x1
        p[2]=y1
        p[3]=0.035
        p[3]=p[3]+0.05
        m=sim.buildMatrix(p,{0,0,a1+math.pi/2})
        q = sim.getQuaternionFromMatrix(m)
        sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,q,nil)
        print("got3")
        sim.wait(5)
        
        p[3]=p[3]-0.05
        sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,q,nil)
        closeGripper()
        p[3]=p[3]+0.05
        sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,q,nil)
        print("got4")
        
    end
    
    sim.setStringSignal("pick", "pick")
    
    droppoint = reddrop
    if code == 2 then 
        droppoint = bluedrop
    end
    if code == 3 then
        droppoint = greendrop
    end
    
    pos = sim.getObjectPosition(droppoint, -1)
    ori = sim.getObjectOrientation(droppoint, -1)
    
    pathAndTraverse(pos[1], pos[2], ori[3])
    openGripper()
    pickFromPlatform(platform)
    sim.setStringSignal("pick", "finish")
    sim.wait(2)
    sim.setStringSignal("pick", "notpick")
    
end 



function sysCall_threadmain()
    
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        local path=sim.getStringSignal('mainthread')
        local code= sim.getIntegerSignal('code')
        local gotsec= sim.getIntegerSignal('gotsec')
        local x = sim.getDoubleSignal('CX')
        local y = sim.getDoubleSignal('CY')
        local a = sim.getDoubleSignal('A')
        local x1 = sim.getDoubleSignal('C1X')
        local y1 = sim.getDoubleSignal('C1Y')
        local a1 = sim.getDoubleSignal('A1')
        sim.clearStringSignal('pick')
        sim.setStringSignal("pick", "notpick")
        if path and #path>0 and code > 0 then
        
            pickupAndTransport(x, y, a, gotsec, x1, y1, a1, code)
            
            sim.clearStringSignal('mainthread')
            sim.clearIntegerSignal('code')
            sim.clearIntegerSignal('gotsec')
            sim.clearDoubleSignal('CX')
            sim.clearDoubleSignal('CY')
            sim.clearDoubleSignal('A')
            sim.clearDoubleSignal('C1X')
            sim.clearDoubleSignal('C1Y')
            sim.clearDoubleSignal('A1')
        end
        sim.switchThread()
    end
    
end

function sysCall_cleanup()
end