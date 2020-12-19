function sysCall_init()
    simRemoteApi.start(19999)
    sim.clearStringSignal('mainthread')
end

function goandpickup(inInts,inFloats,inStrings,inBuffer)
    if #inStrings>=1 then
        sim.setIntegerSignal("code", inInts[1])
        sim.setIntegerSignal("gotsec", inInts[2])
        sim.setDoubleSignal("CX", inFloats[1])
        sim.setDoubleSignal("CY", inFloats[2])
        sim.setDoubleSignal("A", inFloats[3])
        sim.setStringSignal('mainthread',inStrings[1])
        sim.setDoubleSignal("C1X", inFloats[4])
        sim.setDoubleSignal("C1Y", inFloats[5])
        sim.setDoubleSignal("A1", inFloats[6])
        
        return {},{},{'message was displayed'},''
    end
end



function sysCall_cleanup()
end
