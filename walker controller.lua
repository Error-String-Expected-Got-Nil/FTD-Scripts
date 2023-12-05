--TODO:
--  -Add hover axis control (link into pitch/roll)
--  -Rework step length so it isn't based on an angle anymore
--  -Use Lerp for steps (min distance backwards, max distance forwards)
--  -Make step length leg-specific
--  -Allow ankles to turn to ensure feet are in direction of motion
--  -Add custom axis "splayLegs", stopping all leg motion and splaying the legs if it is at 1
--  -Add strafe axis
--  -Make yaw axis cause legs to move tangent to center of mass

deg = Mathf.Rad2Deg
rad = Mathf.Deg2Rad
pi2 = Mathf.PI * 2
startupLog = ""
function logBuffer(str) startupLog = startupLog .. str .. "\n" end

--###########################################################################################
--#                                                                                         #
--#   Spinblocks should be oriented such that clockwise (positive rotation) means upwards   #
--#         (or their global drive response set to -1 if they are counterclockwise)         #
--#                                                                                         #
--#   Additionally, all spinblocks should be in "Rotate at a speed determined by the rate   #
--#                  controller" mode, with rotation speed set to maximum.                  #
--#                                                                                         #
--###########################################################################################

config = {
    stepAngle = 30;             --Farthest angle, in degrees, a leg should move away from its default angle while moving.
    cycleDuration = 2;          --Amount of time, in seconds, it takes for a single walk cycle to pass.
    deltaTime = 1/40;           --Amount of time, in seconds, that passes each tick. Should be 1/40th of a second, the length of an FTD physics step.
    walkDriveDeltaCap = 1/40;   --Maximum amount a leg's walking drive response (main, forward, yaw) can change in a single tick. Exists to smooth out jerky motion.
    heightDriveDeltaCap = 1/40; --Maximum amount a leg's height drive response (pitch, roll) can change in a single tick. Exists to smooth out jerky motion.
}

--(Terminology note: "Vertical" is the vehicle's own up/down axis, "lateral" is its left/right axis, and "medial" is its forward/backward axis.)
--Add an unkeyed table for each leg on the craft into the following table, with these arguments in this order:

--name:                 Name of leg, looks for all spinblocks named leg_<name>_<hip/root/knee/foot/ankle> to determine the spinblocks apart of this leg.
--restReachMedial:      How far the foot should be from the root along the vehicle's medial axis while at rest.
--restReachLateral:     How far the foot should be from the root along the vehicle's lateral axis while at rest.
--cycleOffset:          Offset to the cycle of the leg. Used so you can, say, have half the legs off the ground and the other half on the ground while in motion.
--forwardResponse:      Weight of the forward drive request when determining the leg's medial response.
--mainResponse:         Weight of the main drive request when determining the leg's medial response.
--pitchResponse:        Weight of the pitch drive request when determining the leg's vertical response.
--rollResponse:         Weight of the roll drive request when determining the leg's vertical response.
--hoverResponse:        Weight of the hover drive request when determining the leg's vertical response.
--strafeResponse:       Weight of the strafe drive request when determining the leg's lateral response.
--yawResponseMedial:    Weight of the yaw drive request when determining the leg's medial response.
--yawResponseLateral:   Weight of the yaw drive request when determining the leg's lateral response.
--forwardStride:        Maximum distance forward from resting position in a step.
--backwardStride:       Maximum distance backward from resting position in a step.
--outStride:            Maximum distance from resting position, laterally away from the root, in a step. 
--inStride:             Maximum distance from resting position, laterally towards the root, in a step. 
--heightDeviation:      Maximum the base standing height can change by due to pitch/roll request.
--groundOffset:         Distance from leg root to ground vertically. Should be negative.
--raiseOffset:          Distance from leg root to its highest raised position during the walk cycle. Should be greater than groundOffset.
--rootLength:           Length of the segment attached to the root spinblock of the leg (the one directly attached to the hip). Includes knee spinblock!
--kneeLength:           Length of the segment attached to the knee spinblock of the leg (the one attached to the root segment). Includes foot spinblock!

legSettings = {
  --{  name, rm, rl, co, fr, mr, pr, rr, hr, sr, ym, yl, fs, bs, os, is, hd, go, ro, rl, kl, re};
    {"test",  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -4, -2,  5,  9,  8};
}



--############################################################################
--#                                                                          #
--#   inverseKinematics returns and accepts angles IN RADIANS, NOT DEGREES   #
--#                                                                          #
--############################################################################

inverseKinematics = {}

function inverseKinematics.solveExtension(rootLength, kneeLength, distance, angleOffset)
    angleOffset = angleOffset or 0

    local midpoint = (rootLength ^ 2 - kneeLength ^ 2) / (2 * distance) + (distance / 2)

    local rootAngle = Mathf.Acos(midpoint / rootLength)
    local kneeAngle = -Mathf.Acos((distance - midpoint) / kneeLength)
    local footAngle = -kneeAngle

    kneeAngle = kneeAngle - rootAngle
    rootAngle = rootAngle + angleOffset
    footAngle = footAngle - angleOffset

    return rootAngle, kneeAngle, footAngle
end

function inverseKinematics.solveCoordinates(x, y, rootLength, kneeLength)
    local angleOffset = Mathf.Atan2(y, x)
    local distance = (x ^ 2 + y ^ 2) ^ 0.5

    return inverseKinematics.solveExtension(rootLength, kneeLength, distance, angleOffset)
end



legController = {}
legController.spinblockList = {}
legController.legList = {}
legController.mt = {__call = legController}

--I: The "I" variable from the Update() function.
--For other arguments, see the settings table at the top of the program.
function legController.new(I, name, restReachMedial, restReachLateral, cycleOffset, forwardResponse, mainResponse, pitchResponse, rollResponse, hoverResponse, strafeResponse, 
        yawResponseMedial, yawResponseLateral, forwardStride, backwardStride, outStride, inStride, heightDeviation, groundOffset, raiseOffset, rootLength, kneeLength)
    local leg = {}

    leg.name = name

    leg.hipID = legController.spinblockList["leg_" .. name .. "_hip"]
    leg.rootID = legController.spinblockList["leg_" .. name .. "_root"]
    leg.kneeID = legController.spinblockList["leg_" .. name .. "_knee"]
    leg.footID = legController.spinblockList["leg_" .. name .. "_foot"]
    leg.ankleID = legController.spinblockList["leg_" .. name .. "_ankle"]

    if not leg.hipID then logBuffer("[WARN] Failed to find hip spinblock for leg \"" .. name .. "\"!") end
    if not leg.rootID then logBuffer("[WARN] Failed to find root spinblock for leg \"" .. name .. "\"!") end
    if not leg.kneeID then logBuffer("[WARN] Failed to find knee spinblock for leg \"" .. name .. "\"!") end
    if not leg.footID then logBuffer("[WARN] Failed to find foot spinblock for leg \"" .. name .. "\"!") end
    if not leg.ankleID then logBuffer("[WARN] Failed to find ankle spinblock for leg \"" .. name .. "\"!") end

    leg.restReachMedial = restReachMedial
    leg.restReachLateral = restReachLateral

    leg.cycleOffset = cycleOffset

    leg.forwardResponse = forwardResponse
    leg.yawResponse = yawResponse
    leg.mainResponse = mainResponse
    leg.pitchResponse = pitchResponse
    leg.rollResponse = rollResponse
    leg.hoverResponse = hoverResponse
    leg.strafeResponse = strafeResponse

    leg.forwardStride = forwardStride
    leg.backwardStride = backwardStride
    leg.outStride = outStride
    leg.inStride = inStride

    leg.heightDeviation = heightDeviation
    leg.groundOffset = groundOffset
    leg.raiseOffset = raiseOffset

    leg.rootLength = rootLength
    leg.kneeLength = kneeLength

    leg.controller = legController.newThread(leg, I)
    table.insert(legController.legList, leg)

    setmetatable(leg, legController.mt)

    return leg
end

function legController.actionThread(leg, I)
    local cycleCounter, forwardRequest, yawRequest, mainRequest, walkRequest, pitchRequest, rollRequest, heightRequest, hipAngle, rootAngle, kneeAngle, footAngle, stepLength, heightModifier
    stepLength = config.stepAngle
    local walkResponse, heightResponse = 0, 0
    
    while true do
        cycleCounter, forwardRequest, yawRequest, mainRequest, pitchRequest, rollRequest = coroutine.yield()

        
    end
end

function legController.newThread(leg, I)
    local thread = coroutine.create(legController.actionThread)
    coroutine.resume(thread, leg, I)
    return thread
end

function legController.splay(leg, I)
    I:SetSpinBlockRotationAngle(leg.hipID, 0)
    I:SetSpinBlockRotationAngle(leg.rootID, 0)
    I:SetSpinBlockRotationAngle(leg.kneeID, 0)
    I:SetSpinBlockRotationAngle(leg.footID, 0)
    I:SetSpinBlockRotationAngle(leg.ankleID, 0)
end



function indexNamedSpinblocks(I)
    local spinblocks = {}

    for i = 0, I:GetAllSubconstructsCount() - 1 do
        local id = I:GetSubConstructIdentifier(i)
        if I:IsSpinBlock(id) then
            local blockInfo = I:GetSubConstructInfo(id)
            if blockInfo.CustomName ~= "" then
                spinblocks[blockInfo.CustomName] = id
            end
        end
    end

    return spinblocks
end

isStartup = true
spinblocks = {}
cycleStopwatch = 0

function Update(I)
    if isStartup then
        I:ClearLogs()

        spinblocks = indexNamedSpinblocks(I)

        logBuffer("Detected the following named spinblocks:")
        for k, v in pairs(spinblocks) do logBuffer(k .. " (" .. v .. ")") end

        legController.spinblockList = spinblocks

        for index, settings in ipairs(legSettings) do
            legController.new(I, unpack(settings))
        end

        I:Log(startupLog)
        isStartup = false
    end

    local mainDrive = I:GetPropulsionRequest(0)
    local forwardDrive = I:GetPropulsionRequest(6)
    local yawDrive = I:GetPropulsionRequest(5)
    local pitchDrive = I:GetPropulsionRequest(4)
    local rollDrive = I:GetPropulsionRequest(3)

    for index, leg in ipairs(legController.legList) do
        coroutine.resume(leg.controller, cycleStopwatch, forwardDrive, yawDrive, mainDrive, pitchDrive, rollDrive) 
    end

    cycleStopwatch = (cycleStopwatch + config.deltaTime / config.cycleDuration) % 1 
end