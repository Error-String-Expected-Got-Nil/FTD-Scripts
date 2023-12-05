--TODO:
--  -Add hover axis control (link into pitch/roll)
--  -Rework step length so it isn't based on an angle anymore
--  -Use Lerp for steps (min distance backwards, max distance forwards)
--  -Make step length leg-specific
--  -Allow ankles to turn to ensure feet are in direction of motion
--  -Add custom axis "splayLegs", stopping all leg motion and splaying the legs if it is at 1
--  -Add strafe axis
--  -Make yaw axis cause legs to move tangent to center of mass
--  -Automatically determine leg segment lengths

deg = Mathf.Rad2Deg
rad = Mathf.Deg2Rad
pi2 = Mathf.PI * 2
startupLog = ""
function logBuffer(str) startupLog = startupLog .. str .. "\n" end

--#################################################################################################################
--#                                                                                                               #
--#   This is a general-purpose controller for insectoid walker legs. It should be capable of handling the vast   # 
--#          majority of walkers with such legs, even some exceptionally weird ones, if set up properly.          #  
--#                                                                                                               #
--#   If you do unnatural things to the configuration and perform some modifications, it could probably handle    #
--#                              humanoid walkers too, though it isn't intended to.                               #               
--#                                                                                                               #
--#################################################################################################################

--Terminology note: "Vertical", "lateral", and "medial" (v, l, m) are offsets from the position of a leg's root spinblock, relative to the leg's orientation.

--For properly oriented legs, +v is always up, -v is always down. Lateral and medial depend on which side of the craft a leg is on, but should
--      follow the rule that +l is the same direction as the side of the craft the leg is on, and -l the opposite. That is, +l should be right and -l should be left
--      for a leg on the right side of the craft, and vice versa for the left. +m is the direction the leg goes if its hip spinblock rotates in its positive
--      direction (which should be clockwise if it's upright). You may want to ensure a positive rotation always means +m by adjusting the hip spinblock's global
--      response, to make your settings more readable.

--Basic diagram of a splayed-out leg as viewed from the side. 'O' indicates a lateral spinblock, '[' a vertical one, and '=' is a connecting segment.

--   Ankle   Knee     Hip                        
--     |       |       |                           
--     [O======O======O[                               
--      |             |                          
--    Foot          Root    

--Hip is attached to the craft, ankle to the sticky foot. Ankle ensures the foot is always pointing in the direction it is moving, foot spinblock ensures it is
--      always parallel to the ground. Hip, root, and knee joints combine to allow for general articulation.

config = {
    cycleDuration = 2;          --Amount of time, in seconds, it takes for a single walk cycle to pass.
    deltaTime = 1/40;           --Amount of time, in seconds, that passes each tick. Should be 1/40th of a second, the length of an FTD physics step.
    verticalDeltaCap = 1/40;    --Maximum amount total vertical response can change for any given leg in a single tick. Should be on range (0, 1].
    lateralDeltaCap = 1/40;     --Maximum amount total lateral response can change for any given leg in a single tick. Should be on range (0, 1].
    medialDeltaCap = 1/40;      --Maximum amount total medial response can change for any given leg in a single tick. Should be on range (0, 1].
}

--Add an unkeyed table for each leg on the craft into the following table, with these arguments in this order:

--name:                 Name of the leg. Used to look for the spinblocks that make it up, in the format "leg_<name>_<hip/root/knee/foot/ankle>".
--cycleOffset:          Offset of the leg's walk cycle from the base, so they aren't all trying to get off the ground at the same time. Should be on the range [0, 1).
--restPosition:         Table of offsets to determine where the foot's rest position is. {v, l, m}
--maxPosition:          Maximum offsets from rest for foot position in each axis. Each should be positive. {v, l, m}
--minPosition:          Minimum offsets from rest for foot position in each axis. Each should be negative. {v, l, m}
--mainResponse:         Response weight to main drive in each axis. Main should probably only use the medial axis unless you're doing something weird. {v, l, m}
--rollResponse:         Response weight to roll drive in each axis. Roll should probably only use the vertical axis unless you're doing something weird. {v, l, m}
--pitchResponse:        Response weight to pitch drive in each axis. Pitch should probably only use the vertical axis unless you're doing something weird. {v, l, m}
--yawResponse:          Response weight to yaw drive in each axis. Yaw should probably only use the lateral and medial axes unless you're doing something weird. {v, l, m}
--forwardResponse:      Response weight to forward drive in each axis. Forward should probably only use the medial axis unless you're doing something weird. {v, l, m}
--hoverResponse:        Response weight to hover drive in each axis. Hover should probably only use the vertical axis unless you're doing something weird. {v, l, m}
--strafeResponse:       Response weight to strafe drive in each axis. Strafe should probably only use the lateral axis unless you're doing something weird. {v, l, m}
--stepHeight:           How much the foot raises during a step. Should be positive.
--rootLength:           Length of the segment attached to the root joint spinblock, from the root joint spinblock to (and including) the knee joint spinblock.
--kneeLength:           Length of the segment attached to the knee joint spinblock, from the knee joint spinblock to (and including) the foot joint spinblock.

--The total response of a leg in a given axis equals the sum of the request in each drive multiplied by that leg's response to that drive in the given axis, then clamped 
--    to the range [-1, 1]. This number is then used as a coefficient to determine the direction and length of each step. Technically, a drive response greater than 1 or 
--    less than -1 is valid, though usually has little practical use.

legSettings = {
  --{     name, co, restp {v, l, m}, maxp {v, l, m}, minp {v, l, m}, mr {v, l, m}, rr {v, l, m}, pr {v, l, m}, yr {v, l, m},  fr {v, l, m}, hr {v, l, m}, sr {v, l, m}, sh, rl, kl};
  --{"example",  0,       {0, 0, 0},      {0, 0, 0},      {0, 0, 0},    {0, 0, 0},    {0, 0, 0},    {0, 0, 0},    {0, 0, 0},     {0, 0, 0},    {0, 0, 0},    {0, 0, 0},  0,  0,  0};
}



--inverseKinematics uses RADIANS, not degrees!
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
function legController.new(I, name, restPosition, maxPosition, minPosition, mainResponse, rollResponse, pitchResponse, yawResponse, forwardResponse, hoverResponse, strafeResponse, 
            stepHeight, rootLength, kneeLength)
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

    --ASSIGN LEG VARIABLES HERE!! DON'T FORGET TO DO THAT!

    leg.controller = legController.newThread(leg, I)
    table.insert(legController.legList, leg)

    setmetatable(leg, legController.mt)

    return leg
end

function legController.actionThread(leg, I)
    local cycleCounter, forwardRequest, yawRequest, mainRequest, walkRequest, pitchRequest, rollRequest, heightRequest, hipAngle, rootAngle, kneeAngle, footAngle, stepLength, heightModifier
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
