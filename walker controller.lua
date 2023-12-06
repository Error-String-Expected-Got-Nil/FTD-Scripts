deg = Mathf.Rad2Deg
rad = Mathf.Deg2Rad
pi2 = Mathf.PI * 2
startupLog = ""
function logBuffer(str) startupLog = startupLog .. str .. "\n" end
function vlm(tab) return {v = tab[1], l = tab[2], m = tab[3]} end

--#######################################################################################################
--#                                                                                                     #
--#   This is a general-purpose walker leg controller designed to work with 5-jointed insectoid legs.   # 
--#   It is highly configurable, and should be able to control the vast majority of walkers, even if    #  
--#                                 they are using an unusual layout.                                   #
--#                                                                                                     #
--#                             By Error: String Expected, Got Nil (Esegn)                              #
--#                                                                                                     #
--#######################################################################################################

--Terminology note: "Vertical", "lateral", and "medial" (v, l, m) are offsets from the position of a leg's root spinblock, relative to the leg's orientation. +v is up from the root,
--      -v is down, +l is forward, -l is backward, +m is away, and -m is toward. Note that the lateral direction can be reversed for a leg by making its hip spinblock rotate in
--       reverse, and it is recommended you adjust your legs so that +l/-l is forward/backward relative to the craft (though not required).

--Basic diagram of a splayed-out leg as viewed from the side. 'O' indicates a lateral spinblock, 'I' a vertical one, and '=' is a connecting segment.

--   Ankle   Knee     Hip                       
--     |       |       |                          
--     IO======O======OI                                  
--      |             |                         
--    Foot          Root       
--

--Hip is attached to the craft, ankle to the sticky foot. Ankle ensures the foot is always pointing in the direction it is moving, foot spinblock ensures it is
--      always parallel to the ground. Hip, root, and knee joints combine to allow for general articulation.

--Root, knee, and foot spinblocks should be oriented such that their segment moves upwards when they rotate clockwise. You can also put them in reverse mode for the same effect if 
--      you accidentally built them wrong.

--All leg joint spinblocks should be in "rotate at a speed determined by rate controller" mode, with spin rate set to maximum.

config = {
    cycleDuration = 2;          --Amount of time, in seconds, it takes for a single walk cycle to complete.
    deltaTime = 1/40;           --Amount of time, in seconds, that passes each tick. Should be 1/40th of a second, the length of an FTD physics step.
    verticalDeltaCap = 1/40;    --Maximum amount total vertical response can change for any given leg in a single tick. Should be on the range (0, 1].
    lateralDeltaCap = 1/40;     --Maximum amount total lateral response can change for any given leg in a single tick. Should be on the range (0, 1].
    medialDeltaCap = 1/40;      --Maximum amount total medial response can change for any given leg in a single tick. Should be on the range (0, 1].
}

--Add an unkeyed table for each leg on the craft into the following table, with these arguments in this order:

--name:                 Custom name set to to the ankle spinblock of the leg. The program finds this spinblock, then checks subobject parents upwards to find the foot, knee, root, and hip.
--cycleOffset:          Offset of the leg's walk cycle from the base, so they aren't all trying to get off the ground at the same time. Should be on the range [0, 1).
--restPosition:         Table of offsets to determine where the foot's rest position is. {v, l, m}
--maxPosition:          Maximum offsets from rest position for foot position in each axis, based on drive request. Each should be positive. {v, l, m}
--minPosition:          Minimum offsets from rest position for foot position in each axis, based on drive request. Each should be negative. {v, l, m}
--stepHeight:           How much the foot raises from the ground during a step. Should be positive.
--mainResponse:         Response weight to main drive in each axis. Main should probably only use the lateral axis unless you're doing something weird. {v, l, m}
--rollResponse:         Response weight to roll drive in each axis. Roll should probably only use the vertical axis unless you're doing something weird. {v, l, m}
--pitchResponse:        Response weight to pitch drive in each axis. Pitch should probably only use the vertical axis unless you're doing something weird. {v, l, m}
--yawResponse:          Response weight to yaw drive in each axis. Yaw should probably only use the lateral and medial axes unless you're doing something weird. {v, l, m}
--forwardResponse:      Response weight to forward drive in each axis. Forward should probably only use the lateral axis unless you're doing something weird. {v, l, m}
--hoverResponse:        Response weight to hover drive in each axis. Hover should probably only use the vertical axis unless you're doing something weird. {v, l, m}
--strafeResponse:       Response weight to strafe drive in each axis. Strafe should probably only use the medial axis unless you're doing something weird. {v, l, m}

--The total response of a leg in a given axis equals the sum of the request in each drive multiplied by that leg's response to that drive in the given axis, then clamped 
--    to the range [-1, 1]. This number is then used as a coefficient to determine the direction and length of each step, where 0 means no movement, 1/-1 means a full stride,
--    positive means steps pull the craft in that direction, and negative means steps push the craft away from that direction.

--IMPORTANT NOTE: For vertical offsets, a positive value means the foot will raise more, which will cause the craft to go *down* more. 
--    Slightly unintuitive, but will be kept this way for the sake of consistency.

legSettings = {
  --{     name, co, restp {v, l, m}, maxp {v, l, m}, minp {v, l, m}, sh, mr {v, l, m}, rr {v, l, m}, pr {v, l, m}, yr {v, l, m},  fr {v, l, m}, hr {v, l, m}, sr {v, l, m}};
  --{"example",  0,       {0, 0, 0},      {0, 0, 0},      {0, 0, 0},  0,    {0, 0, 0},    {0, 0, 0},    {0, 0, 0},    {0, 0, 0},     {0, 0, 0},    {0, 0, 0},    {0, 0, 0}};
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
function legController.new(I, name, cycleOffset, restPosition, maxPosition, minPosition, stepHeight, mainResponse, rollResponse, pitchResponse, yawResponse, forwardResponse, 
            hoverResponse, strafeResponse)
    local leg = {}

    leg.name = name

    leg.ankleID = legController.spinblockList[name]
    if not leg.ankleID then logBuffer("[ERROR] Failed to find ankle spinblock for leg \"" .. name .. "\"!") end
    leg.footID = I:GetParent(leg.ankleID)
    if not leg.footID then logBuffer("[ERROR] Failed to find foot spinblock for leg \"" .. name .. "\"!") end
    leg.kneeID = I:GetParent(leg.footID)
    if not leg.kneeID then logBuffer("[ERROR] Failed to find knee spinblock for leg \"" .. name .. "\"!") end
    leg.rootID = I:GetParent(leg.kneeID)
    if not leg.rootID then logBuffer("[ERROR] Failed to find root spinblock for leg \"" .. name .. "\"!") end
    leg.hipID = I:GetParent(leg.rootID)
    if not leg.hipID then logBuffer("[ERROR] Failed to find hip spinblock for leg \"" .. name .. "\"!") end
    
    leg.rootLength = I:GetSubConstructInfo(kneeID).LocalPosition.magnitude
    leg.kneeLength = I:GetSubConstructInfo(footID).LocalPosition.magnitude

    leg.cycleOffset = cycleOffset

    leg.restPosition = vlm(restPosition)
    leg.maxPosition = vlm(maxPosition)
    leg.minPosition = vlm(minPosition)
    leg.stepHeight = stepHeight

    leg.mainResponse = vlm(mainResponse)
    leg.rollResponse = vlm(mainResponse)
    leg.pitchResponse = vlm(mainResponse)
    leg.yawResponse = vlm(mainResponse)
    leg.forwardResponse = vlm(mainResponse)
    leg.hoverResponse = vlm(mainResponse)
    leg.strafeResponse = vlm(mainResponse)

    leg.controller = legController.newThread(leg, I)
    table.insert(legController.legList, leg)

    setmetatable(leg, legController.mt)

    return leg
end

function legController.actionThread(leg, I)
    local verticalResponse, lateralResponse, medialResponse = 0, 0, 0
    
    while true do
        local cycle, mainRequest, rollRequest, pitchRequest, yawRequest, forwardRequest, hoverRequest, strafeRequest = coroutine.yield()

        cycle = (cycle + leg.cycleOffset) % 1

        local verticalRequest = Mathf.Clamp(mainRequest * leg.mainResponse.v + rollRequest * leg.rollResponse.v + pitchRequest * leg.pitchResponse.v + yawRequest * leg.yawResponse.v
                + forwardRequest * leg.forwardResponse.v + hoverRequest * leg.hoverResponse.v + strafeRequest * leg.strafeResponse.v, -1, 1)
        
        local lateralRequest = Mathf.Clamp(mainRequest * leg.mainResponse.l + rollRequest * leg.rollResponse.l + pitchRequest * leg.pitchResponse.l + yawRequest * leg.yawResponse.l
                + forwardRequest * leg.forwardResponse.l + hoverRequest * leg.hoverResponse.l + strafeRequest * leg.strafeResponse.l, -1, 1)
        
        local medialRequest = Mathf.Clamp(mainRequest * leg.mainResponse.m + rollRequest * leg.rollResponse.m + pitchRequest * leg.pitchResponse.m + yawRequest * leg.yawResponse.m
                + forwardRequest * leg.forwardResponse.m + hoverRequest * leg.hoverResponse.m + strafeRequest * leg.strafeResponse.m, -1, 1)

        verticalResponse = verticalResponse + Mathf.Clamp(verticalRequest - verticalResponse, -config.verticalDeltaCap, config.verticalDeltaCap)
        lateralResponse = lateralResponse + Mathf.Clamp(lateralRequest - lateralResponse, -config.lateralDeltaCap, config.lateralDeltaCap)
        medialResponse = medialResponse + Mathf.Clamp(medialRequest - medialResponse, -config.medialDeltaCap, config.medialDeltaCap)

        local footPosition, ankleAngle
        if lateralResponse + medialResponse == 0 then
            footPosition = Vector3(leg.restPosition.v + legController.getVerticalOffset(leg, verticalResponse), leg.restPosition.l, leg.restPosition.m)
            ankleAngle = nil
        else
            --Vector3 will be used to handle step positions; x is vertical, y is lateral, z is medial
            local stepMax, stepMin = legController.getSteps(leg, verticalResponse, lateralResponse, medialResponse)

            --The position of the foot, discounting step height, at the current point in the cycle.
            footPosition = Vector3.Lerp(stepMin, stepMax, (Mathf.Sin(cycle * pi2) + 1) / 2)

            --Add the step height of the foot at the current point in the cycle.
            footPosition.x = footPosition.x + leg.stepHeight * Mathf.Clamp(Mathf.Cos(cycle * pi2), 0, 1)

            --Set ankle to point in direction of movement.
            ankleAngle = deg * Mathf.Atan2(stepMax.y - stepMin.y, stepMax.z - stepMin.z)
            --But make sure it faces forward if we're walking backwards
            if lateralResponse < 0 then ankleAngle = ankleAngle + 180 end
        end

        --Make hip point leg towards the target point, then solve inverse kinematics using horizontal distance from root to desired foot position and the height of the desired position.
        local hipAngle = deg * Mathf.Atan2(footPosition.y, footPosition.z)
        local rootAngle, kneeAngle, footAngle = inverseKinematics.solveCoordinates((footPosition.y ^ 2 + footPosition.z ^ 2) ^ 0.5, footPosition.x, leg.rootLength, leg.kneeLength)
        rootAngle, kneeAngle, footAngle = rootAngle * deg, kneeAngle * deg, footAngle * deg

        --If the leg is at rest, stepMax and stepMin weren't calculated, so we can't used that for the angle of the ankle. Instead we just set it correct for hip angle.
        if ankleAngle == nil then
            ankleAngle = -hipAngle
        end

        I:SetSpinBlockRotationAngle(leg.hipID, hipAngle)
        I:SetSpinBlockRotationAngle(leg.rootID, rootAngle)
        I:SetSpinBlockRotationAngle(leg.kneeID, kneeAngle)
        I:SetSpinBlockRotationAngle(leg.footID, footAngle)
        I:SetSpinBlockRotationAngle(leg.ankleID, ankleAngle)
    end
end

function legController.newThread(leg, I)
    local thread = coroutine.create(legController.actionThread)
    coroutine.resume(thread, leg, I)
    return thread
end

function legController.getVerticalOffset(leg, vr)
    local vertical = leg.restPosition.v

    if vr < 0 then
        vertical = vertical + leg.minPosition.v * Mathf.Abs(vr)
    else
        vertical = vertical + leg.maxPosition.v * Mathf.Abs(vr)
    end

    return vertical
end

--Returns the maximum step and the minimum step. Swaps lateral and medial coordinates as necessary to make steps go in the right direction when lerped.
function legController.getSteps(leg, vr, lr, mr)
    local vertical = legController.getVerticalOffset(leg, vr)

    local lmax, lmin = leg.restPosition.l + leg.maxPosition.l * Mathf.Abs(lr), leg.restPosition.l + leg.minPosition.l * Mathf.Abs(lr)
    if lr < 0 then lmax, lmin = lmin, lmax end

    local mmax, mmin = leg.restPosition.m + leg.maxPosition.m * Mathf.Abs(mr), leg.restPosition.m + leg.minPosition.m * Mathf.Abs(mr)
    if mr < 0 then mmax, mmin = mmin, mmax end

    return Vector3(vertical, lmax, mmax), Vector3(vertical, lmin, mmin)
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
    local rollDrive = I:GetPropulsionRequest(3)
    local pitchDrive = I:GetPropulsionRequest(4)
    local yawDrive = I:GetPropulsionRequest(5)
    local forwardDrive = I:GetPropulsionRequest(6)
    local hoverDrive = I:GetPropulsionRequest(7)
    local strafeDrive = I:GetPropulsionRequest(8)
    

    for index, leg in ipairs(legController.legList) do
        coroutine.resume(leg.controller, cycleStopwatch, mainDrive, rollDrive, pitchDrive, yawDrive, forwardDrive, hoverDrive, strafeDrive) 
    end

    cycleStopwatch = (cycleStopwatch + config.deltaTime / config.cycleDuration) % 1 
end
