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
    defaultGroundOffset = -4;   --Distance from leg root to ground vertically. Should be negative.
    defaultRaiseOffset = -2;    --Distance from leg root to height of foot vertically when it is lifted up during walk cycle. Should be greater than groundOffset.
    defaultReach = 8;           --Distance from leg root to foot horizontally, away from the vehicle. Default if not set by individual leg.
    stepAngle = 30;             --Farthest angle, in degrees, a leg should move away from its default angle while moving.
    cycleDuration = 2;          --Amount of time, in seconds, it takes for a single walk cycle to pass.
    deltaTime = 1/40;           --Amount of time, in seconds, that passes each tick. Should be 1/40th of a second, the length of an FTD physics step.
    walkDriveDeltaCap = 1/40;   --Maximum amount a leg's walking drive response (main, forward, yaw) can change in a single tick. Exists to smooth out jerky motion.
    heightDriveDeltaCap = 1/80; --Maximum amount a leg's height drive response (pitch, roll) can change in a single tick. Exists to smooth out jerky motion.
}

--Add an unkeyed table for each leg on the craft into the following table, with these arguments in this order:

--name:             Name of leg, looks for all spinblocks named leg_<name>_<hip/root/knee/foot/ankle> to determine the spinblocks apart of this leg.
--restAngle:        Angle (IN DEGREES) the hip is offset from 0 while at rest. Also affects angles while in motion.
--cycleOffset:      Offset to the cycle of the leg. Used so you can, say, have half the legs off the ground and the other half on the ground while in motion.
--forwardResponse:  Weight of forward drive request when determining the leg's response.
--yawResponse:      Weight of yaw drive request when determining the leg's response.
--mainResponse:     Weight of main drive request when determining the leg's response.
--pitchResponse:    Weight of the pitch drive request when determining the leg's height response.
--rollResponse:     Weight of the roll drive request when determining the leg's height response.
--heightDeviation:  Maximum the base standing height can change by due to pitch/roll request.
--groundOffset:     Distance from leg root to ground vertically. Should be negative.
--raiseOffset:      Distance from leg root to its highest raised position during the walk cycle. Should be greater than groundOffset.
--rootLength:       Length of the segment attached to the root spinblock of the leg (the one directly attached to the hip).
--kneeLength:       Length of the segment attached to the knee spinblock of the leg (the one attached to the root segment).
--reach:            How far out from the root spinblock should the foot be, horizontally.

legSettings = {
  --{"name", ra, co, fr, yr, mr, pr, rr, hd, go, ro, rl, kl, re};
    {"test",  0,  0,  1,  0,  1,  0,  0,  0, -4, -2,  5,  9,  8};
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
function legController.new(I, name, restAngle, cycleOffset, forwardResponse, yawResponse, mainResponse, pitchResponse, rollResponse, heightDeviation, groundOffset, raiseOffset, rootLength, kneeLength, reach)
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

    leg.restAngle = restAngle
    leg.cycleOffset = cycleOffset
    leg.forwardResponse = forwardResponse
    leg.yawResponse = yawResponse
    leg.mainResponse = mainResponse
    leg.pitchResponse = pitchResponse
    leg.rollResponse = rollResponse
    leg.heightDeviation = heightDeviation
    leg.groundOffset = groundOffset or config.defaultGroundOffset
    leg.raiseOffset = raiseOffset or config.defaultRaiseOffset
    leg.rootLength = rootLength
    leg.kneeLength = kneeLength
    leg.reach = reach or config.defaultReach

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

        walkRequest = forwardRequest * leg.forwardResponse + yawRequest * leg.yawResponse + mainRequest * leg.mainResponse
        heightRequest = pitchRequest * leg.pitchResponse + rollRequest * leg.rollResponse

        walkResponse = walkResponse + Mathf.Clamp(walkRequest - walkResponse, -config.walkDriveDeltaCap, config.walkDriveDeltaCap)
        heightResponse = heightResponse + Mathf.Clamp(heightRequest - heightResponse, -config.heightDriveDeltaCap, config.heightDriveDeltaCap)

        heightModifier = Mathf.Clamp(heightResponse, -1, 1) * leg.heightDeviation
        stepLength = config.stepAngle * Mathf.Clamp(walkResponse, -1, 1)

        cycleCounter = (cycleCounter + leg.cycleOffset) % 1

        rootAngle, kneeAngle, footAngle = 0, 0, 0

        if stepLength == 0 then
            hipAngle = leg.restAngle
            rootAngle, kneeAngle, footAngle = inverseKinematics.solveCoordinates(leg.reach, leg.groundOffset + heightModifier, leg.rootLength, leg.kneeLength)
        else
            hipAngle = stepLength * Mathf.Sin(cycleCounter * pi2) + leg.restAngle
            local stepCoeff = Mathf.Cos(cycleCounter * pi2)    --Should be clamped to [0, 1] but Mathf.Lerp() does that for us already.
            local stepHeight = Mathf.Lerp(leg.groundOffset, leg.raiseOffset, stepCoeff)

            rootAngle, kneeAngle, footAngle = inverseKinematics.solveCoordinates(leg.reach / Mathf.Cos(hipAngle * rad), stepHeight + heightModifier, leg.rootLength, leg.kneeLength)
        end

        --Convert from radians to degrees:
        rootAngle, kneeAngle, footAngle = rootAngle * deg, kneeAngle * deg, footAngle * deg
        ankleAngle = -hipAngle

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