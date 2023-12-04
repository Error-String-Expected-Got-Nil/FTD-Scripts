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
    groundOffset = -4;          --Distance from leg root to ground. Should be negative.
    raiseOffset = -2;           --Distance from leg root to height of foot when it is lifted up during walk cycle. Should be greater than groundOffset.
    legReachOffset = 8;         --Distance from leg root to foot horizontally, away from the vehicle.
    stepAngle = 30;             --Farthest angle, in degrees, a leg should move away from its default angle while moving.
    cycleDuration = 2;          --Amount of time, in seconds, it takes for a single walk cycle to pass.
    deltaTime = 1/40;           --Amount of time, in seconds, that passes each tick. Should be 1/40th of a second, the length of an FTD physics step.
}

--Add an unkeyed table for each leg on the craft into the following table, with these arguments in this order:

--name:             Name of leg, looks for all spinblocks named leg_<name>_<hip/root/knee/foot/ankle> to determine the spinblocks apart of this leg.
--restAngle:        Angle (IN DEGREES) the hip should be when the leg is at rest.
--cycleOffset:      Offset to the cycle of the leg. Used so you can, say, have half the legs off the ground and the other half on the ground while in motion.
--forwardResponse:  Weight of forward drive request when determining the leg's response.
--yawResponse:      Weight of yaw drive request when determining the leg's response.
--mainResponse:     weight of main drive request when determining the leg's response.
--rootLength:       Length of the segment attached to the root spinblock of the leg (the one directly attached to the hip).
--kneeLength:       Length of the segment attached to the knee spinblock of the leg (the one attached to the root segment).
--reach:            How far out from the root spinblock should the foot be, horizontally.

legSettings = {
    {"test", 0, 0, 1, 0, 1, 5, 9, 8};
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
    local kneeAngle = Mathf.Acos((distance - midpoint) / kneeLength)
    local footAngle = kneeAngle - angleOffset

    rootAngle = rootAngle + angleOffset
    kneeAngle = kneeAngle + rootAngle

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
function legController.new(I, name, restAngle, cycleOffset, forwardResponse, yawResponse, mainResponse, rootLength, kneeLength, reach)
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
    leg.rootLength = rootLength
    leg.kneeLength = kneeLength
    leg.reach = reach

    leg.controller = legController.newThread(leg, I)
    table.insert(legController.legList, leg)

    setmetatable(leg, legController.mt)

    return leg
end

function legController.actionThread(leg, I)
    local cycleCounter, forwardRequest, yawRequest, mainRequest, totalRequest, hipAngle, rootAngle, kneeAngle, footAngle, stepLength
    stepLength = config.stepAngle
    
    while true do
        cycleCounter, forwardRequest, yawRequest, mainRequest = coroutine.yield()

        totalRequest = forwardRequest * leg.forwardResponse + yawRequest * leg.yawResponse + mainRequest * leg.mainResponse
        cycleCounter = (cycleCounter + leg.cycleOffset) % 1

        hipAngle, rootAngle, kneeAngle, footAngle = leg.restAngle, 0, 0, 0

        stepLength = config.stepAngle * Mathf.Clamp(totalRequest, -1, 1)

        if stepLength == 0 then
            hipAngle = leg.restAngle
            rootAngle, kneeAngle, footAngle = inverseKinematics.solveCoordinates(leg.reach, config.groundOffset, leg.rootLength, leg.kneeLength)
        else
            hipAngle = stepLength * Mathf.Sin(cycleCounter * pi2)
            local stepCoeff = Mathf.Cos(cycleCounter * pi2)    --Should be clamped to [0, 1] but Mathf.Lerp() does that for us already.
            local stepHeight = Mathf.Lerp(config.groundOffset, config.raiseOffset, stepCoeff)

            rootAngle, kneeAngle, footAngle = inverseKinematics.solveCoordinates(leg.reach / Mathf.Cos(hipAngle * rad), stepHeight, leg.rootLength, leg.kneeLength)
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
            legController.new(I, table.unpack(settings))
        end

        I:Log(startupLog)
        isStartup = false
    end

    for index, leg in ipairs(legController.legList) do
        coroutine.resume(leg.controller, cycleStopwatch, 0, 0, 0) 
    end

    cycleStopwatch = (cycleStopwatch + config.deltaTime / config.cycleDuration) % 1 
end