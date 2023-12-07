--TODO:
--  Ground height detection and adjustment
--      config.automaticGroundSensing = true/false --Should each leg automatically adjust foot position based on ground level? 
--      config.automaticRestHeight                 --If yes, how high should the craft rest?
--                                                 --A leg's resting vertical axis is now an offset from this, and should usually be 0.
--  Make adaptive footing attempt to keep the craft level?

--ACCOUNT FOR INNATE ANGLE OFFSET IN rootAngle AND kneeAngle DUE TO PLACEMENT! THEY MIGHT NOT BE PERFECTLY ALIGNED IN A STRAIGHT LINE
--NEED TO FIX THAT BEFORE USING ON AN ACTUAL CRAFT
--Also need to determine whether VLM is actually relative to root or hip position as-coded and fix it to root position if it's wrong

Deg = Mathf.Rad2Deg
Rad = Mathf.Deg2Rad
Pi2 = Mathf.PI * 2
StartupLog = ""
function LogBuffer(str) StartupLog = StartupLog .. str .. "\n" end
function VLM(tab, tableName, name)
    if not tab or not tab[1] or not tab[2] or not tab[3] then
        LogBuffer("[WARN] One or more " .. tableName .. " settings for leg \"" .. name .. "\" were nil. Defaulting to {v = 0, l = 0, m = 0}.")
        return {v = 0, l = 0, m = 0}
    else
        return {v = tab[1], l = tab[2], m = tab[3]}
    end
end

--#######################################################################################################
--#                                                                                                     #
--#   This is a general-purpose walker leg controller designed to work with 5-jointed insectoid legs.   # 
--#   It is highly configurable, and should be able to control the vast majority of walkers, even if    #  
--#                                 they are using an unusual layout.                                   #
--#                                                                                                     #
--#                             By Error: String Expected, Got Nil (Esegn)                              #
--#                                                                                                     #
--#######################################################################################################

--Terminology note: "vertical", "lateral", and "medial" (v, l, m) are offsets from the position of a leg's root spinblock, relative to the leg's orientation. +v is up from the root,
--      -v is down, +l is forward, -l is backward, +m is away, and -m is toward. Note that the lateral direction can be reversed for a leg by setting its hip spinblock to power 
--      scale -1, and it is recommended you adjust your legs so that +l/-l is forward/backward relative to the craft (though not required; this just makes configuring them easier).

--Basic diagram of a splayed-out leg as viewed from the side. 'O' indicates a lateral spinblock, 'I' a vertical one, and '=' is a connecting segment.

--   Ankle   Knee     Hip                       
--     |       |       |                          
--     IO======O======OI                                  
--      |             |                         
--    Foot          Root       
--

--Hip is attached to the craft, ankle to the sticky foot. Ankle ensures the foot is always pointing in the direction it is moving, foot spinblock ensures it is
--      always parallel to the ground. Hip, root, and knee joints combine to allow for general articulation.

--Root, knee, and foot spinblocks should be oriented such that their segment moves upwards when they rotate clockwise. You can also set their power scale to -1 to reverse their response if
--      you accidentally built them wrong.

--All leg joint spinblocks should be in "rotate at a speed determined by rate controller" mode, with spin rate set to maximum.

Config = {
    cycleDuration = 1;          --Amount of time, in seconds, it takes for a single walk cycle to complete.
    verticalDeltaCap = 4;       --Maximum amount total vertical response can change for any given leg per second.
    lateralDeltaCap = 1;        --Maximum amount total lateral response can change for any given leg per second.
    medialDeltaCap = 1;         --Maximum amount total medial response can change for any given leg per second.
    restDriveThreshold = 0.02;  --When the absolute value of lateral and medial response are both less than this value, a leg should be considered at rest and cease movement.
    showHUDDebugInfo = false;   --Shows some debugging information on the HUD if true.
    stepPulseOnChannel = 1;     --Which drive index to output on when a leg sends a "about to touch ground" synchronization pulse.
    stepPulseOffChannel = 2;    --Which drive index to output on when a leg sends a "about to leave ground" synchronization pulse.
    adaptiveFooting = true;     --If true, legs will attempt to adapt to craft rotation and terrain height to maintain level walking.
    heightAdaptationRate = 1;   --For adaptive footing, when the ground height under the craft changes, do not immediately change the height used for adaptive footing, instead change
                                --    it by this much per second. Makes movement smoother.
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

--The total response of a leg in a given axis equals the sum of each drive's request multiplied by the leg's response to that drive in the given axis, then clamped to the 
--      range [-1, 1]. This number is then used as a coefficient to determine the direction and length of each step, where 0 means no movement, 1/-1 means a full stride,
--      positive means steps pull the craft in that direction, and negative means steps push the craft away from that direction.

--IMPORTANT NOTE: For vertical offsets, a positive value means the foot will raise more, which will cause the craft to go *down* more. 
--      Slightly unintuitive, but will be kept this way for the sake of consistency.

LegSettings = {
  --{     name, co, restp {v, l, m}, maxp {v, l, m}, minp {v, l, m}, sh, mr {v, l, m}, rr {v, l, m}, pr {v, l, m}, yr {v, l, m},  fr {v, l, m}, hr {v, l, m}, sr {v, l, m}};
  --{"example",  0,       {0, 0, 0},      {0, 0, 0},      {0, 0, 0},  0,    {0, 0, 0},    {0, 0, 0},    {0, 0, 0},    {0, 0, 0},     {0, 0, 0},    {0, 0, 0},    {0, 0, 0}};
    {   "test",  0,      {-4, 1, 8},      {1, 6, 2}, {-1, -5.5, -2},  2,    {0, 1, 0},    {0, 0, 0},    {0, 0, 0},   {0, 0, -1},     {0, 0, 0},    {0, 0, 0},    {0, 0, 0}};
}

--In the format [offset] = pulse strength:
--When legs with the cycle offset 'offset' are about to touch the ground, output on the stepPulseOnChannel at pulse strength.
--When about to leave the ground, output on the stepPulseOffChannel at pulse strength.
--Intended to help synchronize clampy feet on legs, but could be used for other things.
StepSyncrhonizationPulses = {
  --[0] = 1;
    [0] = 0.95;
}

StepSynchronizationRequestSecondary = 0;
StepSynchronizationRequestTertiary = 0;



--InverseKinematics uses RADIANS, not degrees!
InverseKinematics = {}

function InverseKinematics.solveExtension(rootLength, kneeLength, distance, angleOffset)
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

function InverseKinematics.solveCoordinates(x, y, rootLength, kneeLength)
    local angleOffset = Mathf.Atan2(y, x)
    local distance = (x ^ 2 + y ^ 2) ^ 0.5

    return InverseKinematics.solveExtension(rootLength, kneeLength, distance, angleOffset)
end



LegController = {}
LegController.spinblockList = {}
LegController.legList = {}

--I: The "I" variable from the Update() function.
--For other arguments, see the settings table at the top of the program.
function LegController.new(I, name, cycleOffset, restPosition, maxPosition, minPosition, stepHeight, mainResponse, rollResponse, pitchResponse, yawResponse, forwardResponse,
            hoverResponse, strafeResponse)
    local leg = {}

    LogBuffer("Creating new leg \"" .. name .. "\"")

    leg.name = name

    leg.ankleID = LegController.spinblockList[name]
    if not leg.ankleID then LogBuffer("[WARN] Failed to find ankle spinblock for leg \"" .. name .. "\"! Cannot construct leg, discarding it.") return end
    leg.footID = I:GetParent(leg.ankleID)
    if not leg.footID then I:Log("[ERROR] Failed to find foot spinblock for leg \"" .. name .. "\"!") end
    leg.kneeID = I:GetParent(leg.footID)
    if not leg.kneeID then I:Log("[ERROR] Failed to find knee spinblock for leg \"" .. name .. "\"!") end
    leg.rootID = I:GetParent(leg.kneeID)
    if not leg.rootID then I:Log("[ERROR] Failed to find root spinblock for leg \"" .. name .. "\"!") end
    leg.hipID = I:GetParent(leg.rootID)
    if not leg.hipID then I:Log("[ERROR] Failed to find hip spinblock for leg \"" .. name .. "\"!") end

    leg.rootLength = I:GetSubConstructInfo(leg.kneeID).LocalPosition.magnitude
    leg.kneeLength = I:GetSubConstructInfo(leg.footID).LocalPosition.magnitude

    leg.cycleOffset = cycleOffset

    leg.restPosition = VLM(restPosition, "rest position", name)
    leg.maxPosition = VLM(maxPosition, "max position", name)
    leg.minPosition = VLM(minPosition, "min position", name)
    leg.stepHeight = stepHeight

    leg.mainResponse = VLM(mainResponse, "main response", name)
    leg.rollResponse = VLM(rollResponse, "roll response", name)
    leg.pitchResponse = VLM(pitchResponse, "pitch response", name)
    leg.yawResponse = VLM(yawResponse, "yaw response", name)
    leg.forwardResponse = VLM(forwardResponse, "forward response", name)
    leg.hoverResponse = VLM(hoverResponse, "hover response", name)
    leg.strafeResponse = VLM(strafeResponse, "strafe response", name)

    leg.controller = LegController.newThread(leg, I)
    table.insert(LegController.legList, leg)

    return leg
end

function LegController.actionThread(leg, I)
    local verticalResponse, lateralResponse, medialResponse = 0, 0, 0
    local currentStepHeight = 0
    local craftGroundAltitudeTracker = I:GetTerrainAltitudeForPosition(I:GetConstructPosition())

    while true do
        local cycle, mainRequest, rollRequest, pitchRequest, yawRequest, forwardRequest, hoverRequest, strafeRequest = coroutine.yield()

        cycle = (cycle + leg.cycleOffset) % 1

        local verticalRequest = Mathf.Clamp(mainRequest * leg.mainResponse.v + rollRequest * leg.rollResponse.v + pitchRequest * leg.pitchResponse.v + yawRequest * leg.yawResponse.v
                + forwardRequest * leg.forwardResponse.v + hoverRequest * leg.hoverResponse.v + strafeRequest * leg.strafeResponse.v, -1, 1)

        local lateralRequest = Mathf.Clamp(mainRequest * leg.mainResponse.l + rollRequest * leg.rollResponse.l + pitchRequest * leg.pitchResponse.l + yawRequest * leg.yawResponse.l
                + forwardRequest * leg.forwardResponse.l + hoverRequest * leg.hoverResponse.l + strafeRequest * leg.strafeResponse.l, -1, 1)

        local medialRequest = Mathf.Clamp(mainRequest * leg.mainResponse.m + rollRequest * leg.rollResponse.m + pitchRequest * leg.pitchResponse.m + yawRequest * leg.yawResponse.m
                + forwardRequest * leg.forwardResponse.m + hoverRequest * leg.hoverResponse.m + strafeRequest * leg.strafeResponse.m, -1, 1)

        local vDeltaCap, lDeltaCap, mDeltaCap = Config.verticalDeltaCap * DeltaTime, Config.lateralDeltaCap * DeltaTime, Config.medialDeltaCap * DeltaTime

        verticalResponse = verticalResponse + Mathf.Clamp(verticalRequest - verticalResponse, -vDeltaCap, vDeltaCap)
        lateralResponse = lateralResponse + Mathf.Clamp(lateralRequest - lateralResponse, -lDeltaCap, lDeltaCap)
        medialResponse = medialResponse + Mathf.Clamp(medialRequest - medialResponse, -mDeltaCap, mDeltaCap)

        if Config.showHUDDebugInfo then I:LogToHud("name: " .. leg.name .. "\nvr: " .. verticalResponse .. "\nlr: " .. lateralResponse .. "\nmr: " .. medialResponse) end

        local adaptiveFootingModifier = 0
        if Config.adaptiveFooting then do
            local craftPosition = I:GetConstructPosition()
            local craftGroundAltitude = I:GetTerrainAltitudeForPosition(craftPosition)
            --Technically the below uses the foot's *current* world position, not the desired world position, as calculating that from VLM position would be hellish.
            --I'm figuring it's probably fine since it should correct itself in a tick or two, and only be wrong by a little bit. I think. If I don't fix this, I was right. 
            local footGroundAltitude = I:GetTerrainAltitudeForPosition(I:GetSubConstructInfo(leg.ankleID).Position)
            local rootPosition = I:GetSubConstructInfo(leg.rootID).Position

            local heightDeltaLimit = Config.heightAdaptationRate * DeltaTime
            craftGroundAltitudeTracker = craftGroundAltitudeTracker + Mathf.Clamp(craftGroundAltitude - craftGroundAltitudeTracker, -heightDeltaLimit, heightDeltaLimit)

            local verticalOffset = craftGroundAltitudeTracker - footGroundAltitude + craftPosition.y - rootPosition.y

            adaptiveFootingModifier = -verticalOffset
        end end

        local footPosition
        local ankleAngle = 0

        local shouldSynchronizeSteps = false
        if Mathf.Abs(lateralResponse) < Config.restDriveThreshold and Mathf.Abs(medialResponse) < Config.restDriveThreshold then
            --Smooth out step height reduction if drives were stopped in the middle of a step.
            currentStepHeight = currentStepHeight + Mathf.Clamp(-currentStepHeight, -Config.verticalDeltaCap * leg.stepHeight, Config.verticalDeltaCap * leg.stepHeight)

            footPosition = Vector3(0, leg.restPosition.l, leg.restPosition.m)
            footPosition.x = LegController.getVerticalOffset(leg, verticalResponse)

            footPosition.x = footPosition.x + adaptiveFootingModifier + currentStepHeight
        else
            --Vector3 will be used to handle step positions; x is vertical, y is lateral, z is medial
            local stepMax, stepMin = LegController.getSteps(leg, verticalResponse, lateralResponse, medialResponse)

            --The position of the foot, discounting step height, at the current point in the cycle.
            footPosition = Vector3.Lerp(stepMin, stepMax, (Mathf.Sin(cycle * Pi2) + 1) / 2)

            --Handle adaptive footing.
            footPosition.x = footPosition.x + adaptiveFootingModifier

            --Add the step height of the foot at the current point in the cycle.
            currentStepHeight = leg.stepHeight * Mathf.Clamp(Mathf.Cos(cycle * Pi2), 0, 1)
            footPosition.x = footPosition.x + currentStepHeight

            shouldSynchronizeSteps = true
        end

        --Make sure leg doesn't try to extend farther than it's able to.
        footPosition = Vector3.ClampMagnitude(footPosition, leg.rootLength + leg.kneeLength)

        --Make hip point leg towards the target point, then solve inverse kinematics using horizontal distance from root to desired foot position and the height of the desired position.
        local hipAngle = Deg * Mathf.Atan2(footPosition.y, footPosition.z)
        local rootAngle, kneeAngle, footAngle = InverseKinematics.solveCoordinates((footPosition.y ^ 2 + footPosition.z ^ 2) ^ 0.5, footPosition.x, leg.rootLength, leg.kneeLength)
        rootAngle, kneeAngle, footAngle = rootAngle * Deg, kneeAngle * Deg, footAngle * Deg

        ankleAngle = ankleAngle - hipAngle

        I:SetSpinBlockRotationAngle(leg.hipID, hipAngle)
        I:SetSpinBlockRotationAngle(leg.rootID, rootAngle)
        I:SetSpinBlockRotationAngle(leg.kneeID, kneeAngle)
        I:SetSpinBlockRotationAngle(leg.footID, footAngle)
        I:SetSpinBlockRotationAngle(leg.ankleID, ankleAngle)

        if StepSyncrhonizationPulses[leg.cycleOffset] and shouldSynchronizeSteps then
            if StepSynchronizationRequestSecondary == 0 and cycle > 0.2 and cycle < 0.25 then
                StepSynchronizationRequestSecondary = StepSyncrhonizationPulses[leg.cycleOffset]
            elseif StepSynchronizationRequestTertiary == 0 and cycle > 0.75 and cycle < 0.8 then
                StepSynchronizationRequestTertiary = StepSyncrhonizationPulses[leg.cycleOffset]
            end
        end
    end
end

function LegController.newThread(leg, I)
    local thread = coroutine.create(LegController.actionThread)
    coroutine.resume(thread, leg, I)
    return thread
end

function LegController.getVerticalOffset(leg, vr)
    local vertical = leg.restPosition.v

    if vr < 0 then
        vertical = vertical + leg.minPosition.v * Mathf.Abs(vr)
    else
        vertical = vertical + leg.maxPosition.v * Mathf.Abs(vr)
    end

    return vertical
end

--Returns the maximum step and the minimum step. Swaps lateral and medial coordinates as necessary to make steps go in the right direction when lerped.
function LegController.getSteps(leg, vr, lr, mr)
    local vertical = LegController.getVerticalOffset(leg, vr)

    local lmax, lmin = leg.restPosition.l + leg.maxPosition.l * Mathf.Abs(lr), leg.restPosition.l + leg.minPosition.l * Mathf.Abs(lr)
    if lr < 0 then lmax, lmin = lmin, lmax end

    local mmax, mmin = leg.restPosition.m + leg.maxPosition.m * Mathf.Abs(mr), leg.restPosition.m + leg.minPosition.m * Mathf.Abs(mr)
    if mr < 0 then mmax, mmin = mmin, mmax end

    return Vector3(vertical, lmax, mmax), Vector3(vertical, lmin, mmin)
end



function IndexNamedSpinblocks(I)
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

IsStartup = true
Spinblocks = {}
CycleStopwatch = 0
PreviousTime = 0
Time = 0
DeltaTime = 0

function Update(I)
    if IsStartup then
        I:ClearLogs()

        Spinblocks = IndexNamedSpinblocks(I)

        LogBuffer("Detected the following named spinblocks:")
        for k, v in pairs(Spinblocks) do LogBuffer(k .. " (" .. v .. ")") end

        LegController.spinblockList = Spinblocks

        for index, settings in ipairs(LegSettings) do
            LegController.new(I, unpack(settings))
        end

        Time = I:GetTime()
        PreviousTime = Time

        I:Log(StartupLog)
        IsStartup = false
    end

    local mainDrive = I:GetPropulsionRequest(0)
    local rollDrive = I:GetPropulsionRequest(3)
    local pitchDrive = I:GetPropulsionRequest(4)
    local yawDrive = I:GetPropulsionRequest(5)
    local forwardDrive = I:GetPropulsionRequest(6)
    local hoverDrive = I:GetPropulsionRequest(7)
    local strafeDrive = I:GetPropulsionRequest(8)

    for index, leg in ipairs(LegController.legList) do
        local resumeSuccess, message = coroutine.resume(leg.controller, CycleStopwatch, mainDrive, rollDrive, pitchDrive, yawDrive, forwardDrive, hoverDrive, strafeDrive)

        if not resumeSuccess and message ~= "cannot resume dead coroutine" then I:Log(message) end
    end

    Time = I:GetTime()
    DeltaTime = Time - PreviousTime

    CycleStopwatch = (CycleStopwatch + DeltaTime / Config.cycleDuration) % 1

    I:SetPropulsionRequest(Config.stepPulseOnChannel, StepSynchronizationRequestSecondary)
    I:SetPropulsionRequest(Config.stepPulseOffChannel, StepSynchronizationRequestTertiary)

    StepSynchronizationRequestSecondary = 0
    StepSynchronizationRequestTertiary = 0

    PreviousTime = Time
end
