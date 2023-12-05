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

        rootAngle, kneeAngle, footAngle = rootAngle * deg, kneeAngle * deg, footAngle * deg
        ankleAngle = -hipAngle

        I:SetSpinBlockRotationAngle(leg.hipID, hipAngle)
        I:SetSpinBlockRotationAngle(leg.rootID, rootAngle)
        I:SetSpinBlockRotationAngle(leg.kneeID, kneeAngle)
        I:SetSpinBlockRotationAngle(leg.footID, footAngle)
        I:SetSpinBlockRotationAngle(leg.ankleID, ankleAngle)
    end
end