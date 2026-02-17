--[[
This file is part of Courseplay (https://github.com/Courseplay/Courseplay_FS25)
Copyright (C) 2025

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Drive strategy for driving to a loader/filler vehicle (slurry tanker) and refilling.
Uses pathfinder approach similar to combine unloader strategy.

]]--

---@class AIDriveStrategyRefillAtLoader : AIDriveStrategyCourse
---@field job CpAIJobFieldWork
AIDriveStrategyRefillAtLoader = CpObject(AIDriveStrategyCourse)

-- Target offset similar to unloader: don't end right at discharge node, leave some distance
AIDriveStrategyRefillAtLoader.refillTargetOffset = 1.5

-- Distance thresholds for determining when we've reached the loader
AIDriveStrategyRefillAtLoader.minDistanceToLoader = 15  -- Distance to loader vehicle itself (meters)
AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode = 8  -- Distance to discharge node (refill point, meters)
AIDriveStrategyRefillAtLoader.minDistanceToRefillTarget = 3  -- Distance to calculated target position (meters)

AIDriveStrategyRefillAtLoader.myStates = {
    SEARCHING_FOR_LOADER = {},
    WAITING_FOR_LOADER = {},
    WAITING_FOR_FOLD = {},
    WAITING_FOR_PATHFINDER = {},
    DRIVING_TO_LOADER = {},
    WAITING_FOR_REFILL = {},
    REFILL_COMPLETE = {},
}

function AIDriveStrategyRefillAtLoader:init(task, job)
    AIDriveStrategyCourse.init(self, task, job)
    AIDriveStrategyCourse.initStates(self, AIDriveStrategyRefillAtLoader.myStates)
    self.state = self.states.SEARCHING_FOR_LOADER
    self.debugChannel = CpDebug.DBG_FIELDWORK
    
    self.refillTimer = 0
    self.refillTimeout = 30000 -- 30 seconds timeout for refilling
    self.loaderVehicle = nil
    self.dischargeNode = nil
    self.refillTargetNode = nil
    self.refillAlignCourse = nil
    self.implementsRaised = false
    
    self.fieldPolygon = nil
    self.loaderSearchTimer = 0
    self.loaderSearchInterval = 5000
    self.lastSearchedFillType = nil
    self.refillSucceeded = false
    self.startCalled = false
    
    -- Folding management
    self.foldStartTime = nil
    self.foldTimeout = 30000  -- 30 seconds max wait for folding
    self.foldCommandRetryMs = 2000  -- Retry fold command every 2 seconds
    self.nextFoldCommandAt = 0  -- Time when next fold command should be sent
end

function AIDriveStrategyRefillAtLoader:delete()
    if self.refillTargetNode then
        CpUtil.destroyNode(self.refillTargetNode)
        self.refillTargetNode = nil
    end
    AIDriveStrategyCourse.delete(self)
end

function AIDriveStrategyRefillAtLoader:initializeImplementControllers(vehicle)
    self:addImplementController(vehicle, MotorController, Motorized, {})
    self:addImplementController(vehicle, WearableController, Wearable, {})
    self:addImplementController(vehicle, FoldableController, Foldable, {})
    self:addImplementController(vehicle, PickupController, Pickup, {})
    self:addImplementController(vehicle, CutterController, Cutter, {})
    self:addImplementController(vehicle, SowingMachineController, SowingMachine, {})
end

function AIDriveStrategyRefillAtLoader:setAIVehicle(vehicle, jobParameters)
    AIDriveStrategyCourse.setAIVehicle(self, vehicle, jobParameters)
    self:raiseImplements()
end

--- Set the field polygon to use for finding loaders
---@param fieldPolygon table [{x, y, z}] field boundary vertices
function AIDriveStrategyRefillAtLoader:setFieldPolygon(fieldPolygon, islandPolygons)
    self.fieldPolygon = fieldPolygon
    self.islandPolygons = islandPolygons
    self:debug('Field polygon set with %d vertices', fieldPolygon and #fieldPolygon or 0)
end

function AIDriveStrategyRefillAtLoader:start()
    if self.startCalled then
        return
    end
    self.startCalled = true
    
    self:debug('Starting refill at loader strategy')
    self.refillSucceeded = false
    
    local fieldPolygon = self.fieldPolygon
    if not fieldPolygon or #fieldPolygon == 0 then
        self:debug('ERROR: No field polygon available, cannot search for loader')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    -- Get required fill type
    local fillTypeIndex = self:getRequiredFillType()
    if not fillTypeIndex then
        self:debug('ERROR: Could not determine required fill type')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    local fillTypeName = g_fillTypeManager:getFillTypeNameByIndex(fillTypeIndex)
    self:debug('Required fill type: %s (index %d)', fillTypeName, fillTypeIndex)
    
    -- Find loader
    self.loaderVehicle, self.dischargeNode = SelfRefillHelper:findBestLoader(fieldPolygon, self.vehicle, fillTypeIndex)
    
    if not self.loaderVehicle then
        self:debug('No loader found yet, entering waiting state')
        self.state = self.states.WAITING_FOR_LOADER
        self.loaderSearchTimer = 0
        self.lastSearchedFillType = fillTypeIndex
        return
    end
    
    self:debug('Loader found: %s', CpUtil.getName(self.loaderVehicle))
    
    -- Check distance to loader
    local distanceToLoader = calcDistanceFrom(self.vehicle:getAIDirectionNode(), self.loaderVehicle.rootNode)
    self:debug('Distance to loader: %.1f meters', distanceToLoader)
    
    -- If very close already (within 10m), skip pathfinding and start refilling
    if distanceToLoader < 10 then
        self:debug('Vehicle is already close to loader, starting refill directly')
        self.state = self.states.WAITING_FOR_REFILL
        self.refillTimer = 0
        self:prepareForRefill()
        return
    end
    
    -- Need pathfinding to reach loader - first fold implements
    self:debug('Vehicle is %.1fm away, folding implements first...', distanceToLoader)
    self:startFoldingForPathfinding()
end

--- Send fold command to a single object (vehicle or implement)
function AIDriveStrategyRefillAtLoader:requestFoldForObject(object)
    if not object then
        return
    end

    local name = CpUtil.getName(object)
    local spec = object.spec_foldable
    if not spec then
        self:debug('REFILL FOLD: %s has no foldable spec', name)
        return
    end

    local isUnfolded = object.getIsUnfolded and object:getIsUnfolded()
    local foldAnimTime = spec.foldAnimTime
    local turnOnFoldDirection = spec.turnOnFoldDirection or 1
    local foldDirection = -turnOnFoldDirection
    self:debug('REFILL FOLD: %s status before command -> unfolded=%s foldAnimTime=%s turnOnDir=%d foldDir=%d',
        name, tostring(isUnfolded), tostring(foldAnimTime), turnOnFoldDirection, foldDirection)

    if object.prepareForAIDriving then
        object:prepareForAIDriving()
    end
    if object.setFoldDirection then
        object:setFoldDirection(foldDirection, true)
        self:debug('REFILL FOLD: %s setFoldDirection(%d) sent', name, foldDirection)
    else
        self:debug('REFILL FOLD: %s has no setFoldDirection()', name)
    end
end

--- Send fold commands to vehicle and all attached implements
function AIDriveStrategyRefillAtLoader:requestFoldAllForRefill()
    self:debug('REFILL FOLD: sending fold commands to vehicle and attached implements')
    self:requestFoldForObject(self.vehicle)
    for _, implement in pairs(self.vehicle:getAttachedAIImplements()) do
        if implement and implement.object then
            self:requestFoldForObject(implement.object)
        end
    end
    self.nextFoldCommandAt = g_currentMission.time + self.foldCommandRetryMs
end

--- Checks if vehicle or attached AI implements are still unfolded.
---@return boolean true when everything relevant is folded for pathfinding.
function AIDriveStrategyRefillAtLoader:areFoldablesReadyForPathfinding()
    local function isFolded(object)
        if not object or not object.spec_foldable then
            return true
        end

        local spec = object.spec_foldable
        local isUnfolded = object.getIsUnfolded and object:getIsUnfolded() or false
        local turnOnFoldDirection = spec.turnOnFoldDirection or 1
        local foldedTargetAnimTime = turnOnFoldDirection == -1 and 1 or 0
        local foldAnimTime = spec.foldAnimTime or foldedTargetAnimTime
        local foldMoveDirection = spec.foldMoveDirection or 0

        local isAtFoldedAnimTarget = math.abs(foldAnimTime - foldedTargetAnimTime) < 0.01
        local isNotMoving = math.abs(foldMoveDirection) < 0.001

        return not isUnfolded and isAtFoldedAnimTarget and isNotMoving
    end

    if not isFolded(self.vehicle) then
        return false
    end

    for _, implement in pairs(self.vehicle:getAttachedAIImplements()) do
        if implement.object and not isFolded(implement.object) then
            return false
        end
    end

    return true
end

--- Start folding implements before pathfinding
function AIDriveStrategyRefillAtLoader:startFoldingForPathfinding()
    self:debug('Starting to fold implements for pathfinding...')
    self:raiseImplements()
    self:requestFoldAllForRefill()
    
    self.state = self.states.WAITING_FOR_FOLD
    self.foldStartTime = g_currentMission.time
    self:debug('Waiting for implements to fold completely...')
end

--- Start pathfinding to the loader (similar to unloader approach)
function AIDriveStrategyRefillAtLoader:startPathfindingToLoader()
    if self.pathfinder and self.pathfinder:isActive() then
        self:debug('Pathfinder already active')
        return
    end
    
    -- Get target parameters
    local targetNode, alignLength, offsetX, loaderVehicle = self:getRefillTargetParameters()
    if not targetNode then
        self:debug('Could not get refill target parameters')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    self.refillTargetNode = targetNode
    self.loaderVehicle = loaderVehicle
    
    -- Create alignment course (straight section parallel to loader)
    self:debug('Creating align course relative to target node from %.1f to %.1f',
            -alignLength + 1, -self.refillTargetOffset)
    self.refillAlignCourse = Course.createFromNode(self.vehicle, self.refillTargetNode,
            offsetX, -alignLength + 1,
            -self.refillTargetOffset,
            1, false)
    
    self.state = self.states.WAITING_FOR_PATHFINDER
    
    local fieldNum = CpFieldUtil.getFieldNumUnderVehicle(self.vehicle)
    local context = PathfinderContext(self.vehicle)
    
    -- Configure pathfinding context (similar to unloader)
    context:maxFruitPercent(100)  -- Slurry tankers can drive through crops
    context:offFieldPenalty(PathfinderContext.defaultOffFieldPenalty)
    context:mustBeAccurate(true)  -- Need precise positioning for refill
    context:useFieldNum(fieldNum)
    context:allowReverse(self:getAllowReversePathfinding())
    
    -- Ignore off-field penalty around the loader to allow bridging gap between field and loader
    context:areaToIgnoreOffFieldPenalty(
            PathfinderUtil.NodeArea.createVehicleArea(self.loaderVehicle, 1.5 * SelfRefillHelper.maxDistanceFromField))
    
    context:maxIterations(PathfinderUtil.getMaxIterationsForFieldPolygon(self.vehicle:cpGetFieldPolygon()))
    
    self.pathfinderController:registerListeners(self,
            self.onPathfindingDoneToLoader,
            self.onPathfindingFailedToLoader,
            self.onPathfindingObstacleAtStart)
    
    self.pathfinderController:findPathToNode(context, self.refillTargetNode, offsetX, -alignLength, 3)
    
    self:debug('Pathfinding started to loader')
end

--- Get target parameters for refill (delegated to SelfRefillHelper)
function AIDriveStrategyRefillAtLoader:getRefillTargetParameters()
    local fillTypeIndex = self.lastSearchedFillType or self:getRequiredFillType()
    return SelfRefillHelper:getLoaderTargetParameters(
            self.vehicle:cpGetFieldPolygon(),
            self.vehicle,
            fillTypeIndex,
            self.loaderVehicle,
            self.dischargeNode)
end

--- Callback when pathfinding to loader is done
function AIDriveStrategyRefillAtLoader:onPathfindingDoneToLoader(controller, success, course, goalNodeInvalid)
    if success then
        self:debug('Pathfinding successful, %d waypoints', course and #course or 0)
        course:append(self.refillAlignCourse)
        self.state = self.states.DRIVING_TO_LOADER
        self:startCourse(course, 1)
        return true
    else
        self:debug('Pathfinding failed, stopping job')
        self.vehicle:stopCurrentAIJob(AIMessageCpErrorNoPathFound.new())
        return false
    end
end

--- Callback when pathfinding fails
function AIDriveStrategyRefillAtLoader:onPathfindingFailedToLoader(controller, lastContext, wasLastRetry,
                                                                    currentRetryAttempt, trailerCollisionsOnly,
                                                                    fruitPenaltyNodePercent, offFieldPenaltyNodePercent)
    local offFieldPenaltyRelaxingSteps = { 0.5, 0.25, 0.1}
    
    if not wasLastRetry then
        -- Relax off-field penalty and retry
        if offFieldPenaltyRelaxingSteps[currentRetryAttempt] then
            self:debug('Pathfinding failed, relaxing off-field penalty to %.2f and retrying',
                    offFieldPenaltyRelaxingSteps[currentRetryAttempt])
            lastContext:offFieldPenalty(offFieldPenaltyRelaxingSteps[currentRetryAttempt] * PathfinderContext.defaultOffFieldPenalty)
        end
        controller:retry(lastContext)
    else
        -- All retries exhausted
        self:debug('Pathfinding failed after all retries')
        self.vehicle:stopCurrentAIJob(AIMessageCpErrorNoPathFound.new())
    end
end

--- Callback when obstacle is detected at start
function AIDriveStrategyRefillAtLoader:onPathfindingObstacleAtStart(controller, lastContext, maxDistance,
                                                                     trailerCollisionsOnly, fruitPenaltyNodePercent,
                                                                     offFieldPenaltyNodePercent)
    if trailerCollisionsOnly then
        self:debug('Pathfinding detected obstacle at start (trailer collisions only), ignoring')
        lastContext:ignoreTrailerAtStartRange(1.5 * self.turningRadius)
        controller:retry(lastContext)
    else
        self:debug('Pathfinding detected obstacle at start, cannot proceed')
        self.vehicle:stopCurrentAIJob(AIMessageCpErrorNoPathFound.new())
    end
end

function AIDriveStrategyRefillAtLoader:update(dt)
    -- Call start() on first update if not called yet
    if not self.startCalled and self.state == self.states.SEARCHING_FOR_LOADER then
        self:debug('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        self:debug('update() calling start() for the first time')
        self:debug('BEFORE start(): self.fieldPolygon is %s', self.fieldPolygon and 'NOT NIL' or 'NIL')
        if self.fieldPolygon then
            self:debug('BEFORE start(): fieldPolygon has %d vertices', #self.fieldPolygon)
        end
        self:debug('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        self:start()
    end
    
    -- Handle waiting for loader to appear
    if self.state == self.states.WAITING_FOR_LOADER then
        self:updateWaitingForLoader(dt)
    end
    
    -- Handle waiting for implements to fold
    if self.state == self.states.WAITING_FOR_FOLD then
        -- Periodically resend fold commands in case some implements didn't respond
        if g_currentMission.time >= self.nextFoldCommandAt then
            self:debug('REFILL FOLD: Resending fold commands...')
            self:requestFoldAllForRefill()
        end
        
        local foldingComplete = self:areFoldablesReadyForPathfinding()
        local timeoutReached = (g_currentMission.time - (self.foldStartTime or 0)) > self.foldTimeout
        
        if foldingComplete then
            self:debug('Implements folded successfully, starting pathfinding now')
            self:startPathfindingToLoader()
        elseif timeoutReached then
            self:debug('WARNING: Fold timeout reached, starting pathfinding anyway')
            self:startPathfindingToLoader()
        else
            if g_updateLoopIndex % 100 == 0 then
                self:debug('Waiting for implements to fold...')
            end
        end
    end

    -- Keep PPC off-track auto-stop disabled while navigating the generated refill approach path.
    -- This must happen before AIDriveStrategyCourse.update(), as PPC logic runs there.
    if self.state == self.states.DRIVING_TO_LOADER then
        self.ppc:disableStopWhenOffTrack(15000)
    end
    
    AIDriveStrategyCourse.update(self, dt)
    self:updateImplementControllers(dt)
    
    if self.state == self.states.WAITING_FOR_REFILL then
        self:updateRefilling(dt)
    end
end

--- Update function while waiting for a loader to appear
function AIDriveStrategyRefillAtLoader:updateWaitingForLoader(dt)
    self.loaderSearchTimer = self.loaderSearchTimer + dt
    
    -- Search for loader every 5 seconds
    if self.loaderSearchTimer >= self.loaderSearchInterval then
        self.loaderSearchTimer = 0
        
        CpUtil.info('=========================================')
        CpUtil.info('REFILL STRATEGY: Searching for loader again...')
        
        -- Search for loader
        local fieldPolygon = self.fieldPolygon
        local fillTypeIndex = self.lastSearchedFillType
        
        if not fieldPolygon or not fillTypeIndex then
            CpUtil.info('REFILL STRATEGY: ERROR - Missing field polygon or fill type')
            self.state = self.states.REFILL_COMPLETE
            return
        end
        
        local fillTypeName = g_fillTypeManager:getFillTypeNameByIndex(fillTypeIndex)
        CpUtil.info('REFILL STRATEGY: Looking for: %s', fillTypeName)
        
        self.loaderVehicle, self.dischargeNode = SelfRefillHelper:findBestLoader(fieldPolygon, self.vehicle, fillTypeIndex)
        
        if self.loaderVehicle then
            -- Found a loader! Continue with normal flow
            local x, y, z = getWorldTranslation(self.loaderVehicle.rootNode)
            CpUtil.info('REFILL STRATEGY: ✓✓✓ SUCCESS - Loader found!')
            CpUtil.info('REFILL STRATEGY: Loader: %s', CpUtil.getName(self.loaderVehicle))
            CpUtil.info('REFILL STRATEGY: Position: %.1f, %.1f, %.1f', x, y, z)
            
            -- Check distance to loader
            local vx, vy, vz = getWorldTranslation(self.vehicle.rootNode)
            local distanceToLoader = math.sqrt((x - vx)^2 + (z - vz)^2)
            CpUtil.info('REFILL STRATEGY: Distance to loader: %.1f meters', distanceToLoader)
            
            -- If very close already (within 30m), skip pathfinding and start refilling immediately
            if distanceToLoader < 30 then
                CpUtil.info('REFILL STRATEGY: ✓ Vehicle is already close to loader!')
                CpUtil.info('REFILL STRATEGY: Skipping pathfinding, starting refill directly')
                CpUtil.info('=========================================')
                self.state = self.states.WAITING_FOR_REFILL
                self.refillTimer = 0
                self:prepareForRefill()
                return
            end
            -- Vehicle is far away, need to calculate path
            CpUtil.info('REFILL STRATEGY: Vehicle is %.1fm away, starting pathfinding...', distanceToLoader)
            CpUtil.info('REFILL STRATEGY: >>> Starting pathfinding to loader <<<')
            
            -- Start pathfinding
            self:startPathfindingToLoader()
            CpUtil.info('=========================================')
        else
            -- Still no loader found
            CpUtil.info('REFILL STRATEGY: Still no loader found, waiting...')
            CpUtil.info('=========================================')
        end
    end
end

function AIDriveStrategyRefillAtLoader:getParallelHeadingForLoader()
    if not self.loaderVehicle or not self.loaderVehicle.rootNode then
        return nil
    end

    local _, loaderYRot, _ = getWorldRotation(self.loaderVehicle.rootNode)
    local _, vehicleYRot, _ = getWorldRotation(self.vehicle:getAIDirectionNode())
    local oppositeLoaderYRot = loaderYRot + math.pi

    local deltaSame = math.abs(CpMathUtil.getDeltaAngle(loaderYRot, vehicleYRot))
    local deltaOpposite = math.abs(CpMathUtil.getDeltaAngle(oppositeLoaderYRot, vehicleYRot))

    if deltaOpposite < deltaSame then
        return oppositeLoaderYRot
    end

    return loaderYRot
end

function AIDriveStrategyRefillAtLoader:getDistanceToDischargeNode()
    if self.dischargeNode and self.dischargeNode.node then
        return calcDistanceFrom(self.vehicle:getAIDirectionNode(), self.dischargeNode.node)
    end
    return nil
end

function AIDriveStrategyRefillAtLoader:getDriveData(dt, vX, vY, vZ)
    self:updateLowFrequencyImplementControllers()
    
    local moveForwards = not self.ppc:isReversing()
    local gx, gz
    
    if self.state == self.states.SEARCHING_FOR_LOADER then
        -- Make sure implements are raised while searching
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        -- Stop and wait at current position
        self:setMaxSpeed(0)
        gx, gz = vX, vZ
        
    elseif self.state == self.states.WAITING_FOR_FOLD then
        -- Stop and wait while implements are folding
        self:setMaxSpeed(0)
        gx, gz = vX, vZ
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        
    elseif self.state == self.states.WAITING_FOR_PATHFINDER then
        -- Stop and wait while pathfinder is calculating route
        self:setMaxSpeed(0)
        gx, gz = vX, vZ
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        
    elseif self.state == self.states.DRIVING_TO_LOADER then
        -- The generated loader approach path can contain larger transitions and temporary crosstrack spikes,
        -- especially with long slurry implements. Keep PPC off-track stop disabled while following this path.
        self.ppc:disableStopWhenOffTrack(15000)

        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        
        if not moveForwards then
            local maxSpeed
            gx, gz, maxSpeed = self:getReverseDriveData()
            self:setMaxSpeed(maxSpeed)
        else
            gx, _, gz = self.ppc:getGoalPointPosition()
            self:setMaxSpeed(self.settings.fieldSpeed:getValue())
        end
        
        -- Check if we've reached the loader
        local distanceToLoader = calcDistanceFrom(self.vehicle:getAIDirectionNode(), 
            self.loaderVehicle.rootNode)
        local distanceToDischarge = self:getDistanceToDischargeNode() or math.huge
        local distanceToTarget = math.huge
        if self.targetRefillPosition then
            local tx, _, tz = getWorldTranslation(self.vehicle:getAIDirectionNode())
            distanceToTarget = MathUtil.vector2Length(self.targetRefillPosition.x - tx, self.targetRefillPosition.z - tz)
        end
        if g_updateLoopIndex % 100 == 0 then  -- Log every ~3 seconds
            self:debug('REFILL STRATEGY: Distance to loader: %.1fm (threshold: %.1fm), discharge: %.1fm (threshold: %.1fm), target: %.1fm (threshold: %.1fm)', 
                distanceToLoader, AIDriveStrategyRefillAtLoader.minDistanceToLoader,
                distanceToDischarge, AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode,
                distanceToTarget, AIDriveStrategyRefillAtLoader.minDistanceToRefillTarget)
        end

        local reachedCourseEndNearTarget = self.ppc:getCourse():isCloseToLastWaypoint(5) and
            distanceToTarget < AIDriveStrategyRefillAtLoader.minDistanceToRefillTarget
        local reachedDischargeNode = distanceToDischarge < AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode
        local reachedLoaderAsFallback = distanceToLoader < AIDriveStrategyRefillAtLoader.minDistanceToLoader and
            distanceToDischarge < (AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode + 1.5)

        if reachedDischargeNode or reachedCourseEndNearTarget or reachedLoaderAsFallback then
            CpUtil.info('=========================================')
            CpUtil.info('REFILL STRATEGY: ✓✓✓ ARRIVED at loader!')
            CpUtil.info('REFILL STRATEGY: Distance to loader: %.1fm, discharge: %.1fm, distance to target: %.1fm',
                distanceToLoader, distanceToDischarge, distanceToTarget)
            CpUtil.info('REFILL STRATEGY: Preparing for refill...')
            self.state = self.states.WAITING_FOR_REFILL
            self.refillTimer = 0
            self.refillApproachRetryCount = 0
            self:prepareForRefill()
            CpUtil.info('REFILL STRATEGY: Waiting for refill to complete...')
            CpUtil.info('=========================================')
        elseif self.ppc:getCourse():isCloseToLastWaypoint(5) then
            self:debug('REFILL STRATEGY: Reached end of course but not close enough (discharge %.1fm, target %.1fm)',
                distanceToDischarge, distanceToTarget)
        end
        
    elseif self.state == self.states.WAITING_FOR_LOADER then
        -- Wait for loader to appear
        self:setMaxSpeed(0)
        gx, gz = 0, 0
        -- Keep implements raised while waiting
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        
    elseif self.state == self.states.WAITING_FOR_REFILL then
        self:setMaxSpeed(0)
        gx, gz = 0, 0
        -- Keep implements raised during refilling
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        
    elseif self.state == self.states.REFILL_COMPLETE then
        self:setMaxSpeed(0)
        gx, gz = 0, 0
    end
    
    self:checkProximitySensors(moveForwards)
    
    return gx, gz, moveForwards, self.maxSpeed, 100
end

function AIDriveStrategyRefillAtLoader:updateRefilling(dt)
    self.refillTimer = self.refillTimer + dt
    
    -- Update controller refilling (tries to load from nearby fill triggers/vehicles)
    if self.controllers then
        for _, controller in pairs(self.controllers) do
            if controller.onUpdateRefilling then
                local timerFinished, hasChanged = controller:onUpdateRefilling()
                if hasChanged then
                    self:debug('REFILL STRATEGY: ✓ Fill level changed during refilling')
                    self.refillTimer = 0 -- Reset timer when filling is active
                    self.refillSucceeded = true  -- Mark refill as successful
                end
            end
        end
    end
    
    -- Check if refilling is complete
    if self:isRefillingComplete() then
        CpUtil.info('=========================================')
        CpUtil.info('REFILL STRATEGY: ✓✓✓ REFILLING COMPLETE!')
        CpUtil.info('REFILL STRATEGY: Tank is now at 95%% or higher')
        CpUtil.info('REFILL STRATEGY: Returning to fieldwork...')
        self.refillSucceeded = true  -- Ensure flag is set
        self.state = self.states.REFILL_COMPLETE
        self:finishRefilling()
        CpUtil.info('=========================================')
        return
    end

    -- If no refill happens and we are still too far from the actual discharge node,
    -- recalculate approach once or twice before timing out.
    if not self.refillSucceeded and self.refillTimer > 5000 and
        self.refillApproachRetryCount < 2 then
        local distanceToDischarge = self:getDistanceToDischargeNode() or math.huge
        if distanceToDischarge > (AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode + 1.0) then
            self.refillApproachRetryCount = self.refillApproachRetryCount + 1
            CpUtil.info('REFILL STRATEGY: No refill detected and still %.1fm from discharge node, recalculating approach (retry %d/2)',
                distanceToDischarge, self.refillApproachRetryCount)
            self:startPathfindingToLoader()
            return
        end
    end
    
    -- Check for timeout
    if self.refillTimer > self.refillTimeout then
        CpUtil.info('=========================================')
        CpUtil.info('REFILL STRATEGY: WARNING - Refilling timeout after %.1f seconds', self.refillTimer / 1000)
        CpUtil.info('REFILL STRATEGY: ')
        CpUtil.info('REFILL STRATEGY: POSSIBLE REASONS:')
        CpUtil.info('REFILL STRATEGY:   □ Tanker discharge not activated (press Q on tanker)')
        CpUtil.info('REFILL STRATEGY:   □ Wrong refill pipe selected on tanker')
        CpUtil.info('REFILL STRATEGY:   □ Vehicle not close enough to tanker')
        CpUtil.info('REFILL STRATEGY:   □ Tanker is empty')
        CpUtil.info('REFILL STRATEGY: ')
        CpUtil.info('REFILL STRATEGY: Vehicle will retry in 30 seconds')
        CpUtil.info('=========================================')
        self.state = self.states.REFILL_COMPLETE
        self:finishRefilling()
        return
    end
    
    if self.refillTimer % 5000 < dt then  -- Log every 5 seconds
        local currentFillLevel = 0
        local totalCapacity = 0
        for fillUnitIndex, fillUnit in pairs(self.vehicle:getFillUnits()) do
            local capacity = self.vehicle:getFillUnitCapacity(fillUnitIndex)
            if capacity and capacity > 100 then
                currentFillLevel = currentFillLevel + self.vehicle:getFillUnitFillLevel(fillUnitIndex)
                totalCapacity = totalCapacity + capacity
            end
        end
        local fillPercent = totalCapacity > 0 and (currentFillLevel / totalCapacity * 100) or 0
        self:debug('REFILL STRATEGY: Waiting for refill... %.1fs/%.1fs (Fill level: %.1f percent)', 
            self.refillTimer / 1000, self.refillTimeout / 1000, fillPercent)
    end
end

function AIDriveStrategyRefillAtLoader:prepareForRefill()
    self:debug('Preparing for refill')
    -- Enable refilling mode for all relevant controllers
    if self.controllers then
        for _, controller in pairs(self.controllers) do
            if controller.onStartRefilling then
                controller:onStartRefilling()
            end
        end
    end
end

function AIDriveStrategyRefillAtLoader:finishRefilling()
    self:debug('Finishing refill')
    -- Disable refilling mode for all relevant controllers
    if self.controllers then
        for _, controller in pairs(self.controllers) do
            if controller.onStopRefilling then
                controller:onStopRefilling()
            end
        end
    end
end

function AIDriveStrategyRefillAtLoader:isRefillingComplete()
    -- Only check fill units that were explicitly registered for refilling by implement controllers.
    -- This avoids false positives from unrelated vehicle tanks (diesel/DEF/etc.).
    local hasAnyRelevantUnit = false
    local allRelevantUnitsFilled = true

    local function isFuelType(fillType)
        return fillType == FillType.DIESEL or
               fillType == FillType.ELECTRICCHARGE or
               fillType == FillType.METHANE or
               fillType == FillType.DEF
    end

    if not self.controllers then
        return false
    end

    for _, controller in pairs(self.controllers) do
        local refillData = controller.refillData
        if refillData and refillData.lastFillLevels then
            for implement, data in pairs(refillData.lastFillLevels) do
                for fillUnitIndex, _ in pairs(data) do
                    local capacity = implement:getFillUnitCapacity(fillUnitIndex)
                    if capacity and capacity > 0 then
                        local fillType = implement:getFillUnitFillType(fillUnitIndex)
                        if fillType ~= FillType.UNKNOWN and not isFuelType(fillType) then
                            hasAnyRelevantUnit = true
                            local fillLevel = implement:getFillUnitFillLevel(fillUnitIndex)
                            local fillLevelPercent = capacity > 0 and (fillLevel / capacity) or 0
                            if fillLevelPercent < 0.95 then
                                allRelevantUnitsFilled = false
                                self:debugSparse('Refill unit %s:%d at %.1f percent (%.0f/%.0f)',
                                    CpUtil.getName(implement), fillUnitIndex, fillLevelPercent * 100, fillLevel, capacity)
                            end
                        end
                    end
                end
            end
        end
    end

    if hasAnyRelevantUnit then
        return allRelevantUnitsFilled
    end

    -- Fallback: some setups do not populate controller refillData consistently.
    -- In that case, evaluate real refill-capable units directly on vehicle + attached implements.
    local requiredFillType = self:getRequiredFillType()
    if not requiredFillType then
        return false
    end

    local function unitSupportsRequiredFillType(object, fillUnitIndex)
        local fillType = object:getFillUnitFillType(fillUnitIndex)
        if fillType == requiredFillType then
            return true
        end
        if fillType == FillType.UNKNOWN and object.getFillUnitSupportedFillTypes then
            local supportedFillTypes = object:getFillUnitSupportedFillTypes(fillUnitIndex)
            return supportedFillTypes ~= nil and supportedFillTypes[requiredFillType] == true
        end
        return false
    end

    local fallbackHasRelevantUnit = false
    local fallbackAllRelevantUnitsFilled = true

    local function evaluateObject(object)
        if not object or not object.getFillUnits then
            return
        end
        local fillUnits = object:getFillUnits()
        if not fillUnits then
            return
        end

        for fillUnitIndex, _ in pairs(fillUnits) do
            local capacity = object:getFillUnitCapacity(fillUnitIndex)
            if capacity and capacity > 0 and unitSupportsRequiredFillType(object, fillUnitIndex) then
                fallbackHasRelevantUnit = true
                local fillLevel = object:getFillUnitFillLevel(fillUnitIndex)
                local fillLevelPercent = fillLevel / capacity
                if fillLevelPercent < 0.95 then
                    fallbackAllRelevantUnitsFilled = false
                    self:debugSparse('Fallback refill unit %s:%d at %.1f percent (%.0f/%.0f)',
                        CpUtil.getName(object), fillUnitIndex, fillLevelPercent * 100, fillLevel, capacity)
                end
            end
        end
    end

    evaluateObject(self.vehicle)
    for _, implement in pairs(self.vehicle:getAttachedAIImplements()) do
        if implement and implement.object then
            evaluateObject(implement.object)
        end
    end

    if fallbackHasRelevantUnit and fallbackAllRelevantUnitsFilled then
        self:debug('REFILL STRATEGY: Fallback completion detection confirmed all relevant units >= 95%%')
    end

    return fallbackHasRelevantUnit and fallbackAllRelevantUnitsFilled
end

--- Get the fill type that the vehicle needs to refill
---@return number|nil fill type index
function AIDriveStrategyRefillAtLoader:getRequiredFillType()
    local vehicle = self.vehicle
    
    -- Helper function to check if a fill type is a fuel (should be ignored)
    local function isFuelType(fillType)
        return fillType == FillType.DIESEL or 
               fillType == FillType.ELECTRICCHARGE or
               fillType == FillType.METHANE or
               fillType == FillType.DEF
    end
    
    -- PRIORITY 1: Check attached implements with Sprayer specialization
    local implements = AIUtil.getAllChildVehiclesWithSpecialization(vehicle, Sprayer)
    if implements then
        for _, implement in pairs(implements) do
            if implement.spec_sprayer then
                local fillType = implement:getFillUnitFillType(implement.spec_sprayer.fillUnitIndex)
                if fillType and fillType ~= FillType.UNKNOWN and not isFuelType(fillType) then
                    return fillType
                end
                -- Try supported spray types
                if implement.spec_sprayer.supportedSprayTypes then
                    for _, sprayType in ipairs(implement.spec_sprayer.supportedSprayTypes) do
                        if not isFuelType(sprayType) then
                            return sprayType
                        end
                    end
                end
            end
        end
    end
    
    -- PRIORITY 2: Check vehicle itself for sprayer
    if SpecializationUtil.hasSpecialization(Sprayer, vehicle.specializations) then
        local sprayerSpec = vehicle.spec_sprayer
        if sprayerSpec then
            local fillType = vehicle:getFillUnitFillType(sprayerSpec.fillUnitIndex)
            if fillType and fillType ~= FillType.UNKNOWN and not isFuelType(fillType) then
                return fillType
            end
            -- Try to get supported fill types
            if sprayerSpec.supportedSprayTypes and #sprayerSpec.supportedSprayTypes > 0 then
                for _, sprayType in ipairs(sprayerSpec.supportedSprayTypes) do
                    if not isFuelType(sprayType) then
                        return sprayType
                    end
                end
            end
        end
    end
    
    -- PRIORITY 3: Check all fill units, but skip fuel types
    local fillUnits = vehicle:getFillUnits()
    if fillUnits then
        for fillUnitIndex, fillUnit in pairs(fillUnits) do
            local capacity = vehicle:getFillUnitCapacity(fillUnitIndex)
            if capacity and capacity > 100 then -- Only significant capacity units
                local fillType = vehicle:getFillUnitFillType(fillUnitIndex)
                if fillType and fillType ~= FillType.UNKNOWN and not isFuelType(fillType) then
                    return fillType
                end
                -- If empty, try to find supported fill types (excluding fuels)
                local fillLevel = vehicle:getFillUnitFillLevel(fillUnitIndex)
                if fillLevel <= 0 and vehicle.getFillUnitSupportedFillTypes then
                    local supportedFillTypes = vehicle:getFillUnitSupportedFillTypes(fillUnitIndex)
                    if supportedFillTypes then
                        for supportedFillType, _ in pairs(supportedFillTypes) do
                            -- Return first non-fuel liquid type found
                            if not isFuelType(supportedFillType) and 
                               g_fillTypeManager:getFillTypeByIndex(supportedFillType).isLiquid then
                                return supportedFillType
                            end
                        end
                    end
                end
            end
        end
    end
    
    return nil
end
