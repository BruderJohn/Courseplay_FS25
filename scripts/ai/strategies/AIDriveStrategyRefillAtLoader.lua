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
AIDriveStrategyRefillAtLoader.refillTargetOffset = 0.5

-- Distance thresholds for determining when we've reached the loader
AIDriveStrategyRefillAtLoader.minDistanceToLoader = 15  -- Distance to loader vehicle itself (meters)
AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode = 5  -- Distance to discharge node (refill point, meters)
AIDriveStrategyRefillAtLoader.minDistanceToRefillTarget = 2  -- Distance to calculated target position (meters)

AIDriveStrategyRefillAtLoader.myStates = {
    SEARCHING_FOR_LOADER = {},
    WAITING_FOR_LOADER = {},
    WAITING_FOR_FOLD = {},
    WAITING_FOR_PATHFINDER = {},
    DRIVING_TO_LOADER = {},
    WAITING_FOR_REFILL = {},
    DRIVING_BACK_TO_COURSE = {},
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
    
    -- Saved course for return
    self.savedCourse = nil
    self.savedWaypointIx = nil
    
    -- Cleanup tracking
    self.dischargeCleaned = false
    
    -- Folding management
    self.foldStartTime = nil
    self.foldTimeout = 30000  -- 30 seconds max wait for folding
    self.foldCommandRetryMs = 2000  -- Retry fold command every 2 seconds
    self.nextFoldCommandAt = 0  -- Time when next fold command should be sent
    
    -- Debug tracking
    self.lastStateChange = g_currentMission.time
    self.lastDebugUpdate = 0
end

function AIDriveStrategyRefillAtLoader:delete()
    -- Ensure discharge cleanup happens during strategy deletion
    CpUtil.info('DEBUG: delete() called for refill strategy')
    
    -- Extra safety: Turn off discharge state one more time before deletion
    if self.vehicle and self.vehicle.spec_dischargeable then
        if self.vehicle.setDischargeState then
            self.vehicle:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
            CpUtil.info('DEBUG: Pre-delete discharge OFF for vehicle')
        end
    end
    if self.vehicle then
        for _, implement in pairs(self.vehicle:getAttachedImplements()) do
            local object = implement.object
            if object and object.spec_dischargeable and object.setDischargeState then
                object:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
                CpUtil.info('DEBUG: Pre-delete discharge OFF for implement')
            end
        end
    end
    
    self:cleanupDischargeState()
    
    -- Don't destroy refillTargetNode - it's a reference to the loader's discharge node, not ours!
    -- Just clear the reference
    self.refillTargetNode = nil
    
    -- Clear references to avoid entity access errors
    self.loaderVehicle = nil
    self.dischargeNode = nil
    CpUtil.info('DEBUG: delete() completed')
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
    CpUtil.info('DEBUG: setAIVehicle called for %s', CpUtil.getName(vehicle))
    AIDriveStrategyCourse.setAIVehicle(self, vehicle, jobParameters)
    self:raiseImplements()
    CpUtil.info('DEBUG: setAIVehicle completed')
end

--- Set the field polygon to use for finding loaders
---@param fieldPolygon table [{x, y, z}] field boundary vertices
function AIDriveStrategyRefillAtLoader:setFieldPolygon(fieldPolygon, islandPolygons)
    self.fieldPolygon = fieldPolygon
    self.islandPolygons = islandPolygons
    self:debug('Field polygon set with %d vertices', fieldPolygon and #fieldPolygon or 0)
end

--- Set the saved course to return to after refilling
---@param course Course the course to return to
---@param waypointIx number waypoint index to continue from
function AIDriveStrategyRefillAtLoader:setSavedCourse(course, waypointIx)
    self.savedCourse = course
    self.savedWaypointIx = waypointIx
    self:debug('Saved course set with waypoint %d', waypointIx)
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
    
    CpUtil.info('DEBUG: Loader search result - vehicle: %s, dischargeNode: %s', 
        self.loaderVehicle and CpUtil.getName(self.loaderVehicle) or 'nil',
        self.dischargeNode and tostring(self.dischargeNode.node) or 'nil')
    
    if self.loaderVehicle then
        CpUtil.info('DEBUG: Found loader with entity ID: %s', tostring(self.loaderVehicle.rootNode))
    end
    
    if not self.loaderVehicle then
        CpUtil.info('=========================================')
        CpUtil.info('REFILL STRATEGY: No loader found yet')
        CpUtil.info('REFILL STRATEGY: Waiting at current position for loader to become available')
        CpUtil.info('REFILL STRATEGY: Will search again every %.1f seconds', self.loaderSearchInterval / 1000)
        CpUtil.info('=========================================')
        self.state = self.states.WAITING_FOR_LOADER
        self.loaderSearchTimer = 0
        self.lastSearchedFillType = fillTypeIndex
        self:setInfoText(InfoTextManager.WAITING_FOR_UNLOADER)  -- Show waiting message to user
        return
    end
    
    self:debug('Loader found: %s', CpUtil.getName(self.loaderVehicle))
    
    -- Check distance to loader
    local distanceToLoader = math.huge  -- Default fallback value
    if self.loaderVehicle and self.loaderVehicle.rootNode then
        distanceToLoader = calcDistanceFrom(self.vehicle:getAIDirectionNode(), self.loaderVehicle.rootNode)
        self:debug('Distance to loader: %.1f meters', distanceToLoader)
        
        -- If very close already (within 10m), skip pathfinding and start refilling
        if distanceToLoader < 10 then
            self:debug('Vehicle is already close to loader, starting refill directly')
            self.state = self.states.WAITING_FOR_REFILL
            self.refillTimer = 0
            self:prepareForRefill()
            return
        end
    else
        self:debug('Cannot calculate distance to loader (missing rootNode)')
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
    CpUtil.info('=========================================')
    CpUtil.info('PATHFINDING START: Beginning pathfinding to loader...')
    
    if self.pathfinder and self.pathfinder:isActive() then
        CpUtil.info('PATHFINDING START: WARNING - Pathfinder already active, skipping')
        CpUtil.info('=========================================')
        return
    end
    
    -- Validate that we still have a loader
    if not self.loaderVehicle then
        CpUtil.info('PATHFINDING START: ERROR - No loader vehicle set')
        CpUtil.info('PATHFINDING START: Aborting pathfinding')
        CpUtil.info('=========================================')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    if not self.dischargeNode then
        CpUtil.info('PATHFINDING START: ERROR - No discharge node set')
        CpUtil.info('PATHFINDING START: Aborting pathfinding')
        CpUtil.info('=========================================')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    CpUtil.info('PATHFINDING START: Loader: %s', CpUtil.getName(self.loaderVehicle))
    CpUtil.info('PATHFINDING START: Discharge node: %s', tostring(self.dischargeNode.node))
    
    -- Get target parameters
    local targetNode, alignLength, offsetX, loaderVehicle = self:getRefillTargetParameters()
    if not targetNode then
        CpUtil.info('PATHFINDING START: ERROR - Could not get refill target parameters')
        CpUtil.info('PATHFINDING START: This usually means the loader or discharge node became invalid')
        CpUtil.info('PATHFINDING START: Aborting pathfinding')
        CpUtil.info('=========================================')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    CpUtil.info('PATHFINDING START: [OK] Target parameters obtained')
    CpUtil.info('PATHFINDING START: Target node: %s', tostring(targetNode))
    CpUtil.info('PATHFINDING START: Align length: %.1f m', alignLength)
    CpUtil.info('PATHFINDING START: Offset X: %.1f m (%s side)', offsetX, offsetX < 0 and 'LEFT' or 'RIGHT')
    
    self.refillTargetNode = targetNode
    self.loaderVehicle = loaderVehicle
    
    -- Create alignment course (straight section parallel to loader)
    CpUtil.info('PATHFINDING START: Creating alignment course...')
    self.refillAlignCourse = Course.createFromNode(self.vehicle, self.refillTargetNode,
            offsetX, -alignLength + 1,
            -self.refillTargetOffset,
            1, false)
    CpUtil.info('PATHFINDING START: [OK] Alignment course created')
    
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
    if self.loaderVehicle then
        CpUtil.info('PATHFINDING START: Creating off-field penalty ignore area around loader')
        context:areaToIgnoreOffFieldPenalty(
                PathfinderUtil.NodeArea.createVehicleArea(self.loaderVehicle, 1.5 * SelfRefillHelper.maxDistanceFromField))
        CpUtil.info('PATHFINDING START: [OK] Penalty ignore area added')
    else
        CpUtil.info('PATHFINDING START: WARNING - No loader vehicle for penalty ignore area')
    end
    
    local maxIterations = PathfinderUtil.getMaxIterationsForFieldPolygon(self.vehicle:cpGetFieldPolygon())
    context:maxIterations(maxIterations)
    CpUtil.info('PATHFINDING START: Max iterations: %d', maxIterations)
    
    CpUtil.info('PATHFINDING START: Registering pathfinder callbacks...')
    self.pathfinderController:registerListeners(self,
            self.onPathfindingDoneToLoader,
            self.onPathfindingFailedToLoader,
            self.onPathfindingObstacleAtStart)
    
    CpUtil.info('PATHFINDING START: Starting pathfinder...')
    CpUtil.info('PATHFINDING START: Parameters:')
    CpUtil.info('PATHFINDING START:   - Target node: %s', tostring(self.refillTargetNode))
    CpUtil.info('PATHFINDING START:   - Offset X: %.1f', offsetX)
    CpUtil.info('PATHFINDING START:   - Z Start: %.1f', -alignLength)
    CpUtil.info('PATHFINDING START:   - Z End: %.1f', 3)
    
    local success = self.pathfinderController:findPathToNode(context, self.refillTargetNode, offsetX, -alignLength, 3)
    
    if success then
        CpUtil.info('PATHFINDING START: [OK] Pathfinder started successfully')
    else
        CpUtil.info('PATHFINDING START: ERROR - Pathfinder failed to start')
    end
    CpUtil.info('=========================================')
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
    
    CpUtil.info('=========================================')  
    CpUtil.info('PATHFINDING FAILED: Attempt %d', currentRetryAttempt)
    CpUtil.info('PATHFINDING FAILED: Trailer collisions only: %s', tostring(trailerCollisionsOnly))
    CpUtil.info('PATHFINDING FAILED: Fruit penalty: %.2f%%', fruitPenaltyNodePercent or 0)
    CpUtil.info('PATHFINDING FAILED: Off-field penalty: %.2f%%', offFieldPenaltyNodePercent or 0)
    
    if not wasLastRetry then
        -- Relax off-field penalty and retry
        if offFieldPenaltyRelaxingSteps[currentRetryAttempt] then
            CpUtil.info('PATHFINDING FAILED: Relaxing off-field penalty to %.2f and retrying',
                    offFieldPenaltyRelaxingSteps[currentRetryAttempt])
            lastContext:offFieldPenalty(offFieldPenaltyRelaxingSteps[currentRetryAttempt] * PathfinderContext.defaultOffFieldPenalty)
        end
        CpUtil.info('=========================================')  
        controller:retry(lastContext)
    else
        -- All retries exhausted
        CpUtil.info('PATHFINDING FAILED: All retries exhausted - NO PATH FOUND')
        CpUtil.info('PATHFINDING FAILED: Possible causes:')
        CpUtil.info('  - Chosen side is blocked (fence, building, etc.)')
        CpUtil.info('  - Vehicle position is bad for pathfinding')
        CpUtil.info('  - Loader is in inaccessible location')
        CpUtil.info('=========================================')  
        self.vehicle:stopCurrentAIJob(AIMessageCpErrorNoPathFound.new())
    end
end

--- Callback when obstacle is detected at start
function AIDriveStrategyRefillAtLoader:onPathfindingObstacleAtStart(controller, lastContext, maxDistance,
                                                                     trailerCollisionsOnly, fruitPenaltyNodePercent,
                                                                     offFieldPenaltyNodePercent)
    CpUtil.info('=========================================')  
    CpUtil.info('PATHFINDING: Obstacle detected at start position')
    CpUtil.info('PATHFINDING: Max distance: %.1f m', maxDistance or 0)
    CpUtil.info('PATHFINDING: Trailer collisions only: %s', tostring(trailerCollisionsOnly))
    
    if trailerCollisionsOnly then
        CpUtil.info('PATHFINDING: Ignoring trailer collisions and retrying')
        CpUtil.info('=========================================')  
        lastContext:ignoreTrailerAtStartRange(1.5 * self.turningRadius)
        controller:retry(lastContext)
    else
        CpUtil.info('PATHFINDING: Real obstacle at start - cannot proceed')
        CpUtil.info('PATHFINDING: Vehicle might be blocked or in bad position')
        CpUtil.info('=========================================')  
        self.vehicle:stopCurrentAIJob(AIMessageCpErrorNoPathFound.new())
    end
end

--- Start pathfinding to return to the fieldwork course
function AIDriveStrategyRefillAtLoader:startReturnToCourse()
    if not self.savedCourse or not self.savedWaypointIx then
        self:debug('ERROR: No saved course/waypoint for return, completing refill without return path')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    self:debug('Starting pathfinding to return to course at waypoint %d', self.savedWaypointIx)
    self.pathfindingStartedAt = g_currentMission.time
    self.state = self.states.WAITING_FOR_PATHFINDER
    
    local context = PathfinderContext(self.vehicle):allowReverse(self:getAllowReversePathfinding())
    context:ignoreFruit(not self.settings.avoidFruit:getValue())
    context:offFieldPenalty(PathfinderContext.defaultOffFieldPenalty)
    
    self.pathfinderController:registerListeners(self,
            self.onPathfindingDoneToReturnToCourse,
            self.onPathfindingFailedToReturnToCourse,
            self.onPathfindingObstacleAtStartForReturn)
    
    -- Find path back to the saved waypoint
    self.pathfinderController:findPathToWaypoint(context, self.savedCourse, 
            self.savedWaypointIx, 0, 0, 1)
end

--- Callback when pathfinding back to course is successful
function AIDriveStrategyRefillAtLoader:onPathfindingDoneToReturnToCourse(controller, success, course, goalNodeInvalid)
    if success then
        self:debug('Pathfinding back to course successful, %d waypoints (%d ms)', 
                course and #course or 0, 
                g_currentMission.time - (self.pathfindingStartedAt or 0))
        
        -- Adjust course for towed implements
        course:adjustForTowedImplements(2)
        
        -- Create alignment segment: straight line from end of pathfinder course 
        -- in the direction of the target waypoint to ensure proper alignment
        local lastX, _, lastZ = course:getWaypointPosition(course:getNumberOfWaypoints())
        local targetX, _, targetZ = self.savedCourse:getWaypointPosition(self.savedWaypointIx)
        local targetAngle = self.savedCourse:getWaypointAngleDeg(self.savedWaypointIx)
        
        -- Calculate a point beyond the target waypoint for alignment
        local fm = self:getFrontAndBackMarkers()
        local alignmentDistance = fm + 4
        local targetYRot = math.rad(targetAngle)
        local alignEndX = targetX + math.sin(targetYRot) * alignmentDistance
        local alignEndZ = targetZ + math.cos(targetYRot) * alignmentDistance
        
        -- Create straight alignment course from pathfinder end to beyond target waypoint
        local alignmentCourse = Course.createFromTwoWorldPositions(self.vehicle,
                lastX, lastZ, alignEndX, alignEndZ, 0, 0, 0, 3, false)
        course:append(alignmentCourse)
        
        self:debug('Added alignment course: %.1fm to straighten up', alignmentDistance)
        
        self.state = self.states.DRIVING_BACK_TO_COURSE
        self.ppc:setNormalLookaheadDistance()
        self:startCourse(course, 1)
        return true
    else
        self:debug('Pathfinding back to course failed, completing without return path')
        self.state = self.states.REFILL_COMPLETE
        return false
    end
end

--- Callback when pathfinding back to course fails
function AIDriveStrategyRefillAtLoader:onPathfindingFailedToReturnToCourse(controller, lastContext, wasLastRetry,
                                                                           currentRetryAttempt, trailerCollisionsOnly,
                                                                           fruitPenaltyNodePercent, offFieldPenaltyNodePercent)
    local offFieldPenaltyRelaxingSteps = { 0.5, 0.25, 0.1}
    
    if not wasLastRetry then
        -- Relax off-field penalty and retry
        if offFieldPenaltyRelaxingSteps[currentRetryAttempt] then
            self:debug('Return pathfinding failed, relaxing off-field penalty to %.2f and retrying',
                    offFieldPenaltyRelaxingSteps[currentRetryAttempt])
            lastContext:offFieldPenalty(offFieldPenaltyRelaxingSteps[currentRetryAttempt] * PathfinderContext.defaultOffFieldPenalty)
        end
        controller:retry(lastContext)
    else
        -- All retries exhausted - give up and complete without return path
        self:debug('Return pathfinding failed after all retries, completing without return path')
        self.state = self.states.REFILL_COMPLETE
    end
end

--- Callback when obstacle is detected at start for return path
function AIDriveStrategyRefillAtLoader:onPathfindingObstacleAtStartForReturn(controller, lastContext, maxDistance,
                                                                              trailerCollisionsOnly, fruitPenaltyNodePercent,
                                                                              offFieldPenaltyNodePercent)
    if trailerCollisionsOnly then
        self:debug('Return pathfinding detected obstacle at start (trailer collisions only), ignoring')
        lastContext:ignoreTrailerAtStartRange(1.5 * self.turningRadius)
        controller:retry(lastContext)
    else
        self:debug('Return pathfinding detected obstacle at start, completing without return path')
        self.state = self.states.REFILL_COMPLETE
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
    elseif self.startCalled and self.state == self.states.SEARCHING_FOR_LOADER then
        -- Strategy is being reused for a second refill - reset important flags
        if self.refillSucceeded or self.refillTimer > 0 then
            self:debug('=========================================')
            self:debug('REFILL STRATEGY: Resetting for repeated refill')
            self:debug('Previous state: refillSucceeded=%s, refillTimer=%.1fs', 
                tostring(self.refillSucceeded), self.refillTimer / 1000)
            
            -- Reset refill tracking
            self.refillSucceeded = false
            self.refillTimer = 0
            self.refillApproachRetryCount = 0
            
            -- Reset loader search
            self.loaderSearchTimer = 0
            
            -- Reset start flag so start() can be called again
            self.startCalled = false
            
            -- Clear previous loader references
            self.loaderVehicle = nil
            self.dischargeNode = nil
            self.refillTargetNode = nil
            
            self:debug('REFILL STRATEGY: All flags reset, ready for new refill cycle')
            self:debug('=========================================')
        end
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
            CpUtil.info('REFILL STRATEGY: WARNING - Missing field polygon or fill type, will retry')
            -- Don't abort, just wait and try again next cycle
            return
        end
        
        local fillTypeName = g_fillTypeManager:getFillTypeNameByIndex(fillTypeIndex)
        CpUtil.info('REFILL STRATEGY: Looking for: %s', fillTypeName)
        
        self.loaderVehicle, self.dischargeNode = SelfRefillHelper:findBestLoader(fieldPolygon, self.vehicle, fillTypeIndex)
        
        if self.loaderVehicle then
            -- Found a loader! Clear waiting message and continue with normal flow
            self:clearInfoText(InfoTextManager.WAITING_FOR_UNLOADER)
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
            CpUtil.info('REFILL STRATEGY: Vehicle is %.1fm away', distanceToLoader)
            CpUtil.info('REFILL STRATEGY: >>> Starting folding process before pathfinding <<<')
            
            -- Start folding implements first, then pathfinding will follow automatically
            self:startFoldingForPathfinding()
            CpUtil.info('=========================================')
        else
            -- Still no loader found - continue waiting
            if g_updateLoopIndex % 300 == 0 then  -- Log every ~10 seconds
                local fillTypeName = g_fillTypeManager:getFillTypeNameByIndex(fillTypeIndex)
                CpUtil.info('REFILL STRATEGY: Still waiting for loader (%s)...', fillTypeName)
            end
            CpUtil.info('REFILL STRATEGY: Still no loader found, will check again in %.1f seconds', 
                self.loaderSearchInterval / 1000)
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

--- Indicates whether this strategy has completed its work
---@return boolean true when refilling is complete and strategy should finish
function AIDriveStrategyRefillAtLoader:isDone()
    local done = self.state == self.states.REFILL_COMPLETE and self.refillSucceeded
    if done then
        CpUtil.info('DEBUG: isDone() returning true - strategy complete')
    end
    return done
end

function AIDriveStrategyRefillAtLoader:getDriveData(dt, vX, vY, vZ)
    self:updateLowFrequencyImplementControllers()
    
    -- Debug status reporting every 10 seconds 
    if g_currentMission.time > self.lastDebugUpdate + 10000 then
        self.lastDebugUpdate = g_currentMission.time
        CpUtil.info('DEBUG STATUS: State=%s, RefillComplete=%s, DischargeCleaned=%s, LoaderVehicle=%s', 
            self:getStateAsString(), 
            tostring(self.state == self.states.REFILL_COMPLETE),
            tostring(self.dischargeCleaned),
            self.loaderVehicle and CpUtil.getName(self.loaderVehicle) or 'nil')
        
        -- Check for problematic discharge references
        if self.vehicle and self.vehicle.spec_dischargeable then
            local spec = self.vehicle.spec_dischargeable
            if spec.currentDischargeNode and spec.currentDischargeNode.dischargeObject then
                CpUtil.info('DEBUG WARNING: Vehicle still has discharge object: %s', 
                    tostring(spec.currentDischargeNode.dischargeObject))
            end
            if spec.dischargeNodes then
                for i, node in pairs(spec.dischargeNodes) do
                    if node.dischargeObject then
                        CpUtil.info('DEBUG WARNING: Discharge node %d still has object: %s', 
                            i, tostring(node.dischargeObject))
                    end
                end
            end
        end
    end
    
    -- Early detection and cleanup
    if g_updateLoopIndex % 10 == 0 then  -- Check every 10th update for early completion detection
        if self:isDone() and not self.dischargeCleaned then
            CpUtil.info('DEBUG: Strategy done but discharge not cleaned - emergency cleanup!')
            self:cleanupDischargeState()
        end
    end
    
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
        local distanceToLoader = math.huge
        if self.loaderVehicle and self.loaderVehicle.rootNode then
            distanceToLoader = calcDistanceFrom(self.vehicle:getAIDirectionNode(), 
                self.loaderVehicle.rootNode)
        end
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
        
    elseif self.state == self.states.DRIVING_BACK_TO_COURSE then
        -- Driving back to the fieldwork course with pathfinder-generated path
        if not moveForwards then
            local maxSpeed
            gx, gz, maxSpeed = self:getReverseDriveData()
            self:setMaxSpeed(maxSpeed)
        else
            gx, _, gz = self.ppc:getGoalPointPosition()
            self:setMaxSpeed(self.settings.fieldSpeed:getValue())
        end
        
        -- Keep implements raised while returning to course
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        
        -- Check if we've reached the end of the return course
        if self.ppc:getCourse():isCloseToLastWaypoint(3) then
            CpUtil.info('=========================================')
            CpUtil.info('REFILL STRATEGY: ✓✓✓ RETURNED TO COURSE!')
            CpUtil.info('REFILL STRATEGY: Now at saved waypoint %d', self.savedWaypointIx)
            CpUtil.info('REFILL STRATEGY: Completing refill process...')
            CpUtil.info('=========================================')
            self.state = self.states.REFILL_COMPLETE
        end
        
    elseif self.state == self.states.REFILL_COMPLETE then
        self:setMaxSpeed(0)
        gx, gz = 0, 0
        -- Ensure immediate cleanup when entering complete state
        if not self.dischargeCleaned then
            CpUtil.info('DEBUG: REFILL_COMPLETE state - calling cleanup...')
            self:cleanupDischargeState()
            self.dischargeCleaned = true
        end
        
        -- CONTINUOUS MONITORING: Check for lingering discharge references every 5 updates
        if g_updateLoopIndex % 5 == 0 then
            self:monitorDischargeReferences()
        end
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
        CpUtil.info('===========================================')
        CpUtil.info('REFILL STRATEGY: ✓✓✓ REFILLING COMPLETE!')
        CpUtil.info('REFILL STRATEGY: Tank is now at 95%% or higher')
        CpUtil.info('DEBUG: About to call finishRefilling...')
        self.refillSucceeded = true  -- Ensure flag is set
        self:finishRefilling()
        -- Immediate cleanup to prevent entity errors
        if not self.dischargeCleaned then
            CpUtil.info('DEBUG: Discharge not yet cleaned, calling cleanup...')
            self:cleanupDischargeState()
        else
            CpUtil.info('DEBUG: Discharge already cleaned')
        end
        
        -- Start pathfinding back to course instead of completing immediately
        CpUtil.info('REFILL STRATEGY: Starting pathfinding back to course...')
        self:startReturnToCourse()
        CpUtil.info('===========================================')
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
        -- Immediate cleanup to prevent entity errors
        if not self.dischargeCleaned then
            self:cleanupDischargeState()
        end
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

--- Clean up discharge state to prevent entity access errors
function AIDriveStrategyRefillAtLoader:cleanupDischargeState()
    if self.dischargeCleaned then
        CpUtil.info('DEBUG: cleanupDischargeState() - Already cleaned up, skipping')
        return  -- Already cleaned up
    end
    
    CpUtil.info('===========================================')
    CpUtil.info('DEBUG: Starting discharge state cleanup...')
    CpUtil.info('DEBUG: Vehicle: %s', CpUtil.getName(self.vehicle))
    self.dischargeCleaned = true
    
    -- STEP 1: Turn OFF discharge state FIRST to prevent game from accessing entities
    CpUtil.info('DEBUG: Step 1 - Turning OFF discharge states...')
    if self.vehicle and self.vehicle.spec_dischargeable then
        if self.vehicle.setDischargeState then
            self.vehicle:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
            CpUtil.info('DEBUG: Vehicle discharge state set to OFF')
        end
    end
    
    -- Also turn OFF for all implements
    if self.vehicle then
        for i, implement in pairs(self.vehicle:getAttachedImplements()) do
            local object = implement.object
            if object and object.spec_dischargeable then
                if object.setDischargeState then
                    object:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
                    CpUtil.info('DEBUG: Implement %d discharge state set to OFF', i)
                end
            end
        end
    end
    
    -- STEP 2: Now clear all entity references
    CpUtil.info('DEBUG: Step 2 - Clearing entity references...')
    
    -- Reset discharge state to prevent entity access errors
    if self.vehicle and self.vehicle.spec_dischargeable then
        local spec = self.vehicle.spec_dischargeable
        CpUtil.info('DEBUG: Vehicle has dischargeable spec')
        
        -- Clear any cached discharge targets
        if spec.currentDischargeNode then
            CpUtil.info('DEBUG: Found currentDischargeNode, clearing discharge references...')
            CpUtil.info('DEBUG: Before cleanup - dischargeObject: %s, dischargeFillUnitIndex: %s', 
                tostring(spec.currentDischargeNode.dischargeObject), 
                tostring(spec.currentDischargeNode.dischargeFillUnitIndex))
            spec.currentDischargeNode.dischargeObject = nil
            spec.currentDischargeNode.dischargeFillUnitIndex = nil 
            spec.currentDischargeNode.dischargeHit = false
            
            -- NUCLEAR: Clear ALL possible cached references including node IDs
            spec.currentDischargeNode.dischargeObjectInfo = nil
            spec.currentDischargeNode.dischargeTargetObject = nil
            spec.currentDischargeNode.lastDischargeObject = nil
            spec.currentDischargeNode.dischargeFailedReason = nil
            spec.currentDischargeNode.dischargeFailedReasonShown = nil
            spec.currentDischargeNode.dischargeFailedObjectId = nil
            spec.currentDischargeNode.lastDischargeObjectId = nil
            spec.currentDischargeNode.dischargeHitObject = nil
            spec.currentDischargeNode.dischargeHitObjectUnitIndex = nil
            spec.currentDischargeNode.dischargeHitObjectId = nil
            spec.currentDischargeNode.dischargeHitTerrain = nil
            spec.currentDischargeNode.lastDischargeDistanceCheck = nil
            
            if spec.currentDischargeNode.raycastInfo then
                spec.currentDischargeNode.raycastInfo.hitObject = nil
                spec.currentDischargeNode.raycastInfo.hitObjectId = nil
                spec.currentDischargeNode.raycastInfo.hitTerrain = nil
            end
            
            CpUtil.info('DEBUG: currentDischargeNode cleared')
        else
            CpUtil.info('DEBUG: No currentDischargeNode found')
        end
        
        -- Clear discharge node discharge objects to prevent stale references
        if spec.dischargeNodes then
            CpUtil.info('DEBUG: Found %d discharge nodes, clearing all...', #spec.dischargeNodes)
            for i, dischargeNode in pairs(spec.dischargeNodes) do
                if dischargeNode.dischargeObject then
                    CpUtil.info('DEBUG: Node %d had dischargeObject %s, clearing...', i, tostring(dischargeNode.dischargeObject))
                end
                dischargeNode.dischargeObject = nil
                dischargeNode.dischargeFillUnitIndex = nil
                dischargeNode.dischargeHit = false
                
                -- NUCLEAR: Clear ALL possible cached references including node IDs
                dischargeNode.dischargeObjectInfo = nil
                dischargeNode.dischargeTargetObject = nil
                dischargeNode.lastDischargeObject = nil
                dischargeNode.dischargeFailedReason = nil
                dischargeNode.dischargeFailedReasonShown = nil
                dischargeNode.dischargeFailedObjectId = nil
                dischargeNode.lastDischargeObjectId = nil
                dischargeNode.dischargeHitObject = nil
                dischargeNode.dischargeHitObjectUnitIndex = nil
                dischargeNode.dischargeHitObjectId = nil
                dischargeNode.dischargeHitTerrain = nil
                dischargeNode.lastDischargeDistanceCheck = nil
                
                if dischargeNode.raycastInfo then
                    dischargeNode.raycastInfo.hitObject = nil
                    dischargeNode.raycastInfo.hitObjectId = nil
                    dischargeNode.raycastInfo.hitTerrain = nil
                end
            end
            CpUtil.info('DEBUG: All discharge nodes cleared')
        else
            CpUtil.info('DEBUG: No discharge nodes found')
        end
    else
        CpUtil.info('DEBUG: Vehicle has no dischargeable spec')
    end
    
    -- Also check implements for discharge state
    if self.vehicle then
        local implements = self.vehicle:getAttachedImplements()
        CpUtil.info('DEBUG: Checking %d attached implements...', #implements)
        for i, implement in pairs(implements) do
            local object = implement.object
            if object and object.spec_dischargeable then
                CpUtil.info('DEBUG: Implement %d (%s) has dischargeable spec', i, CpUtil.getName(object))
                local spec = object.spec_dischargeable
                if spec.currentDischargeNode then
                    CpUtil.info('DEBUG: Implement %d currentDischargeNode - dischargeObject: %s', i,
                        tostring(spec.currentDischargeNode.dischargeObject))
                    spec.currentDischargeNode.dischargeObject = nil
                    spec.currentDischargeNode.dischargeFillUnitIndex = nil 
                    spec.currentDischargeNode.dischargeHit = false
                    
                    -- NUCLEAR: Clear ALL possible cached references including node IDs
                    spec.currentDischargeNode.dischargeObjectInfo = nil
                    spec.currentDischargeNode.dischargeTargetObject = nil
                    spec.currentDischargeNode.lastDischargeObject = nil
                    spec.currentDischargeNode.dischargeFailedReason = nil
                    spec.currentDischargeNode.dischargeFailedReasonShown = nil
                    spec.currentDischargeNode.dischargeFailedObjectId = nil
                    spec.currentDischargeNode.lastDischargeObjectId = nil
                    spec.currentDischargeNode.dischargeHitObject = nil
                    spec.currentDischargeNode.dischargeHitObjectUnitIndex = nil
                    spec.currentDischargeNode.dischargeHitObjectId = nil
                    spec.currentDischargeNode.dischargeHitTerrain = nil
                    spec.currentDischargeNode.lastDischargeDistanceCheck = nil
                    
                    if spec.currentDischargeNode.raycastInfo then
                        spec.currentDischargeNode.raycastInfo.hitObject = nil
                        spec.currentDischargeNode.raycastInfo.hitObjectId = nil
                        spec.currentDischargeNode.raycastInfo.hitTerrain = nil
                    end
                    CpUtil.info('DEBUG: Implement %d currentDischargeNode cleared', i)
                end
                if spec.dischargeNodes then
                    CpUtil.info('DEBUG: Implement %d has %d discharge nodes', i, #spec.dischargeNodes)
                    for j, dischargeNode in pairs(spec.dischargeNodes) do
                        if dischargeNode.dischargeObject then
                            CpUtil.info('DEBUG: Implement %d node %d had dischargeObject %s', i, j, 
                                tostring(dischargeNode.dischargeObject))
                        end
                        dischargeNode.dischargeObject = nil
                        dischargeNode.dischargeFillUnitIndex = nil
                        dischargeNode.dischargeHit = false
                        
                        -- NUCLEAR: Clear ALL possible cached references including node IDs
                        dischargeNode.dischargeObjectInfo = nil
                        dischargeNode.dischargeTargetObject = nil
                        dischargeNode.lastDischargeObject = nil
                        dischargeNode.dischargeFailedReason = nil
                        dischargeNode.dischargeFailedReasonShown = nil
                        dischargeNode.dischargeFailedObjectId = nil
                        dischargeNode.lastDischargeObjectId = nil
                        dischargeNode.dischargeHitObject = nil
                        dischargeNode.dischargeHitObjectUnitIndex = nil
                        dischargeNode.dischargeHitObjectId = nil
                        dischargeNode.dischargeHitTerrain = nil
                        dischargeNode.lastDischargeDistanceCheck = nil
                        
                        if dischargeNode.raycastInfo then
                            dischargeNode.raycastInfo.hitObject = nil
                            dischargeNode.raycastInfo.hitObjectId = nil
                            dischargeNode.raycastInfo.hitTerrain = nil
                        end
                    end
                end
            else
                CpUtil.info('DEBUG: Implement %d (%s) has no dischargeable spec', i, object and CpUtil.getName(object) or 'nil')
            end
        end
    end
    
    CpUtil.info('DEBUG: Discharge state cleanup completed')
    
    -- STEP 3: Clear spec-level cached discharge information
    CpUtil.info('DEBUG: Step 3 - Clearing spec-level discharge cache...')
    if self.vehicle and self.vehicle.spec_dischargeable then
        local spec = self.vehicle.spec_dischargeable
        -- Clear any spec-level cached objects/distances
        spec.dischargeHitObject = nil
        spec.dischargeHitObjectUnitIndex = nil
        spec.lastDischargeObject = nil
        spec.lastDischargeDistance = nil
        CpUtil.info('DEBUG: Vehicle spec-level discharge cache cleared')
    end
    
    if self.vehicle then
        for i, implement in pairs(self.vehicle:getAttachedImplements()) do
            local object = implement.object
            if object and object.spec_dischargeable then
                local spec = object.spec_dischargeable
                spec.dischargeHitObject = nil
                spec.dischargeHitObjectUnitIndex = nil
                spec.lastDischargeObject = nil
                spec.lastDischargeDistance = nil
                CpUtil.info('DEBUG: Implement %d spec-level discharge cache cleared', i)
            end
        end
    end
    
    -- STEP 4: Disable discharge triggers (additional safety)
    if self.vehicle and self.vehicle.spec_dischargeable then
        CpUtil.info('DEBUG: Disabling discharge triggers...')
        local spec = self.vehicle.spec_dischargeable
        if spec.dischargeTriggers then
            for _, trigger in pairs(spec.dischargeTriggers) do
                trigger.isEnabled = false
            end
        end
    end
    
    -- Also for implements
    if self.vehicle then
        for i, implement in pairs(self.vehicle:getAttachedImplements()) do
            local object = implement.object
            if object and object.spec_dischargeable then
                local spec = object.spec_dischargeable
                if spec.dischargeTriggers then
                    for _, trigger in pairs(spec.dischargeTriggers) do
                        trigger.isEnabled = false
                    end
                end
            end
        end
    end
    
    CpUtil.info('===========================================')
end

--- Continuously monitor and clear any problematic discharge references
function AIDriveStrategyRefillAtLoader:monitorDischargeReferences()
    local foundProblem = false
    
    -- Ensure discharge state is OFF
    if self.vehicle and self.vehicle.spec_dischargeable then
        local spec = self.vehicle.spec_dischargeable
        if spec.currentDischargeState ~= Dischargeable.DISCHARGE_STATE_OFF then
            CpUtil.info('DEBUG MONITOR: Discharge state not OFF, forcing OFF...')
            if self.vehicle.setDischargeState then
                self.vehicle:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
            end
            foundProblem = true
        end
        
        -- Check spec-level caching
        if spec.dischargeHitObject or spec.lastDischargeObject then
            CpUtil.info('DEBUG MONITOR: Found spec-level discharge cache, clearing...')
            spec.dischargeHitObject = nil
            spec.dischargeHitObjectUnitIndex = nil
            spec.lastDischargeObject = nil
            spec.lastDischargeDistance = nil
            foundProblem = true
        end
        
        if spec.currentDischargeNode and spec.currentDischargeNode.dischargeObject then
            CpUtil.info('DEBUG MONITOR: Found lingering vehicle discharge reference, clearing...')
            spec.currentDischargeNode.dischargeObject = nil
            spec.currentDischargeNode.dischargeFillUnitIndex = nil
            spec.currentDischargeNode.dischargeObjectInfo = nil
            spec.currentDischargeNode.dischargeTargetObject = nil
            spec.currentDischargeNode.lastDischargeObject = nil
            spec.currentDischargeNode.dischargeFailedObjectId = nil
            spec.currentDischargeNode.lastDischargeObjectId = nil
            spec.currentDischargeNode.dischargeHitObject = nil
            spec.currentDischargeNode.dischargeHitObjectUnitIndex = nil
            spec.currentDischargeNode.dischargeHitObjectId = nil
            spec.currentDischargeNode.dischargeHitTerrain = nil
            spec.currentDischargeNode.lastDischargeDistanceCheck = nil
            foundProblem = true
        end
        if spec.dischargeNodes then
            for i, node in pairs(spec.dischargeNodes) do
                if node.dischargeObject then
                    CpUtil.info('DEBUG MONITOR: Found lingering vehicle discharge node %d reference, clearing...', i)
                    node.dischargeObject = nil
                    node.dischargeFillUnitIndex = nil
                    node.dischargeObjectInfo = nil
                    node.dischargeTargetObject = nil
                    node.lastDischargeObject = nil
                    node.dischargeFailedObjectId = nil
                    node.lastDischargeObjectId = nil
                    node.dischargeHitObject = nil
                    node.dischargeHitObjectUnitIndex = nil
                    node.dischargeHitObjectId = nil
                    node.dischargeHitTerrain = nil
                    node.lastDischargeDistanceCheck = nil
                    foundProblem = true
                end
            end
        end
    end
    
    -- Check implements
    if self.vehicle then
        for i, implement in pairs(self.vehicle:getAttachedImplements()) do
            local object = implement.object
            if object and object.spec_dischargeable then
                local spec = object.spec_dischargeable
                
                -- Ensure implement discharge state is OFF
                if spec.currentDischargeState ~= Dischargeable.DISCHARGE_STATE_OFF then
                    CpUtil.info('DEBUG MONITOR: Implement %d discharge state not OFF, forcing OFF...', i)
                    if object.setDischargeState then
                        object:setDischargeState(Dischargeable.DISCHARGE_STATE_OFF, true)
                    end
                    foundProblem = true
                end
                
                -- Check implement spec-level caching
                if spec.dischargeHitObject or spec.lastDischargeObject then
                    CpUtil.info('DEBUG MONITOR: Found implement %d spec-level discharge cache, clearing...', i)
                    spec.dischargeHitObject = nil
                    spec.dischargeHitObjectUnitIndex = nil
                    spec.lastDischargeObject = nil
                    spec.lastDischargeDistance = nil
                    foundProblem = true
                end
                
                if spec.currentDischargeNode and spec.currentDischargeNode.dischargeObject then
                    CpUtil.info('DEBUG MONITOR: Found lingering implement %d discharge reference, clearing...', i)
                    spec.currentDischargeNode.dischargeObject = nil
                    spec.currentDischargeNode.dischargeFillUnitIndex = nil
                    spec.currentDischargeNode.dischargeObjectInfo = nil
                    spec.currentDischargeNode.dischargeTargetObject = nil
                    spec.currentDischargeNode.lastDischargeObject = nil
                    spec.currentDischargeNode.dischargeFailedObjectId = nil
                    spec.currentDischargeNode.lastDischargeObjectId = nil
                    spec.currentDischargeNode.dischargeHitObject = nil
                    spec.currentDischargeNode.dischargeHitObjectUnitIndex = nil
                    spec.currentDischargeNode.dischargeHitObjectId = nil
                    spec.currentDischargeNode.dischargeHitTerrain = nil
                    spec.currentDischargeNode.lastDischargeDistanceCheck = nil
                    foundProblem = true
                end
                if spec.dischargeNodes then
                    for j, node in pairs(spec.dischargeNodes) do
                        if node.dischargeObject then
                            CpUtil.info('DEBUG MONITOR: Found lingering implement %d node %d reference, clearing...', i, j)
                            node.dischargeObject = nil
                            node.dischargeFillUnitIndex = nil
                            node.dischargeObjectInfo = nil
                            node.dischargeTargetObject = nil
                            node.lastDischargeObject = nil
                            node.dischargeFailedObjectId = nil
                            node.lastDischargeObjectId = nil
                            node.dischargeHitObject = nil
                            node.dischargeHitObjectUnitIndex = nil
                            node.dischargeHitObjectId = nil
                            node.dischargeHitTerrain = nil
                            node.lastDischargeDistanceCheck = nil
                            foundProblem = true
                        end
                    end
                end
            end
        end
    end
    
    if foundProblem then
        CpUtil.info('DEBUG MONITOR: Cleared lingering discharge references')
    end
end

function AIDriveStrategyRefillAtLoader:finishRefilling()
    CpUtil.info('DEBUG: finishRefilling() called')
    
    self:cleanupDischargeState()
    
    -- Disable refilling mode for all relevant controllers
    if self.controllers then
        local numControllers = 0
        for _ in pairs(self.controllers) do
            numControllers = numControllers + 1
        end
        CpUtil.info('DEBUG: Stopping %d controllers...', numControllers)
        for _, controller in pairs(self.controllers) do
            if controller.onStopRefilling then
                controller:onStopRefilling()
            end
        end
    else
        CpUtil.info('DEBUG: No controllers to stop')
    end
    
    -- Don't destroy refillTargetNode - it's a reference to the loader's discharge node, not ours!
    -- Just clear the reference
    if self.refillTargetNode then
        CpUtil.info('DEBUG: Clearing refill target node reference')
        self.refillTargetNode = nil
    end
    
    self.loaderVehicle = nil
    self.dischargeNode = nil
    CpUtil.info('DEBUG: finishRefilling() completed')
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
            return false, true
        end
        local fillUnits = object:getFillUnits()
        if not fillUnits then
            return false, true
        end

        local objectHasRelevantUnit = false
        local objectAllUnitsFilled = true

        for fillUnitIndex, _ in pairs(fillUnits) do
            local capacity = object:getFillUnitCapacity(fillUnitIndex)
            if capacity and capacity > 0 and unitSupportsRequiredFillType(object, fillUnitIndex) then
                objectHasRelevantUnit = true
                local fillLevel = object:getFillUnitFillLevel(fillUnitIndex)
                local fillLevelPercent = fillLevel / capacity
                if fillLevelPercent < 0.95 then
                    objectAllUnitsFilled = false
                    self:debugSparse('Fallback refill unit %s:%d at %.1f percent (%.0f/%.0f)',
                        CpUtil.getName(object), fillUnitIndex, fillLevelPercent * 100, fillLevel, capacity)
                end
            end
        end

        return objectHasRelevantUnit, objectAllUnitsFilled
    end

    local visited = {}
    local function traverseObject(object)
        if not object or visited[object] then
            return
        end
        visited[object] = true

        local hasUnit, allFilled = evaluateObject(object)
        if hasUnit then
            fallbackHasRelevantUnit = true
            fallbackAllRelevantUnitsFilled = fallbackAllRelevantUnitsFilled and allFilled
        end

        if object.getAttachedImplements then
            for _, impl in pairs(object:getAttachedImplements()) do
                traverseObject(impl and (impl.object or impl))
            end
        end

        if object.getAttachedAIImplements then
            for _, impl in pairs(object:getAttachedAIImplements()) do
                traverseObject(impl and (impl.object or impl))
            end
        end
    end

    traverseObject(self.vehicle)

    if fallbackHasRelevantUnit and fallbackAllRelevantUnitsFilled then
        self:debug('REFILL STRATEGY: Recursive completion detection confirmed all relevant units >= 95%%')
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
