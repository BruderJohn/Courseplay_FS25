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

Drive strategy for driving to a loader/filler vehicle and refilling.

]]--

---@class AIDriveStrategyRefillAtLoader : AIDriveStrategyCourse
---@field job CpAIJobFieldWork
AIDriveStrategyRefillAtLoader = CpObject(AIDriveStrategyCourse)

AIDriveStrategyRefillAtLoader.myStates = {
    SEARCHING_FOR_LOADER = {},
    WAITING_FOR_LOADER = {},
    WAITING_FOR_FOLD = {},  -- Warten bis Implements zusammengeklappt sind
    DRIVING_TO_LOADER_PATHFINDING = {},
    DRIVING_TO_LOADER = {},
    WAITING_FOR_REFILL = {},
    REFILL_COMPLETE = {},
}

-- minimum distance to drive to loader
AIDriveStrategyRefillAtLoader.minDistanceToLoader = 5
AIDriveStrategyRefillAtLoader.minDistanceToRefillTarget = 8
AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode = 3.5

function AIDriveStrategyRefillAtLoader:init(task, job)
    AIDriveStrategyCourse.init(self, task, job)
    AIDriveStrategyCourse.initStates(self, AIDriveStrategyRefillAtLoader.myStates)
    self.state = self.states.SEARCHING_FOR_LOADER
    self.debugChannel = CpDebug.DBG_FIELDWORK
    self.refillTimer = 0
    self.refillTimeout = 30000 -- 30 seconds timeout for refilling
    self.loaderVehicle = nil
    self.dischargeNode = nil
    self.implementsRaised = false
    self.startCalled = false  -- Track if start() has been called
    self.fieldPolygon = nil  -- Will be set by task
    self.islandPolygons = nil
    self.pathfindingStartedAt = 0  -- Track when pathfinding started
    self.loaderSearchTimer = 0  -- Timer for retrying loader search
    self.loaderSearchInterval = 5000  -- Retry every 5 seconds
    self.lastSearchedFillType = nil  -- Track what we're searching for
    self.targetRefillPosition = nil
    self.refillAlignCourse = nil
    self.refillApproachNode = nil
    self.refillApproachRetryCount = 0
    self.foldStartedAt = 0
    self.foldTimeoutMs = 12000
    self.nextFoldCommandAt = 0
    self.foldCommandRetryMs = 1500
    self.nextFoldTimeoutLogAt = 0
end

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

function AIDriveStrategyRefillAtLoader:delete()
    if self.refillApproachNode then
        CpUtil.destroyNode(self.refillApproachNode)
        self.refillApproachNode = nil
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
    -- Raise implements before starting to drive to loader
    self:raiseImplements()
    -- Start will be called later in the update cycle
end

--- Set the field polygon to use for finding loaders
---@param fieldPolygon table [{x, y, z}] field boundary vertices
---@param islandPolygons table|nil [[{x, y, z}]] array of island polygons
function AIDriveStrategyRefillAtLoader:setFieldPolygon(fieldPolygon, islandPolygons)
    self:debug('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    self:debug('setFieldPolygon() CALLED')
    self:debug('fieldPolygon parameter is: %s', fieldPolygon and 'NOT NIL' or 'NIL')
    if fieldPolygon then
        self:debug('fieldPolygon has %d vertices', #fieldPolygon)
        for i = 1, math.min(3, #fieldPolygon) do
            self:debug('  Vertex %d: x=%.1f, z=%.1f', i, fieldPolygon[i].x, fieldPolygon[i].z)
        end
    end
    self:debug('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    
    self.fieldPolygon = fieldPolygon
    self.islandPolygons = islandPolygons
    
    self:debug('After assignment: self.fieldPolygon is %s', self.fieldPolygon and 'NOT NIL' or 'NIL')
end

function AIDriveStrategyRefillAtLoader:start()
    -- Prevent multiple calls to start()
    if self.startCalled then
        return
    end
    self.startCalled = true
    
    CpUtil.info('=========================================')
    CpUtil.info('REFILL STRATEGY: Starting refill at loader')
    CpUtil.info('REFILL STRATEGY: Checking field polygon...')
    
    self.refillSucceeded = false  -- Track if refill actually worked
    
    -- Use the field polygon passed from the task, not from vehicle
    local fieldPolygon = self.fieldPolygon
    
    if fieldPolygon then
        CpUtil.info('REFILL STRATEGY: ✓ Field polygon available with %d vertices', #fieldPolygon)
        for i = 1, math.min(3, #fieldPolygon) do
            CpUtil.info('REFILL STRATEGY:   Vertex %d: x=%.1f, z=%.1f', i, fieldPolygon[i].x, fieldPolygon[i].z)
        end
    else
        CpUtil.info('REFILL STRATEGY: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        CpUtil.info('REFILL STRATEGY: CRITICAL ERROR - No field polygon available')
        CpUtil.info('REFILL STRATEGY: Cannot search for loader without field boundary')
        CpUtil.info('REFILL STRATEGY: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        CpUtil.info('REFILL STRATEGY: ')
        CpUtil.info('REFILL STRATEGY: POSSIBLE SOLUTIONS:')
        CpUtil.info('REFILL STRATEGY:   1. Stop Courseplay and start again')
        CpUtil.info('REFILL STRATEGY:   2. Manually drive to tanker for refilling')
        CpUtil.info('REFILL STRATEGY:   3. Wait 30 seconds for automatic retry')
        CpUtil.info('REFILL STRATEGY: ')
        CpUtil.info('REFILL STRATEGY: The vehicle will STOP')
        CpUtil.info('REFILL STRATEGY: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    
    -- Get the fill type we need
    CpUtil.info('REFILL STRATEGY: Determining required fill type...')
    local fillTypeIndex = self:getRequiredFillType()
    if not fillTypeIndex then
        CpUtil.info('REFILL STRATEGY: ERROR - Could not determine required fill type')
        CpUtil.info('REFILL STRATEGY: Check if vehicle has a sprayer/manure spreader attached')
        self.state = self.states.REFILL_COMPLETE
        return
    end
    local fillTypeName = g_fillTypeManager:getFillTypeNameByIndex(fillTypeIndex)
    CpUtil.info('REFILL STRATEGY: ✓ Required fill type: %s (index %d)', fillTypeName, fillTypeIndex)
    
    -- Find the best loader
    CpUtil.info('REFILL STRATEGY: Searching for loader within 30m of field edge...')
    self.loaderVehicle, self.dischargeNode = SelfRefillHelper:findBestLoader(fieldPolygon, self.vehicle, fillTypeIndex)
    
    if not self.loaderVehicle then
        CpUtil.info('REFILL STRATEGY: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        CpUtil.info('REFILL STRATEGY: WARNING - No suitable loader found yet!')
        CpUtil.info('REFILL STRATEGY: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        CpUtil.info('REFILL STRATEGY: ')
        CpUtil.info('REFILL STRATEGY: CHECKLIST:')
        CpUtil.info('REFILL STRATEGY:   □ Tanker is within 30m of field edge')
        CpUtil.info('REFILL STRATEGY:   □ Tanker is STOPPED (not moving)')
        CpUtil.info('REFILL STRATEGY:   □ Tanker is NOT controlled by Courseplay')
        CpUtil.info('REFILL STRATEGY:   □ Tanker contains: %s', fillTypeName)
        CpUtil.info('REFILL STRATEGY:   □ Tanker has at least 100 liters')
        CpUtil.info('REFILL STRATEGY: ')
        CpUtil.info('REFILL STRATEGY: Vehicle will WAIT and search every 5 seconds')
        CpUtil.info('REFILL STRATEGY: Place a suitable tanker to continue automatically')
        CpUtil.info('REFILL STRATEGY: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        self.state = self.states.WAITING_FOR_LOADER
        self.loaderSearchTimer = 0
        self.lastSearchedFillType = fillTypeIndex
        return
    end
    
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
    CpUtil.info('REFILL STRATEGY: Vehicle is %.1fm away, need to fold implements first...', distanceToLoader)
    CpUtil.info('REFILL STRATEGY: >>> Folding implements (Schleppschlauch etc.) for better pathfinding <<<')
    
    -- WICHTIG: Implements zusammenklappen BEVOR Pfad berechnet wird
    self:raiseImplements()
    self:requestFoldAllForRefill()
    self.state = self.states.WAITING_FOR_FOLD
    self.foldStartedAt = g_currentMission.time
    self.targetLoaderData = {
        fieldPolygon = fieldPolygon,
        fillTypeIndex = fillTypeIndex,
        distanceToLoader = distanceToLoader
    }
    CpUtil.info('REFILL STRATEGY: Waiting for implements to fold...')
    CpUtil.info('=========================================')
    return
end

--- Start pathfinding after implements are folded
function AIDriveStrategyRefillAtLoader:startPathfindingAfterFold()
    local fieldPolygon = self.targetLoaderData.fieldPolygon
    local fillTypeIndex = self.targetLoaderData.fillTypeIndex
    local distanceToLoader = self.targetLoaderData.distanceToLoader
    
    CpUtil.info('=========================================')
    CpUtil.info('REFILL STRATEGY: ✓ Implements folded, now calculating path...')
    CpUtil.info('REFILL STRATEGY: Distance to loader: %.1fm', distanceToLoader)
    
    local targetNode, alignLength, offsetX = SelfRefillHelper:getLoaderTargetParameters(
        fieldPolygon, self.vehicle, fillTypeIndex, self.loaderVehicle, self.dischargeNode)

    if not targetNode then
        CpUtil.info('REFILL STRATEGY: ERROR - Could not calculate loader target parameters')
        CpUtil.info('REFILL STRATEGY: Falling back to waiting for manual positioning')
        self.state = self.states.WAITING_FOR_REFILL
        self.refillTimer = 0
        return
    end

    CpUtil.info('REFILL STRATEGY: Target parameters: alignLength %.1f, offsetX %.1f', alignLength, offsetX)
    local targetX, targetY, targetZ = getWorldTranslation(targetNode)
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle:getAIDirectionNode())
    local localTargetX, _, localTargetZ = localToLocal(targetNode, self.vehicle:getAIDirectionNode(), 0, 0, 0)
    CpUtil.info('REFILL STRATEGY: Target node world: (%.1f, %.1f, %.1f)', targetX, targetY, targetZ)
    CpUtil.info('REFILL STRATEGY: Vehicle start world: (%.1f, %.1f, %.1f), target local: (%.1f, %.1f)',
        vehicleX, vehicleY, vehicleZ, localTargetX, localTargetZ)

    -- Build a deterministic final alignment segment parallel to the loader axis.
    self.refillAlignCourse = nil
    if self.refillApproachNode then
        CpUtil.destroyNode(self.refillApproachNode)
        self.refillApproachNode = nil
    end
    if self.dischargeNode and self.dischargeNode.node then
        local dx, _, dz = getWorldTranslation(self.dischargeNode.node)
        self.targetRefillPosition = { x = dx, z = dz }
        CpUtil.info('REFILL STRATEGY: Direct target position from discharge node: (%.1f, %.1f)', dx, dz)
    else
        self.targetRefillPosition = { x = targetX, z = targetZ }
        CpUtil.info('REFILL STRATEGY: Direct target position from target node: (%.1f, %.1f)', targetX, targetZ)
    end
    
    -- Use PathfinderContext for pathfinding (same configuration as UnloadCombine self-unload)
    CpUtil.info('REFILL STRATEGY: Calculating path to refill position...')
    local fieldNum = CpFieldUtil.getFieldNumUnderVehicle(self.vehicle)
    local context = PathfinderContext(self.vehicle)
    
    -- Use low fruit tolerance and default off-field penalty (stay on field!)
    context:maxFruitPercent(10)
    context:offFieldPenalty(PathfinderContext.defaultOffFieldPenalty)
    context:mustBeAccurate(true)
    context:useFieldNum(fieldNum)
    context:allowReverse(true)
    
    -- Ignore off-field penalty around the loader (similar to trailer unload)
    -- This allows pathfinder to reach the loader at field edge without penalty
    context:areaToIgnoreOffFieldPenalty(
        PathfinderUtil.NodeArea.createVehicleArea(
            self.loaderVehicle, 
            1.5 * SelfRefillHelper.maxDistanceFromField))  -- 45m radius
    
    -- Ignore the loader vehicle and its root vehicle (truck) for collision detection
    local vehiclesToIgnore = { self.loaderVehicle }
    local rootVehicle = self.loaderVehicle:getRootVehicle()
    if rootVehicle and rootVehicle ~= self.loaderVehicle then
        table.insert(vehiclesToIgnore, rootVehicle)
    end
    context:vehiclesToIgnore(vehiclesToIgnore)
    
    -- Adaptive iteration limit based on field size
    context:maxIterations(PathfinderUtil.getMaxIterationsForFieldPolygon(self.vehicle:cpGetFieldPolygon()))
    
    CpUtil.info('REFILL STRATEGY: Using field %d, max fruit 10%%, default off-field penalty', fieldNum or 0)
    CpUtil.info('REFILL STRATEGY: Ignoring off-field penalty in 45m radius around loader')
    
    local approachNode = targetNode
    if self.dischargeNode and self.dischargeNode.node and self.loaderVehicle and self.loaderVehicle.rootNode then
        local goalX, goalY, goalZ = getWorldTranslation(self.dischargeNode.node)
        local goalYRot = self:getParallelHeadingForLoader()
        if not goalYRot then
            local _, loaderYRot, _ = getWorldRotation(self.loaderVehicle.rootNode)
            goalYRot = loaderYRot
        end
        self.refillApproachNode = createTransformGroup('cpRefillApproachNode')
        link(getRootNode(), self.refillApproachNode)
        setTranslation(self.refillApproachNode, goalX, goalY, goalZ)
        setRotation(self.refillApproachNode, 0, goalYRot, 0)
        approachNode = self.refillApproachNode
        CpUtil.info('REFILL STRATEGY: Using virtual approach node at discharge position with loader-parallel heading %.1f°',
            math.deg(goalYRot))
    end

    -- Same pattern as unload wagon: path to offset start, then append a straight parallel align segment.
    self.refillAlignCourse = Course.createFromNode(self.vehicle, approachNode,
        offsetX, -alignLength + 1, 0, 1, false)
    if self.refillAlignCourse then
        local lastIx = self.refillAlignCourse:getNumberOfWaypoints()
        local tx, tz = self.refillAlignCourse:getWaypointPosition(lastIx)
        self.targetRefillPosition = { x = tx, z = tz }
        CpUtil.info('REFILL STRATEGY: Target position from refill align course: (%.1f, %.1f)', tx, tz)
    end

    self.pathfindingStartedAt = g_currentMission.time
    CpUtil.info('REFILL STRATEGY: Path request (unload-style) -> offsetX %.2f, approachZ %.2f', offsetX, -alignLength)
    self.pathfinder, result = PathfinderUtil.startPathfindingFromVehicleToNode(
        approachNode, offsetX, -alignLength, context)
    
    if result.done then
        -- Pathfinding completed immediately
        return self:onPathfindingDoneToLoader(result.path)
    else
        -- Pathfinding still running, wait for callback
        CpUtil.info('REFILL STRATEGY: Pathfinding started, waiting for completion...')
        CpUtil.info('REFILL STRATEGY: >>> Vehicle will STOP during calculation <<<')
        self.state = self.states.DRIVING_TO_LOADER_PATHFINDING
        self:setPathfindingDoneCallback(self, self.onPathfindingDoneToLoader)
        -- Explizit Geschwindigkeit auf 0 setzen während Pfadberechnung
        self:setMaxSpeed(0)
    end
    CpUtil.info('=========================================')
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
    
    -- Warten bis Implements zusammengeklappt sind
    if self.state == self.states.WAITING_FOR_FOLD then
        self:setMaxSpeed(0)
        if g_currentMission.time >= (self.nextFoldCommandAt or 0) then
            self:requestFoldAllForRefill()
        end
        local foldReady = self:areFoldablesReadyForPathfinding()
        local timedOut = (g_currentMission.time - (self.foldStartedAt or 0)) > self.foldTimeoutMs

        if foldReady then
            -- Implements sind jetzt zusammengeklappt, jetzt Pathfinding starten
            self:startPathfindingAfterFold()
        elseif timedOut then
            if g_currentMission.time >= (self.nextFoldTimeoutLogAt or 0) then
                CpUtil.info('REFILL STRATEGY: Fold wait timeout reached, still waiting for complete fold animation')
                self.nextFoldTimeoutLogAt = g_currentMission.time + 5000
            end
        else
            self:debugSparse('Waiting for implements to fold completely before pathfinding...')
        end
        return  -- Nicht weiter updaten während Falten
    end
    
    -- WICHTIG: Während Pfadberechnung Fahrzeug anhalten
    if self.state == self.states.DRIVING_TO_LOADER_PATHFINDING then
        -- Fahrzeug muss während Pathfinding komplett stoppen
        self:setMaxSpeed(0)
        self:updatePathfinding()
        -- Nach updatePathfinding nochmal sicherstellen dass Speed 0 ist
        self:setMaxSpeed(0)
        return  -- Wichtig: Nicht weiter updaten während Pathfinding
    end
    
    -- Handle waiting for loader to appear
    if self.state == self.states.WAITING_FOR_LOADER then
        self:updateWaitingForLoader(dt)
    end

    -- Keep PPC off-track auto-stop disabled while navigating the generated refill approach path.
    -- This must happen before AIDriveStrategyCourse.update(), as PPC logic runs there.
    if self.state == self.states.DRIVING_TO_LOADER or
       self.state == self.states.DRIVING_TO_LOADER_PATHFINDING then
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
            CpUtil.info('REFILL STRATEGY: Vehicle is %.1fm away, folding implements first...', distanceToLoader)
            CpUtil.info('REFILL STRATEGY: >>> Folding implements for better pathfinding <<<')
            
            -- WICHTIG: Implements zusammenklappen BEVOR Pfad berechnet wird
            self:raiseImplements()
            self:requestFoldAllForRefill()
            self.state = self.states.WAITING_FOR_FOLD
            self.foldStartedAt = g_currentMission.time
            self.targetLoaderData = {
                fieldPolygon = fieldPolygon,
                fillTypeIndex = fillTypeIndex,
                distanceToLoader = distanceToLoader
            }
            CpUtil.info('REFILL STRATEGY: Waiting for implements to fold...')
            CpUtil.info('=========================================')
        else
            -- Still no loader found
            CpUtil.info('REFILL STRATEGY: Still no loader found, waiting...')
            CpUtil.info('=========================================')
        end
    end
end

--- Callback for when pathfinding to loader completes
function AIDriveStrategyRefillAtLoader:onPathfindingDoneToLoader(path)
    if path and #path > 2 then
        CpUtil.info('REFILL STRATEGY: ✓ Path calculated - %d waypoints (%d ms)',
            #path, g_currentMission.time - (self.pathfindingStartedAt or 0))
        local first = path[1]
        local last = path[#path]
        if first and last and first.x and first.z and last.x and last.z then
            CpUtil.info('REFILL STRATEGY: Path endpoints -> start (%.1f, %.1f), end (%.1f, %.1f)',
                first.x, first.z, last.x, last.z)
        end
        CpUtil.info('REFILL STRATEGY: >>> Driving to loader NOW <<<')
        local course = Course(self.vehicle, CpMathUtil.pointsToGameInPlace(path), true)
        if self.refillAlignCourse then
            course:append(self.refillAlignCourse)
        end
        self:startCourse(course, 1)
        self.state = self.states.DRIVING_TO_LOADER
        self.ppc:disableStopWhenOffTrack(15000)
        CpUtil.info('=========================================')
    else
        CpUtil.info('REFILL STRATEGY: WARNING - Could not find path to loader')
        CpUtil.info('REFILL STRATEGY: Path length: %d', path and #path or 0)
        CpUtil.info('REFILL STRATEGY: Waiting at current position for manual help')
        self.state = self.states.WAITING_FOR_REFILL
        CpUtil.info('=========================================')
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
        
    elseif self.state == self.states.WAITING_FOR_FOLD then
        -- Stop and wait while implements are folding
        -- WICHTIG: Fahrzeug komplett anhalten während Implements zusammenklappen
        self:setMaxSpeed(0)
        gx, gz = vX, vZ  -- Aktuelle Position als Ziel (nicht bewegen!)
        moveForwards = true
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        -- Explizit zurückgeben um weitere Verarbeitung zu vermeiden
        self:checkProximitySensors(moveForwards)
        return gx, gz, moveForwards, 0, 100  -- Geschwindigkeit = 0!
        
    elseif self.state == self.states.DRIVING_TO_LOADER_PATHFINDING then
        -- Stop and wait while pathfinding is calculating
        -- WICHTIG: Fahrzeug komplett anhalten während Pfadberechnung
        self:setMaxSpeed(0)
        gx, gz = vX, vZ  -- Aktuelle Position als Ziel (nicht bewegen!)
        moveForwards = true
        if not self.implementsRaised then
            self:raiseImplements()
            self.implementsRaised = true
        end
        -- Explizit zurückgeben um weitere Verarbeitung zu vermeiden
        self:checkProximitySensors(moveForwards)
        return gx, gz, moveForwards, 0, 100  -- Geschwindigkeit = 0!
        
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
        self.refillApproachRetryCount < 2 and self.targetLoaderData then
        local distanceToDischarge = self:getDistanceToDischargeNode() or math.huge
        if distanceToDischarge > (AIDriveStrategyRefillAtLoader.minDistanceToDischargeNode + 1.0) then
            self.refillApproachRetryCount = self.refillApproachRetryCount + 1
            CpUtil.info('REFILL STRATEGY: No refill detected and still %.1fm from discharge node, recalculating approach (retry %d/2)',
                distanceToDischarge, self.refillApproachRetryCount)
            self.state = self.states.DRIVING_TO_LOADER_PATHFINDING
            self:startPathfindingAfterFold()
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

    -- Never complete immediately if we could not identify any refill-relevant unit.
    return hasAnyRelevantUnit and allRelevantUnitsFilled
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
