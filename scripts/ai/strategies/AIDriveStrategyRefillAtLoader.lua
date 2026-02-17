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
    
    -- Erstelle manuellen Pfad (Bogen-Gerade-Bogen-Gerade) statt Pathfinder
    local waypoints = self:createManualRefillPath()
    
    if not waypoints or #waypoints < 2 then
        CpUtil.info('REFILL STRATEGY: ERROR - Could not create manual refill path')
        CpUtil.info('REFILL STRATEGY: Falling back to waiting for manual positioning')
        self.state = self.states.WAITING_FOR_REFILL
        self.refillTimer = 0
        return
    end
    
    -- Speichere Zielposition vom letzten Waypoint
    local lastWp = waypoints[#waypoints]
    self.targetRefillPosition = { x = lastWp.x, z = lastWp.z }
    CpUtil.info('REFILL STRATEGY: Target position: (%.1f, %.1f)', lastWp.x, lastWp.z)
    
    -- Erstelle Course aus den Waypoints
    CpUtil.info('REFILL STRATEGY: Creating course from %d waypoints...', #waypoints)
    local course = Course(self.vehicle, waypoints, true)
    course:enrichWaypointData()
    
    CpUtil.info('REFILL STRATEGY: >>> Starting drive to loader NOW <<<')
    self:startCourse(course, 1)
    self.state = self.states.DRIVING_TO_LOADER
    self.ppc:disableStopWhenOffTrack(15000)
    CpUtil.info('=========================================')
end

--- Erstellt einen manuellen Pfad zum Auflader aus Bögen und Geraden
--- Pfad besteht aus: Bogen -> Gerade -> Bogen -> Gerade (parallel zum Auflader)
function AIDriveStrategyRefillAtLoader:createManualRefillPath()
    CpUtil.info('=========================================')
    CpUtil.info('REFILL STRATEGY: Erstelle manuellen Refill-Pfad (Bogen-Gerade-Bogen-Gerade)')
    
    -- Hole Fahrzeugposition und Richtung
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle:getAIDirectionNode())
    local _, vehicleYRot, _ = getWorldRotation(self.vehicle:getAIDirectionNode())
    CpUtil.info('REFILL STRATEGY: Fahrzeugposition: (%.1f, %.1f, %.1f), Richtung: %.1f°', 
        vehicleX, vehicleY, vehicleZ, math.deg(vehicleYRot))
    
    -- Hole Aufladerposition und Richtung
    if not self.loaderVehicle or not self.dischargeNode then
        CpUtil.info('REFILL STRATEGY: FEHLER - Kein Auflader oder Discharge Node vorhanden')
        return nil
    end
    
    local loaderX, loaderY, loaderZ = getWorldTranslation(self.loaderVehicle.rootNode)
    local _, loaderYRot, _ = getWorldRotation(self.loaderVehicle.rootNode)
    local dischargeX, dischargeY, dischargeZ = getWorldTranslation(self.dischargeNode.node)
    
    CpUtil.info('REFILL STRATEGY: Auflader Position: (%.1f, %.1f, %.1f), Richtung: %.1f°', 
        loaderX, loaderY, loaderZ, math.deg(loaderYRot))
    CpUtil.info('REFILL STRATEGY: Discharge Node: (%.1f, %.1f, %.1f)', 
        dischargeX, dischargeY, dischargeZ)
    
    -- Berechne Turning Radius
    local turningRadius = AIUtil.getTurningRadius(self.vehicle)
    if not turningRadius or turningRadius < 5 then
        turningRadius = 8  -- Mindest-Radius
    end
    CpUtil.info('REFILL STRATEGY: Turning Radius: %.1f m', turningRadius)
    
    -- Erstelle Waypoints-Array
    local waypoints = {}
    local waypointStep = 1.5  -- Abstand zwischen Waypoints
    
    -- ============================================================================
    -- Berechne Zielgeometrie (Endposition 20m hinter dem Discharge Node)
    -- ============================================================================
    
    -- Richtung vom Auflader zum Discharge Node
    local loaderToDischargeX = dischargeX - loaderX
    local loaderToDischargeZ = dischargeZ - loaderZ
    local loaderToDischargeLength = math.sqrt(loaderToDischargeX * loaderToDischargeX + loaderToDischargeZ * loaderToDischargeZ)
    
    if loaderToDischargeLength < 0.1 then
        CpUtil.info('REFILL STRATEGY: FEHLER - Auflader und Discharge Node zu nah beieinander')
        return nil
    end
    
    local loaderDirX = loaderToDischargeX / loaderToDischargeLength
    local loaderDirZ = loaderToDischargeZ / loaderToDischargeLength
    
    -- Finale Fahrtrichtung (parallel zum Auflader, in Richtung Discharge Node)
    local finalYRot = math.atan2(loaderDirX, loaderDirZ)
    
    -- Seitlicher Versatz (rechtwinklig zur Laderichtung)
    local sideOffset = 6  -- 6m seitlich versetzt
    local behindDistance = 20  -- 20m hinter dem Discharge Node
    
    -- Berechne seitliche Richtung (90° gedreht)
    local sideDirX = -loaderDirZ
    local sideDirZ = loaderDirX
    
    -- Startpunkt der finalen parallelen Gerade (20m hinter Discharge Node, seitlich versetzt)
    local parallelStartX = dischargeX - loaderDirX * behindDistance + sideDirX * sideOffset
    local parallelStartZ = dischargeZ - loaderDirZ * behindDistance + sideDirZ * sideOffset
    
    CpUtil.info('REFILL STRATEGY: Parallele Anfahrt Start: (%.1f, %.1f)', parallelStartX, parallelStartZ)
    CpUtil.info('REFILL STRATEGY: Finale Richtung: %.1f°', math.deg(finalYRot))
    
    -- ============================================================================
    -- Rückwärtsplanung: Starte vom Ziel und arbeite zum Fahrzeug zurück
    -- ============================================================================
    
    -- Berechne Fahrtrichtung vom Fahrzeug grob in Richtung Ziel
    local vehicleToTargetX = parallelStartX - vehicleX
    local vehicleToTargetZ = parallelStartZ - vehicleZ
    local distanceToTarget = math.sqrt(vehicleToTargetX * vehicleToTargetX + vehicleToTargetZ * vehicleToTargetZ)
    
    if distanceToTarget < 2 * turningRadius then
        CpUtil.info('REFILL STRATEGY: WARNUNG - Zu nah am Ziel, verwende einfachen direkten Pfad')
        -- Erstelle einfachen direkten Pfad
        local steps = math.ceil(distanceToTarget / waypointStep)
        for i = 0, steps do
            local t = i / steps
            local wpX = vehicleX + vehicleToTargetX * t
            local wpZ = vehicleZ + vehicleToTargetZ * t
            table.insert(waypoints, { x = wpX, z = wpZ })
        end
        table.insert(waypoints, { x = dischargeX, z = dischargeZ })
        return waypoints
    end
    
    -- Bestimme Abbiegewinkel (positiv = links, negativ = rechts)
    local angleDiff = CpMathUtil.getDeltaAngle(finalYRot, vehicleYRot)
    local turnDirection = angleDiff > 0 and 1 or -1  -- 1 = links, -1 = rechts
    local totalTurnAngle = math.abs(angleDiff)
    
    -- Begrenze auf maximal 180°
    if totalTurnAngle > math.pi then
        totalTurnAngle = 2 * math.pi - totalTurnAngle
        turnDirection = -turnDirection
    end
    
    -- Teile Total Turn Angle auf zwei Bögen auf (z.B. je 50%)
    local arc1Angle = totalTurnAngle * 0.5
    local arc2Angle = totalTurnAngle * 0.5
    
    CpUtil.info('REFILL STRATEGY: Gesamtwinkel: %.1f°, Richtung: %s', 
        math.deg(totalTurnAngle), turnDirection > 0 and 'links' or 'rechts')
    CpUtil.info('REFILL STRATEGY: Bogen 1: %.1f°, Bogen 2: %.1f°', 
        math.deg(arc1Angle), math.deg(arc2Angle))
    
    -- ============================================================================
    -- SEGMENT 1: Erster Bogen vom Fahrzeug
    -- ============================================================================
    local arc1Steps = math.ceil((arc1Angle * turningRadius) / waypointStep)
    local arc1AngleStep = arc1Angle / arc1Steps
    
    -- Bogenmittelpunkt
    local arc1CenterX = vehicleX - math.sin(vehicleYRot) * turningRadius * turnDirection
    local arc1CenterZ = vehicleZ - math.cos(vehicleYRot) * turningRadius * turnDirection
    
    -- Erstelle Waypoints für ersten Bogen
    for i = 0, arc1Steps do
        local angle = vehicleYRot + (i * arc1AngleStep * turnDirection)
        local wpX = arc1CenterX + math.sin(angle) * turningRadius * turnDirection
        local wpZ = arc1CenterZ + math.cos(angle) * turningRadius * turnDirection
        table.insert(waypoints, { x = wpX, z = wpZ })
    end
    
    local arc1EndX = waypoints[#waypoints].x
    local arc1EndZ = waypoints[#waypoints].z
    local arc1EndAngle = vehicleYRot + (arc1Angle * turnDirection)
    
    CpUtil.info('REFILL STRATEGY: Bogen 1 Ende: (%.1f, %.1f), Winkel: %.1f°', 
        arc1EndX, arc1EndZ, math.deg(arc1EndAngle))
    
    -- ============================================================================
    -- SEGMENT 2: Gerade zwischen den Bögen
    -- ============================================================================
    -- Berechne benötigte Länge der Geraden basierend auf verbleibender Distanz
    local remainingX = parallelStartX - arc1EndX
    local remainingZ = parallelStartZ - arc1EndZ
    local remainingDist = math.sqrt(remainingX * remainingX + remainingZ * remainingZ)
    
    -- Abzug für zweiten Bogen (ungefähr)
    local arc2Length = arc2Angle * turningRadius
    local straightDistance = math.max(10, remainingDist - arc2Length - 2 * turningRadius)
    
    local straight1DirX = math.sin(arc1EndAngle)
    local straight1DirZ = math.cos(arc1EndAngle)
    
    local straight1Steps = math.ceil(straightDistance / waypointStep)
    for i = 1, straight1Steps do
        local wpX = arc1EndX + straight1DirX * i * waypointStep
        local wpZ = arc1EndZ + straight1DirZ * i * waypointStep
        table.insert(waypoints, { x = wpX, z = wpZ })
    end
    
    local straight1EndX = waypoints[#waypoints].x
    local straight1EndZ = waypoints[#waypoints].z
    
    CpUtil.info('REFILL STRATEGY: Gerade Ende: (%.1f, %.1f), Länge: %.1f m', 
        straight1EndX, straight1EndZ, straightDistance)
    
    -- ============================================================================
    -- SEGMENT 3: Zweiter Bogen zur parallelen Endrichtung
    -- ============================================================================
    local arc2Steps = math.ceil((arc2Angle * turningRadius) / waypointStep)
    local arc2AngleStep = arc2Angle / arc2Steps
    
    -- Bogenmittelpunkt
    local arc2CenterX = straight1EndX - math.sin(arc1EndAngle) * turningRadius * turnDirection
    local arc2CenterZ = straight1EndZ - math.cos(arc1EndAngle) * turningRadius * turnDirection
    
    -- Erstelle Waypoints für zweiten Bogen (zurück zur Zielrichtung)
    for i = 1, arc2Steps do
        local angle = arc1EndAngle + (i * arc2AngleStep * -turnDirection)
        local wpX = arc2CenterX + math.sin(angle) * turningRadius * turnDirection
        local wpZ = arc2CenterZ + math.cos(angle) * turningRadius * turnDirection
        table.insert(waypoints, { x = wpX, z = wpZ })
    end
    
    local arc2EndX = waypoints[#waypoints].x
    local arc2EndZ = waypoints[#waypoints].z
    
    CpUtil.info('REFILL STRATEGY: Bogen 2 Ende: (%.1f, %.1f)', arc2EndX, arc2EndZ)
    
    -- ============================================================================
    -- SEGMENT 4: Finale Gerade parallel zum Auflader
    -- ============================================================================
    local finalDistX = dischargeX - arc2EndX
    local finalDistZ = dischargeZ - arc2EndZ
    local finalDistance = math.sqrt(finalDistX * finalDistX + finalDistZ * finalDistZ)
    
    local finalStraightSteps = math.ceil(finalDistance / waypointStep)
    
    CpUtil.info('REFILL STRATEGY: Finale Gerade - Distanz: %.1f m', finalDistance)
    
    for i = 1, finalStraightSteps do
        local wpX = arc2EndX + loaderDirX * i * waypointStep
        local wpZ = arc2EndZ + loaderDirZ * i * waypointStep
        table.insert(waypoints, { x = wpX, z = wpZ })
    end
    
    -- Füge finalen Waypoint bei Discharge Node hinzu
    table.insert(waypoints, { x = dischargeX, z = dischargeZ })
    
    CpUtil.info('REFILL STRATEGY: ✓ Manueller Pfad erstellt mit %d Waypoints', #waypoints)
    CpUtil.info('=========================================')
    
    return waypoints
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
    
    -- Handle waiting for loader to appear
    if self.state == self.states.WAITING_FOR_LOADER then
        self:updateWaitingForLoader(dt)
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
