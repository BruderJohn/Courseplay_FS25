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
]]

--- Helper functions for finding a loader/filler vehicle around the field where a sprayer or manure spreader can refill
---@class SelfRefillHelper
SelfRefillHelper = {}
SelfRefillHelper.debugChannel = CpDebug.DBG_FIELDWORK
-- search for loaders/fillers within this distance from the field
SelfRefillHelper.maxDistanceFromField = 30

--- Find a loader/filler vehicle we can use for refilling
---@param fieldPolygon Polygon the field boundary. We'll look for loaders on this field, or close to the boundary.
---@param myVehicle table vehicle to refill
---@param fillTypeIndex number required fill type
---@return table|nil loader/filler vehicle (nil if no loader found)
---@return table|nil discharge node of the loader
---@return number|nil distance of loader from myVehicle
function SelfRefillHelper:findBestLoader(fieldPolygon, myVehicle, fillTypeIndex)
    if fieldPolygon == nil or #fieldPolygon == 0 then
        CpUtil.errorVehicle(myVehicle, 'Field polygon is nil or empty, can\'t find a loader to refill from')
        return nil
    end
    
    local bestLoader, bestDischargeNode
    local minDistance = math.huge
    
    for _, otherVehicle in pairs(g_currentMission.vehicleSystem.vehicles) do
        -- Look for vehicles with discharge capability (slurry tankers, manure spreaders, liquid tankers, etc.)
        if SpecializationUtil.hasSpecialization(Dischargeable, otherVehicle.specializations) then
            local rootVehicle = otherVehicle:getRootVehicle()
            local x, _, z = getWorldTranslation(otherVehicle.rootNode)
            
            -- Check if the loader is within range of the field perimeter
            local isOnField, closestDistance = CpMathUtil.isWithinDistanceToPolygon(fieldPolygon, x, z, SelfRefillHelper.maxDistanceFromField)
            if not isOnField then
                -- not within range, but could still be on the field
                isOnField = CpMathUtil.isPointInPolygon(fieldPolygon, x, z)
            end
            
            local lastSpeed = rootVehicle:getLastSpeed()
            local isCpActive = rootVehicle.getIsCpActive and rootVehicle:getIsCpActive()
            
            CpUtil.debugVehicle(self.debugChannel, myVehicle,
                    '%s is a potential loader %s on my field, closest distance to the field is %.1f, root vehicle is %s, last speed %.1f, CP active %s',
                    otherVehicle:getName(), isOnField and '' or 'NOT', closestDistance,
                    rootVehicle:getName(), lastSpeed, isCpActive)
            
            -- Consider only vehicles on or near the field, not driven by CP, stopped, and not ourselves
            if rootVehicle ~= myVehicle and not isCpActive and lastSpeed < 0.1 and isOnField then
                local d = calcDistanceFrom(myVehicle:getAIDirectionNode(), otherVehicle.rootNode or otherVehicle.nodeId)
                
                -- Check if this vehicle can provide the required fill type
                local canRefill, dischargeNode = self:canRefillFrom(otherVehicle, myVehicle, fillTypeIndex)

                if canRefill and dischargeNode and dischargeNode.node then
                    local nx, ny, nz = getWorldTranslation(dischargeNode.node)
                    CpUtil.debugVehicle(self.debugChannel, myVehicle,
                        'Candidate loader %s distance %.1f m, discharge node %d world pos (%.1f, %.1f, %.1f)',
                        otherVehicle:getName(), d, dischargeNode.index or -1, nx, ny, nz)
                end
                
                if d < minDistance and canRefill then
                    bestLoader = otherVehicle
                    bestDischargeNode = dischargeNode
                    minDistance = d
                end
            end
        end
    end
    
    if bestLoader then
        CpUtil.debugVehicle(self.debugChannel, myVehicle,
                'Best loader is %s at %.1f meters',
                bestLoader:getName(), minDistance)
        if bestDischargeNode and bestDischargeNode.node then
            local dx, dy, dz = getWorldTranslation(bestDischargeNode.node)
            local lx, ly, lz = getWorldTranslation(bestLoader.rootNode)
            CpUtil.debugVehicle(self.debugChannel, myVehicle,
                'Selected discharge node %d world pos (%.1f, %.1f, %.1f), loader root pos (%.1f, %.1f, %.1f)',
                bestDischargeNode.index or -1, dx, dy, dz, lx, ly, lz)
        end
        return bestLoader, bestDischargeNode, minDistance
    else
        CpUtil.infoVehicle(myVehicle, 'Found no loader to refill from.')
        return nil
    end
end

--- Check if we can refill from a loader vehicle
---@param loaderVehicle table potential loader vehicle
---@param myVehicle table vehicle that needs refilling
---@param fillTypeIndex number required fill type
---@return boolean can refill from this vehicle
---@return table|nil discharge node if can refill
function SelfRefillHelper:canRefillFrom(loaderVehicle, myVehicle, fillTypeIndex)
    local dischargeableSpec = loaderVehicle.spec_dischargeable
    if not dischargeableSpec then
        return false, nil
    end
    
    -- Check all discharge nodes
    for _, dischargeNode in ipairs(dischargeableSpec.dischargeNodes) do
        -- Check if the discharge node supports our fill type
        local fillLevel = loaderVehicle:getFillUnitFillLevel(dischargeNode.fillUnitIndex)
        local fillType = loaderVehicle:getFillUnitFillType(dischargeNode.fillUnitIndex)
        
        CpUtil.debugVehicle(self.debugChannel, myVehicle,
                'Checking discharge node: fillLevel %.1f, fillType %s (required: %s)',
                fillLevel, g_fillTypeManager:getFillTypeNameByIndex(fillType), 
                g_fillTypeManager:getFillTypeNameByIndex(fillTypeIndex))
        
        -- Check if fill types match and loader has enough fill level
        if fillLevel > 100 and (fillType == fillTypeIndex or fillType == FillType.UNKNOWN) then
            -- Check if the loader is accessible (farm ownership)
            if loaderVehicle.getIsFillAllowedFromFarm == nil or 
               loaderVehicle:getIsFillAllowedFromFarm(myVehicle:getActiveFarm()) then
                CpUtil.debugVehicle(self.debugChannel, myVehicle,
                        'Can refill from %s, discharge node %d',
                        loaderVehicle:getName(), dischargeNode.index)
                return true, dischargeNode
            end
        end
    end
    
    return false, nil
end

--- Get target parameters for approaching the loader
---@param fieldPolygon Polygon the field boundary
---@param myVehicle table vehicle that needs refilling
---@param fillTypeIndex number required fill type
---@param bestLoader table|nil optional loader to use (will find one if nil)
---@param dischargeNode table|nil discharge node of the loader
---@return number|nil target node
---@return number|nil align length
---@return number|nil offset X
---@return table|nil loader vehicle
function SelfRefillHelper:getLoaderTargetParameters(fieldPolygon, myVehicle, fillTypeIndex, bestLoader, dischargeNode)
    if not bestLoader then
        bestLoader, dischargeNode = self:findBestLoader(fieldPolygon, myVehicle, fillTypeIndex)
        if not bestLoader then
            return nil
        end
    end
    
    local targetNode = dischargeNode.node or bestLoader.rootNode
    local loaderRootNode = bestLoader.rootNode
    local loaderLength = bestLoader.size.length
    local loaderWidth = bestLoader.size.width
    
    -- Calculate approach parameters
    local _, steeringLength = AIUtil.getSteeringParameters(myVehicle)
    local _, frontMarkerOffset = Markers.getFrontMarkerNode(myVehicle)
    
    -- Position to approach: behind/beside the loader depending on discharge node position
    local _, _, dZ = localToLocal(loaderRootNode, targetNode, 0, 0, 0)
    local alignLength = math.max((loaderLength / 2) + math.abs(dZ) + myVehicle.size.length / 2 + frontMarkerOffset, steeringLength)
    
    -- Offset to the side to receive the discharge
    local offsetX = (loaderWidth / 2) + (myVehicle.size.width / 2) + 1.5
    
    -- Determine which side to approach from based on discharge node position
    local nodeX, _, _ = localToLocal(targetNode, loaderRootNode, 0, 0, 0)
    if nodeX < 0 then
        offsetX = -offsetX  -- Approach from left side
    end
    
    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle,
            'Loader params: length %.1f, width %.1f, alignLength %.1f, offsetX %.1f, nodeX %.1f, dZ %.1f, steeringLen %.1f, frontMarker %.1f',
            loaderLength, loaderWidth, alignLength, offsetX, nodeX, dZ, steeringLength, frontMarkerOffset)

        local tx, ty, tz = getWorldTranslation(targetNode)
        local lx, ly, lz = getWorldTranslation(loaderRootNode)
        local localTargetX, _, localTargetZ = localToLocal(targetNode, myVehicle:getAIDirectionNode(), 0, 0, 0)
        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle,
            'Target node world (%.1f, %.1f, %.1f), loader root world (%.1f, %.1f, %.1f), target local to vehicle (%.1f, %.1f)',
            tx, ty, tz, lx, ly, lz, localTargetX, localTargetZ)
    
    return targetNode, alignLength, offsetX, bestLoader
end

--- Calculate precise refill position where discharge node and fill node overlap
---@param fieldPolygon table Field boundary polygon
---@param myVehicle table The vehicle with implement to refill
---@param fillTypeIndex number Required fill type
---@param bestLoader table The loader vehicle
---@param dischargeNode table The discharge node of the loader
---@return number|nil x World X coordinate of target position
---@return number|nil z World Z coordinate of target position  
---@return number|nil yRot Target rotation (facing discharge node)
function SelfRefillHelper:calculatePreciseRefillPosition(fieldPolygon, myVehicle, fillTypeIndex, bestLoader, dischargeNode)
    if not bestLoader or not dischargeNode or not dischargeNode.node then
        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 'Cannot calculate refill position: invalid loader or discharge node')
        return nil
    end
    
    -- Get discharge node world position
    local dischargeX, dischargeY, dischargeZ = getWorldTranslation(dischargeNode.node)
    
    -- Find the fill node/trigger of the implement
    local implements = myVehicle:getAttachedImplements()
    local fillNode = nil
    local fillNodeOffsetX, fillNodeOffsetZ = 0, 0
    
    for _, implement in pairs(implements) do
        local implementVehicle = implement.object
        
        -- Check for pipe specialization (common for liquid sprayers/spreaders)
        if implementVehicle.spec_pipe then
            local pipeSpec = implementVehicle.spec_pipe
            if pipeSpec.nodes and #pipeSpec.nodes > 0 then
                -- Use the first pipe node as fill point
                fillNode = pipeSpec.nodes[1].node
                CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 'Found pipe node for refilling')
                break
            end
        end
        
        -- Check for fillVolume (alternative fill point)
        if implementVehicle.spec_fillVolume then
            local fillVolumeSpec = implementVehicle.spec_fillVolume
            if fillVolumeSpec.volumes and #fillVolumeSpec.volumes > 0 then
                for _, volume in ipairs(fillVolumeSpec.volumes) do
                    if volume.fillTriggerNode then
                        fillNode = volume.fillTriggerNode
                        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 'Found fillVolume trigger for refilling')
                        break
                    end
                end
                if fillNode then break end
            end
        end
        
        -- Fallback: use implement root node with offset
        if not fillNode and implementVehicle.rootNode then
            fillNode = implementVehicle.rootNode
            -- Estimate fill point at rear of implement
            fillNodeOffsetZ = -implementVehicle.size.length / 2
            CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 'Using implement root node as fallback with offset')
        end
    end
    
    if not fillNode then
        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 'No fill node found on implement')
        return nil
    end
    
    -- Calculate where vehicle needs to be positioned so fill node aligns with discharge node
    -- We need to find vehicle position where fillNode (after offset) = dischargeNode position
    
    -- Get current fill node position relative to vehicle
    local fillNodeLocalX, fillNodeLocalY, fillNodeLocalZ = localToLocal(fillNode, myVehicle.rootNode, fillNodeOffsetX, 0, fillNodeOffsetZ)
    
    -- Calculate target vehicle position
    -- vehiclePos + fillNodeOffset = dischargePos
    -- vehiclePos = dischargePos - fillNodeOffset
    
    -- We need to rotate the offset by vehicle's rotation
    -- For now, assume vehicle should face the loader
    local loaderX, _, loaderZ = getWorldTranslation(bestLoader.rootNode)
    local targetYRot = math.atan2(dischargeX - loaderX, dischargeZ - loaderZ)
    
    -- Apply rotation to fill node offset
    local cosRot = math.cos(targetYRot)
    local sinRot = math.sin(targetYRot)
    local rotatedOffsetX = fillNodeLocalX * cosRot - fillNodeLocalZ * sinRot
    local rotatedOffsetZ = fillNodeLocalX * sinRot + fillNodeLocalZ * cosRot
    
    -- Calculate target vehicle position
    local targetX = dischargeX - rotatedOffsetX
    local targetZ = dischargeZ - rotatedOffsetZ
    
    -- Check if target position is on the field
    local isOnField, distanceToField = CpMathUtil.isWithinDistanceToPolygon(fieldPolygon, targetX, targetZ, SelfRefillHelper.maxDistanceFromField)
    
    if not isOnField then
        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
            'Calculated refill position (%.1f, %.1f) is too far from field (%.1fm)', 
            targetX, targetZ, distanceToField)
        -- Try to find a position closer to field edge
        -- Move position towards field center
        local fieldCenterX, fieldCenterZ = 0, 0
        for i = 1, #fieldPolygon do
            fieldCenterX = fieldCenterX + fieldPolygon[i].x
            fieldCenterZ = fieldCenterZ + fieldPolygon[i].z
        end
        fieldCenterX = fieldCenterX / #fieldPolygon
        fieldCenterZ = fieldCenterZ / #fieldPolygon
        
        -- Move target 5m towards field center
        local dirX = fieldCenterX - targetX
        local dirZ = fieldCenterZ - targetZ
        local dirLength = math.sqrt(dirX * dirX + dirZ * dirZ)
        if dirLength > 0.1 then
            targetX = targetX + (dirX / dirLength) * 5
            targetZ = targetZ + (dirZ / dirLength) * 5
        end
    end
    
    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
        'Calculated precise refill position: (%.1f, %.1f) rotation: %.1f°, on field: %s', 
        targetX, targetZ, math.deg(targetYRot), tostring(isOnField))
    
    -- Validate that the position is collision-free
    local isPositionValid = self:validateRefillPosition(myVehicle, targetX, targetZ, targetYRot, bestLoader)
    
    if not isPositionValid then
        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
            'Position (%.1f, %.1f) has collision, trying alternative positions...', 
            targetX, targetZ)
        
        -- Try alternative positions with different distances from loader
        local dischargeX, dischargeY, dischargeZ = getWorldTranslation(dischargeNode.node)
        local loaderX, _, loaderZ = getWorldTranslation(bestLoader.rootNode)
        
        -- Calculate direction from loader to discharge node
        local dirX = dischargeX - loaderX
        local dirZ = dischargeZ - loaderZ
        local dirLength = math.sqrt(dirX * dirX + dirZ * dirZ)
        if dirLength > 0.1 then
            dirX = dirX / dirLength
            dirZ = dirZ / dirLength
        end
        
        -- Try positions at different distances (1m, 2m, 3m, 4m further away)
        for extraDistance = 1, 10 do
            local testX = targetX + dirX * extraDistance
            local testZ = targetZ + dirZ * extraDistance
            
            -- Check if still within field range
            local testIsOnField = CpMathUtil.isWithinDistanceToPolygon(fieldPolygon, testX, testZ, SelfRefillHelper.maxDistanceFromField)
            
            if testIsOnField then
                local testIsValid = self:validateRefillPosition(myVehicle, testX, testZ, targetYRot, bestLoader)
                if testIsValid then
                    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
                        'Found valid alternative position at +%dm: (%.1f, %.1f)', 
                        extraDistance, testX, testZ)
                    return testX, testZ, targetYRot
                end
            end
        end
        
        -- No valid position found
        CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
            'Could not find collision-free position near discharge node')
        return nil
    end
    
    return targetX, targetZ, targetYRot
end

--- Validate that a refill position is collision-free
---@param myVehicle table The vehicle with implement
---@param targetX number World X coordinate
---@param targetZ number World Z coordinate  
---@param targetYRot number Target rotation
---@param loaderVehicle table The loader vehicle to ignore
---@return boolean true if position is valid (no collision)
function SelfRefillHelper:validateRefillPosition(myVehicle, targetX, targetZ, targetYRot, loaderVehicle)
    -- Create a temporary node at the target position
    local testNode = createTransformGroup('refillTestNode')
    link(getRootNode(), testNode)
    local terrainHeight = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, targetX, 0, targetZ)
    setTranslation(testNode, targetX, terrainHeight, targetZ)
    setRotation(testNode, 0, targetYRot, 0)
    
    -- Create collision detector (ignore the loader vehicle AND its root vehicle/truck)
    local vehiclesToIgnore = {}
    if loaderVehicle then
        table.insert(vehiclesToIgnore, loaderVehicle)
        -- Also ignore the root vehicle (e.g., truck pulling the loader trailer)
        local rootVehicle = loaderVehicle:getRootVehicle()
        if rootVehicle and rootVehicle ~= loaderVehicle then
            table.insert(vehiclesToIgnore, rootVehicle)
            CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
                'Also ignoring root vehicle: %s', CpUtil.getName(rootVehicle))
        end
    end
    local collisionDetector = PathfinderCollisionDetector(myVehicle, vehiclesToIgnore, {}, false)
    
    -- Use VehicleSizeScanner to get the ACTUAL current size of vehicle + implements
    local sizeScanner = VehicleSizeScanner()
    local front, rear, left, right = sizeScanner:scan(myVehicle, myVehicle.rootNode)
    
    local vehicleLength = front - rear
    local vehicleWidth = left - right
    
    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
        'Measured vehicle size: %.1fm x %.1fm (front:%.1f rear:%.1f left:%.1f right:%.1f)', 
        vehicleWidth, vehicleLength, front, rear, left, right)
    
    -- Add small buffer for safety (0.5m on each side)
    local safetyBuffer = 0.5
    
    -- Overlap box parameters using scanned size
    local overlapBoxParams = {
        width = (vehicleWidth / 2) + safetyBuffer,
        length = (vehicleLength / 2) + safetyBuffer,
        xOffset = 0,
        zOffset = 0
    }
    
    -- Check for collisions
    local collidingShapes = collisionDetector:findCollidingShapes(testNode, myVehicle, overlapBoxParams)
    
    -- Clean up
    CpUtil.destroyNode(testNode)
    
    -- Position is valid if no collisions found
    local isValid = collidingShapes == 0
    
    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle, 
        'Position validation (%.1f, %.1f): %s (%d collisions) using size %.1fm x %.1fm', 
        targetX, targetZ, isValid and 'VALID' or 'INVALID', collidingShapes, vehicleWidth, vehicleLength)
    
    return isValid
end

