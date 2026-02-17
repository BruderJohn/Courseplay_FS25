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

--- Helper functions for finding a loader/filler vehicle (slurry tanker) around the field where a sprayer or manure spreader can refill
--- Similar approach to SelfUnloadHelper for combine unloaders
---@class SelfRefillHelper
SelfRefillHelper = {}
SelfRefillHelper.debugChannel = CpDebug.DBG_FIELDWORK
-- search for loaders/fillers within this distance from the field
SelfRefillHelper.maxDistanceFromField = 30

--- Find a loader/filler vehicle we can use for refilling (slurry tanker, liquid fertilizer trailer, etc.)
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

--- Get target parameters for approaching the loader (similar to SelfUnloadHelper for trailers)
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
    
    local _, _, dZ = localToLocal(loaderRootNode, targetNode, 0, 0, 0)
    
    -- Calculate approach parameters similar to trailer unloading
    -- Position vehicle beside the discharge node to receive liquid
    local offsetX = math.max(3.0, loaderWidth / 2) + (myVehicle.size.width / 2) + 1.5
    
    -- Determine which side to approach from based on discharge node position
    local nodeX, _, _ = localToLocal(targetNode, loaderRootNode, 0, 0, 0)
    if nodeX < 0 then
        offsetX = -offsetX  -- Approach from left side
    end
    
    -- Arrive at the loader alignLength meters behind the target to allow proper alignment
    local _, steeringLength = AIUtil.getSteeringParameters(myVehicle)
    local _, frontMarkerOffset = Markers.getFrontMarkerNode(myVehicle)
    local alignLength = (loaderLength / 2) + dZ + math.max(myVehicle.size.length / 2 + frontMarkerOffset, steeringLength)
    
    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, myVehicle,
            'Loader length: %.1f, width: %.1f, dZ: %.1f, align length %.1f, my length: %.1f, steering length %.1f, offsetX %.1f, frontMarkerOffset: %.2f',
            loaderLength, loaderWidth, dZ, alignLength, 
            myVehicle.size.length, steeringLength, offsetX, frontMarkerOffset)
    
    return targetNode, alignLength, offsetX, bestLoader
end

