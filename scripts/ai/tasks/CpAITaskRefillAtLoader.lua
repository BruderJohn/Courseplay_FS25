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

---@class CpAITaskRefillAtLoader : CpAITask
CpAITaskRefillAtLoader = CpObject(CpAITask)

function CpAITaskRefillAtLoader:reset()
	self.foundLoader = nil
	CpAITask.reset(self)
end

function CpAITaskRefillAtLoader:start()
    if self.isServer then
        self:debug('CP refill at loader task started')
        local strategy = AIDriveStrategyRefillAtLoader(self, self.job)
        strategy:setAIVehicle(self.vehicle, self.job:getCpJobParameters())
        self.vehicle:startCpWithStrategy(strategy)
    end
    CpAITask.start(self)
end

function CpAITaskRefillAtLoader:stop(wasJobStopped)
    if self.isServer then
        self:debug('CP refill at loader task stopped')
        self.vehicle:stopCpDriver(wasJobStopped)
    end
    CpAITask.stop(self)
end

function CpAITaskRefillAtLoader:update(dt)
	if self.isServer and not self.foundLoader then
		-- Try to find a loader only once
		local vehicle = self.vehicle
		local fieldPolygon = vehicle:cpGetFieldPolygon()
		if fieldPolygon then
			-- Get the fill type we need to refill
			local fillTypeIndex = self:getRequiredFillType()
			if fillTypeIndex then
				local loader, dischargeNode = SelfRefillHelper:findBestLoader(fieldPolygon, vehicle, fillTypeIndex)
				self.foundLoader = loader ~= nil
				if not self.foundLoader then
					self:debug('No loader found, skipping refill task')
					self:skip()
				end
			else
				self:debug('Could not determine required fill type, skipping refill task')
				self:skip()
			end
		else
			self:debug('No field polygon available, skipping refill task')
			self:skip()
		end
	end
	CpAITask.update(self, dt)
end

--- Get the fill type that the vehicle needs to refill
---@return number|nil fill type index
function CpAITaskRefillAtLoader:getRequiredFillType()
	local vehicle = self.vehicle
	
	-- Check for sprayer
	if SpecializationUtil.hasSpecialization(Sprayer, vehicle.specializations) then
		local sprayerSpec = vehicle.spec_sprayer
		if sprayerSpec then
			local fillType = vehicle:getFillUnitFillType(sprayerSpec.fillUnitIndex)
			if fillType and fillType ~= FillType.UNKNOWN then
				return fillType
			end
			-- Try to get supported fill types
			if sprayerSpec.supportedSprayTypes and #sprayerSpec.supportedSprayTypes > 0 then
				return sprayerSpec.supportedSprayTypes[1]
			end
		end
	end
	
	-- Check for manure spreader or slurry tanker
	if SpecializationUtil.hasSpecialization(ManureBarrel, vehicle.specializations) or
	   SpecializationUtil.hasSpecialization(Sprayer, vehicle.specializations) then
		-- Get the first fill unit's fill type
		local fillUnits = vehicle:getFillUnits()
		if fillUnits then
			for fillUnitIndex, fillUnit in pairs(fillUnits) do
				local fillType = vehicle:getFillUnitFillType(fillUnitIndex)
				if fillType and fillType ~= FillType.UNKNOWN then
					return fillType
				end
			end
		end
	end
	
	-- Check attached implements
	local implements = AIUtil.getAllChildVehiclesWithSpecialization(vehicle, Sprayer)
	if implements then
		for _, implement in pairs(implements) do
			if implement.spec_sprayer then
				local fillType = implement:getFillUnitFillType(implement.spec_sprayer.fillUnitIndex)
				if fillType and fillType ~= FillType.UNKNOWN then
					return fillType
				end
			end
		end
	end
	
	-- Try to get any fill type from fill units
	local fillUnits = vehicle:getFillUnits()
	if fillUnits then
		for fillUnitIndex, fillUnit in pairs(fillUnits) do
			local capacity = vehicle:getFillUnitCapacity(fillUnitIndex)
			if capacity and capacity > 0 then
				local fillLevel = vehicle:getFillUnitFillLevel(fillUnitIndex)
				local fillType = vehicle:getFillUnitFillType(fillUnitIndex)
				if fillType and fillType ~= FillType.UNKNOWN then
					return fillType
				end
				-- If empty, try to find supported fill types
				if fillLevel <= 0 and vehicle.getFillUnitSupportedFillTypes then
					local supportedFillTypes = vehicle:getFillUnitSupportedFillTypes(fillUnitIndex)
					if supportedFillTypes then
						for supportedFillType, _ in pairs(supportedFillTypes) do
							-- Return first liquid type found
							if g_fillTypeManager:getFillTypeByIndex(supportedFillType).isLiquid then
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
