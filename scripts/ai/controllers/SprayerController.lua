--- Controller for sprayers and fertilizer spreaders
--- Main motivation as of now is to turn the sprayer off while the vehicle is not moving
--- for whatever reason, for instance in a convoy waiting to start.
---@class SprayerController : ImplementController
SprayerController = CpObject(ImplementController)

--- Dummy placeholder for now
function SprayerController:init(vehicle, sprayer)
    self.sprayer = sprayer
    self.sprayerSpec = sprayer.spec_sprayer
    ImplementController.init(self, vehicle, self.sprayer)
    local fillUnitIndex = self.implement:getSprayerFillUnitIndex()
    if self.implement:getFillUnitCapacity(fillUnitIndex) > 0 then
        self:addRefillImplementAndFillUnit(self.implement, self.implement:getSprayerFillUnitIndex())
    end
    for _, supportedSprayType in ipairs(self.sprayerSpec.supportedSprayTypes) do
        for _, src in ipairs(self.sprayerSpec.fillTypeSources[supportedSprayType]) do
            self:debug("Found additional tank for refilling: %s|%d", src.vehicle, src.fillUnitIndex)
            if src.vehicle:getFillUnitCapacity(src.fillUnitIndex) > 0 and not src.vehicle.spec_sprayer then
                self:addRefillImplementAndFillUnit(src.vehicle, src.fillUnitIndex)
            end
        end
    end
end

--- Refill handling
-------------------------

function SprayerController:needsRefilling()
    -- If Giants helper buy options are enabled, no refilling needed (infinite fill)
    if self.sprayerSpec.isSlurryTanker and g_currentMission.missionInfo.helperSlurrySource > 1 or 
        self.sprayerSpec.isManureSpreader and g_currentMission.missionInfo.helperManureSource > 1 or 
        self.sprayerSpec.isFertilizerSprayer and g_currentMission.missionInfo.helperBuyFertilizer then 
        self:debug('SPRAYER: Helper auto-buy is enabled, no refilling needed')
        return false
    end
    
    ImplementUtil.hasFillLevelChanged(self.refillData.lastFillLevels)
    
    for implement, data in pairs(self.refillData.lastFillLevels) do 
        for fillUnitIndex, fillLevel in pairs(data) do
            local capacity = implement:getFillUnitCapacity(fillUnitIndex)
            if fillLevel <= 0 then
                self:debug('SPRAYER: Fill level is 0 -> needs refilling')
                return true
            end
        end
    end
    return false
end

function SprayerController:update()
    -- Update is called by the drive strategy for all controllers
end

-- Track which vehicles have already been stopped to avoid repeated stop calls
local stoppedVehicles = {}
-- Track when refill was last attempted to prevent endless retries
local refillAttemptTimestamps = {}

local function processSprayerArea(sprayer, superFunc, ...)
    local rootVehicle = sprayer.rootVehicle
    if rootVehicle.getIsCpActive and rootVehicle:getIsCpActive() then
        local specSpray = sprayer.spec_sprayer
        local sprayerParams = specSpray.workAreaParameters
        
        -- Check if sprayer is actually empty or very low (check real fill level, not sprayerParams which we manipulate)
        if not sprayer:getIsSprayerExternallyFilled() and rootVehicle:getLastSpeed() > 0.1 then
            local fillUnitIndex = sprayer:getSprayerFillUnitIndex()
            if fillUnitIndex then
                local fillLevel = sprayer:getFillUnitFillLevel(fillUnitIndex)
                local capacity = sprayer:getFillUnitCapacity(fillUnitIndex)
                
                -- Stop only when the tank is really empty.
                local stopThresholdPercentage = 0
                local stopThreshold = 0
                
                -- Only stop once per empty tank event
                if fillLevel <= stopThreshold and not stoppedVehicles[rootVehicle] then
                    CpUtil.info('=========================================')
                    CpUtil.info('SPRAYER: Stopping CP - tank almost empty!')
                    CpUtil.info('SPRAYER: Fill level: %.1f%% (threshold: %.1f%%)', 
                        (fillLevel / capacity) * 100, stopThresholdPercentage)
                    CpUtil.info('SPRAYER: Capacity: %.1f liters, Remaining: %.1f liters', capacity, fillLevel)
                    CpUtil.info('SPRAYER: Vehicle: %s', CpUtil.getName(rootVehicle))
                    CpUtil.info('SPRAYER: Calling stopCurrentAIJob with AIMessageErrorOutOfFill')
                    CpUtil.info('=========================================')
                    stoppedVehicles[rootVehicle] = true
                    rootVehicle:stopCurrentAIJob(AIMessageErrorOutOfFill.new())
                elseif fillLevel > 0 then
                    -- Reset the stopped flag when tank is no longer empty.
                    if stoppedVehicles[rootVehicle] then
                        CpUtil.debugImplement(CpDebug.DBG_IMPLEMENTS, sprayer, 
                            'SPRAYER: ✓ Tank no longer empty, ready for next cycle')
                    end
                    stoppedVehicles[rootVehicle] = nil
                    -- Also reset refill attempt timestamp to allow new attempts
                    refillAttemptTimestamps[rootVehicle] = nil
                end
            end
        end
        
        --- If the vehicle is standing, then disable the sprayer.
        if rootVehicle:getLastSpeed() < 0.1 then
            sprayerParams.sprayFillLevel = 0
            if sprayer:getFillUnitCapacity(sprayer.spec_sprayer.fillUnitIndex) <= 0 then 
                --- Needs to be set, as otherwise the sprayer will endlessly stop the fieldworker ...
                sprayerParams.sprayFillType = FillType.LIQUIDMANURE
            end
        end
    end
    return superFunc(sprayer, ...)
end
Sprayer.processSprayerArea = Utils.overwrittenFunction(Sprayer.processSprayerArea, processSprayerArea)