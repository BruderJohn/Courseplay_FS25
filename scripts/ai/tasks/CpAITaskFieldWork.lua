
---@class CpAITaskFieldWork : CpAITask
CpAITaskFieldWork = CpObject(CpAITask)

function CpAITaskFieldWork:reset()
	self.startPosition = nil
	self.waitingForRefillingActive = false
	self.drivingToLoaderActive = false
	self.refillStrategy = nil
	self.refillFailedTimestamp = nil
	self.refillCooldownMs = 30000  -- 30 seconds cooldown after refill failure
	self.savedFieldPolygon = nil  -- Save field polygon for refill strategy
	self.savedIslandPolygons = nil
	self.waitingForFieldBoundary = false  -- Waiting for field boundary detection to finish
	CpAITask.reset(self)
end

function CpAITaskFieldWork:setStartPosition(startPosition)
	self.startPosition = startPosition
end

function CpAITaskFieldWork:setWaitingForRefillingActive()
	local cpSpec = self.vehicle.spec_cpAIFieldWorker
	if not self.waitingForRefillingActive and cpSpec.driveStrategy then
		self.waitingForRefillingActive = true
		cpSpec.driveStrategy:raiseControllerEvent(
			AIDriveStrategyCourse.onStartRefillingEvent)
	end
end

function CpAITaskFieldWork:setDrivingToLoaderActive()
	local cpSpec = self.vehicle.spec_cpAIFieldWorker
	
	-- Check if we're in cooldown after a previous failure
	if self.refillFailedTimestamp then
		local timeSinceFailure = g_currentMission.time - self.refillFailedTimestamp
		if timeSinceFailure < self.refillCooldownMs then
			local remainingSeconds = math.ceil((self.refillCooldownMs - timeSinceFailure) / 1000)
			if g_updateLoopIndex % 100 == 0 then  -- Log every ~3 seconds
				self:debug('REFILL: In cooldown, waiting %d more seconds before next attempt', remainingSeconds)
			end
			return
		end
		-- Cooldown expired, clear flag
		self:debug('REFILL: Cooldown expired, ready for new refill attempt')
		self.refillFailedTimestamp = nil
	end
	
	if not self.drivingToLoaderActive and not self.waitingForFieldBoundary and cpSpec.driveStrategy then
		CpUtil.info('=========================================')
		CpUtil.info('REFILL: Starting automatic drive to loader')
		CpUtil.info('REFILL: Current waypoint: %d', cpSpec.driveStrategy.ppc:getCurrentWaypointIx())
		
		-- WICHTIG: Fahrzeug sofort anhalten während Strategie-Wechsel
		self.vehicle:cpHold(60000, true)  -- 60 Sekunden maximale Haltezeit
		CpUtil.info('REFILL: ✓ Vehicle STOPPED during strategy switch')
		
		-- Try to get field polygon
		CpUtil.info('REFILL: Checking for field polygon...')
		self.savedFieldPolygon = self.vehicle:cpGetFieldPolygon()
		self.savedIslandPolygons = self.vehicle:cpGetIslandPolygons()
		
		if self.savedFieldPolygon then
			CpUtil.info('REFILL: ✓ Field polygon available with %d vertices', #self.savedFieldPolygon)
			for i = 1, math.min(3, #self.savedFieldPolygon) do
				CpUtil.info('REFILL:   Vertex %d: x=%.1f, z=%.1f', i, self.savedFieldPolygon[i].x, self.savedFieldPolygon[i].z)
			end
		else
			-- No polygon available - need to detect field boundary first
			CpUtil.info('REFILL: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			CpUtil.info('REFILL: No field polygon available yet')
			CpUtil.info('REFILL: Starting field boundary detection NOW...')
			
			-- Check if detection is already running
			if self.vehicle:cpIsFieldBoundaryDetectionRunning() then
				CpUtil.info('REFILL: Detection already running, waiting for it to finish')
			else
				-- Start detection at vehicle's current position
				local x, _, z = getWorldTranslation(self.vehicle.rootNode)
				CpUtil.info('REFILL: Starting detection at vehicle position: %.1f, %.1f', x, z)
				
				-- Start the detection with a callback
				self.vehicle:cpDetectFieldBoundary(x, z, self, function(task, vehicle, fieldPolygon, islandPolygons)
					task.savedFieldPolygon = fieldPolygon
					task.savedIslandPolygons = islandPolygons
					task.waitingForFieldBoundary = false
					
					if fieldPolygon then
						CpUtil.info('REFILL: ✓✓✓ Field boundary detected with %d vertices', #fieldPolygon)
						CpUtil.info('REFILL: Now continuing with refill...')
					else
						CpUtil.info('REFILL: ERROR - Field boundary detection failed (no polygon returned)')
					end
				end)
				
				CpUtil.info('REFILL: Field boundary detection started, waiting for results...')
			end
			
			CpUtil.info('REFILL: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
			
			-- Set flag to wait for boundary detection
			self.waitingForFieldBoundary = true
			return  -- Don't proceed with refill yet
		end
		
		self.drivingToLoaderActive = true
		-- Save current course state
		self.savedCourse = cpSpec.driveStrategy.course
		self.savedWaypointIx = cpSpec.driveStrategy.ppc:getCurrentWaypointIx()
		CpUtil.info('REFILL: Saved course state for return (waypoint %d)', self.savedWaypointIx)
		
		-- Create and start refill strategy
		CpUtil.info('REFILL: Creating AIDriveStrategyRefillAtLoader')
		self.refillStrategy = AIDriveStrategyRefillAtLoader(self, self.job)
		CpUtil.info('REFILL: Strategy instance created')
		
		-- Pass the saved field polygon to the refill strategy BEFORE setAIVehicle
		-- because setAIVehicle triggers the first update which calls start()
		CpUtil.info('REFILL: Passing field polygon to strategy...')
		if self.savedFieldPolygon then
			CpUtil.info('REFILL: Passing field polygon with %d vertices', #self.savedFieldPolygon)
			self.refillStrategy:setFieldPolygon(self.savedFieldPolygon, self.savedIslandPolygons)
			CpUtil.info('REFILL: ✓ Field polygon passed successfully')
		else
			CpUtil.info('REFILL: !!!!! WARNING - No field polygon to pass !!!!!')
		end
		
		-- Pass the saved course and waypoint for return after refilling
		CpUtil.info('REFILL: Passing saved course and waypoint to strategy...')
		if self.savedCourse and self.savedWaypointIx then
			self.refillStrategy:setSavedCourse(self.savedCourse, self.savedWaypointIx)
			CpUtil.info('REFILL: ✓ Saved course passed successfully')
		else
			CpUtil.info('REFILL: !!!!! WARNING - No saved course to pass !!!!!')
		end
		
		CpUtil.info('REFILL: Calling setAIVehicle...')
		self.refillStrategy:setAIVehicle(self.vehicle, self.job:getCpJobParameters())
		CpUtil.info('REFILL: setAIVehicle done')

		-- IMPORTANT: replace active AIFieldWorker strategy list as well,
		-- otherwise the old fieldwork strategy may keep running in parallel and trigger off-track stops.
		local aiSpec = self.vehicle.spec_aiFieldWorker
		if aiSpec and aiSpec.driveStrategies then
			for i = #aiSpec.driveStrategies, 1, -1 do
				aiSpec.driveStrategies[i]:delete()
				table.remove(aiSpec.driveStrategies, i)
			end
		else
			aiSpec.driveStrategies = {}
		end
		table.insert(aiSpec.driveStrategies, self.refillStrategy)
		cpSpec.driveStrategy = self.refillStrategy
		
		CpUtil.info('REFILL: Calling startCpWithStrategy...')
		self.vehicle:startCpWithStrategy(self.refillStrategy)
		CpUtil.info('REFILL: ✓ Refill strategy started successfully')
		CpUtil.info('=========================================')
	end
end

function CpAITaskFieldWork:update(dt)
	-- Safety check: vehicle might be nil after job has been stopped
	if not self.vehicle then
		return
	end
	
	-- Check if we're waiting for field boundary detection
	if self.waitingForFieldBoundary then
		if not self.vehicle:cpIsFieldBoundaryDetectionRunning() then
			-- Detection finished! Check if we got a polygon
			self.savedFieldPolygon = self.vehicle:cpGetFieldPolygon()
			self.savedIslandPolygons = self.vehicle:cpGetIslandPolygons()
			
			if self.savedFieldPolygon then
				CpUtil.info('REFILL: ✓✓✓ Field boundary detection complete!')
				CpUtil.info('REFILL: Got polygon with %d vertices', #self.savedFieldPolygon)
				CpUtil.info('REFILL: Now continuing with refill process...')
				self.waitingForFieldBoundary = false
				-- Continue with refill now that we have the polygon
				self:setDrivingToLoaderActive()
			else
				CpUtil.info('REFILL: ERROR - Field boundary detection completed but returned no polygon')
				CpUtil.info('REFILL: This is very unusual - stopping job')
				self.waitingForFieldBoundary = false
				self.refillFailedTimestamp = g_currentMission.time
				self.vehicle:stopCurrentAIJob(AIMessageErrorOutOfFill.new())
			end
		else
			-- Still waiting - keep vehicle stopped
			self.vehicle:cpHold(1500, true)
			self.vehicle:setCpInfoTextActive(InfoTextManager.WAITING)
			if g_updateLoopIndex % 100 == 0 then  -- Every ~3 seconds
				CpUtil.info('REFILL: Still waiting for field boundary detection...')
			end
		end
		return  -- Don't do anything else while waiting
	end
	
	-- Hack to reevaluate the refill condition for the new setting state after it changed.
	local settingWasChanged = false
	if self.lastSettingValues then 
		if self.lastSettingValues.optionalFertilizer ~= self.vehicle:getCpSettings().sowingMachineFertilizerEnabled:getValue() then
			settingWasChanged = true
		end
		if self.lastSettingValues.optionalSowing ~= self.vehicle:getCpSettings().optionalSowingMachineEnabled:getValue() then
			settingWasChanged = true
		end
	end

	if self.waitingForRefillingActive then 
		self.vehicle:cpHold(1500, true)
		local cpSpec = self.vehicle.spec_cpAIFieldWorker
		self.vehicle:setCpInfoTextActive(InfoTextManager.NEEDS_FILLING)

		local readyToContinue, fillLevelHasChanged = true, false
		cpSpec.driveStrategy:raiseControllerEventWithLambda(
			AIDriveStrategyCourse.onUpdateRefillingEvent,
			function(timerHasFinished, hasChanged)
				readyToContinue = readyToContinue and timerHasFinished
				fillLevelHasChanged = fillLevelHasChanged or hasChanged
			end)
		if readyToContinue and fillLevelHasChanged or settingWasChanged then
			cpSpec.driveStrategy:raiseControllerEvent(
				AIDriveStrategyCourse.onStopRefillingEvent)
			self.waitingForRefillingActive = false
			self.vehicle:resetCpActiveInfoText(InfoTextManager.NEEDS_FILLING)
		end
	end

	if self.drivingToLoaderActive then
		local cpSpec = self.vehicle.spec_cpAIFieldWorker
		
		-- Check if we're waiting for a loader to appear
		if self.refillStrategy and self.refillStrategy.state == self.refillStrategy.states.WAITING_FOR_LOADER then
			-- Just wait, vehicle is stopped and searching for loader every 5 seconds
			-- Don't stop the job, let it continue waiting
			self.vehicle:setCpInfoTextActive(InfoTextManager.NEEDS_FILLING)
			if g_updateLoopIndex % 200 == 0 then  -- Every ~6 seconds
				self:debug('REFILL: Waiting for suitable loader to appear at field edge...')
			end
			return  -- Continue waiting without stopping job
		end
		
		-- Check if we're waiting for implements to fold
		if self.refillStrategy and self.refillStrategy.state == self.refillStrategy.states.WAITING_FOR_FOLD then
			-- Just wait, vehicle is stopped and implements are folding
			-- Don't stop the job, let it continue waiting
			self.vehicle:setCpInfoTextActive(InfoTextManager.NEEDS_FILLING)
			if g_updateLoopIndex % 100 == 0 then  -- Every ~3 seconds
				self:debug('REFILL: Waiting for implements to fold completely...')
			end
			return  -- Continue waiting without stopping job
		end
		
		-- Check if refill strategy has finished
		if self.refillStrategy and self.refillStrategy.state == self.refillStrategy.states.REFILL_COMPLETE then
			self:debug('=========================================')
			
			-- Check if refill actually succeeded or failed
			local refillSucceeded = self.refillStrategy.refillSucceeded or false
			
			if refillSucceeded then
				self:debug('REFILL: Refill completed successfully, returning to fieldwork')
				-- Clear the "needs filling" error message
				self.vehicle:resetCpAllActiveInfoTexts()
				-- Restart fieldwork strategy with saved course
				if self.savedCourse then
					self:debug('REFILL: Restoring fieldwork course at waypoint %d', self.savedWaypointIx)
					local strategy = AIDriveStrategyFieldWorkCourse(self, self.job)
					strategy:setAIVehicle(self.vehicle, self.job:getCpJobParameters())
					strategy:start(self.savedCourse, self.savedWaypointIx, self.job:getCpJobParameters())
					local aiSpec = self.vehicle.spec_aiFieldWorker
					local cpSpec = self.vehicle.spec_cpAIFieldWorker
					if aiSpec and aiSpec.driveStrategies then
						for i = #aiSpec.driveStrategies, 1, -1 do
							aiSpec.driveStrategies[i]:delete()
							table.remove(aiSpec.driveStrategies, i)
						end
					else
						aiSpec.driveStrategies = {}
					end
					table.insert(aiSpec.driveStrategies, strategy)
					cpSpec.driveStrategy = strategy
					self.vehicle:startCpWithStrategy(strategy)
					self.savedCourse = nil
					self.savedWaypointIx = nil
					self:debug('REFILL: Fieldwork resumed successfully')
				else
					self:debug('REFILL: ERROR - No saved course found!')
				end
			else
				self:debug('REFILL: Refill FAILED (no loader found or other error)')
				self:debug('REFILL: Setting cooldown period of %d seconds', self.refillCooldownMs / 1000)
				self:debug('REFILL: Vehicle will STOP and wait for manual intervention')
				self:debug('REFILL: You can:')
				self:debug('REFILL:   - Manually drive to a tanker to refill')
				self:debug('REFILL:   - Place a tanker at field edge and restart CP')
				self:debug('REFILL:   - Wait for automatic retry after cooldown')
				self.refillFailedTimestamp = g_currentMission.time
				-- Stop the job completely - do not restart fieldwork
				-- The user needs to manually refill or place a tanker
				self:debug('REFILL: Stopping AI job due to failed refill attempt')
				self.vehicle:stopCurrentAIJob(AIMessageErrorOutOfFill.new())
				self.savedCourse = nil
				self.savedWaypointIx = nil
			end
			
			self.drivingToLoaderActive = false
			self.refillStrategy = nil
			self:debug('=========================================')
		elseif self.refillStrategy then
			-- Log current state
			local stateName = self.refillStrategy.state and self.refillStrategy.state.name or 'UNKNOWN'
			if g_updateLoopIndex % 100 == 0 then  -- Every ~3 seconds
				self:debug('REFILL: Current state: %s', stateName)
			end
		end
	end
	
	-- Store settings values for change detection (only if vehicle is still valid)
	if self.vehicle then
		self.lastSettingValues = {
			optionalFertilizer = self.vehicle:getCpSettings().sowingMachineFertilizerEnabled:getValue(),
			optionalSowing = self.vehicle:getCpSettings().optionalSowingMachineEnabled:getValue()
		}
	end
end

--- This function can be used to trace the triggering of AI events. The timing of these is critical for multiplayer
--- to work properly, these traces help to determine if this timing is correct. Bad timing will result in implements
--- not lowering or not turning on in multiplayer, while there are no symptoms in single player.
---
--- The game engine needs a separate phase for preparing to make sure that the
---   * onAIFieldWorkerStart,
---   * onAIFieldWorkerPrepareForWork,
---   * onAIImplementStartLine
--- events are generated one by one, one in each update loop so that at the end of the loop,
--- AIFieldWorker:updateAIFieldWorker() triggers a onAIFieldWorkerActive event.
---
function CpAITaskFieldWork:turnOnAIEventTrace()
	self.vehicle.raiseAIEvent = function(vehicle, event1, event2, ...)
		if vehicle.cpLastRaiseAIEvent ~= event1 and vehicle.cpLastRaiseAIEvent2 ~= event2 then
			CpUtil.infoVehicle(vehicle, "raiseAIEvent %s %s", event1, event2)
		end
		vehicle.cpLastRaiseAIEvent1 = event1
		vehicle.cpLastRaiseAIEvent2 = event2
		AIVehicle.raiseAIEvent(vehicle, event1, event2, ...)
	end

	if self.vehicle.actionController ~= nil then
		self.vehicle.actionController.onAIEvent = function(actionController, sourceVehicle, eventName)
			if eventName ~= 'onAIFieldWorkerActive' and eventName ~= 'onAIImplementActive' then
				CpUtil.infoVehicle(self.vehicle, "   onAIEvent %s, source %s", eventName, CpUtil.getName(sourceVehicle))
			end
			VehicleActionController.onAIEvent(actionController, sourceVehicle, eventName)
		end
	end
end

--- Makes sure the cp fieldworker gets started.
function CpAITaskFieldWork:start()
	CpUtil.info("=========================================")
	CpUtil.info("FIELD WORK: Task START() called")
	CpUtil.info("FIELD WORK: Field polygon will be captured at refill time")
	CpUtil.info("FIELD WORK: (Field boundary detection may not be complete yet)")
	CpUtil.info("=========================================")
	
	local spec = self.vehicle.spec_aiFieldWorker
	spec.isActive = true
	if self.isServer then
		self.vehicle:updateAIFieldWorkerImplementData()
		self.vehicle:raiseAIEvent("onAIFieldWorkerStart", "onAIImplementStart")
		if self.vehicle:getAINeedsTrafficCollisionBox() and (AIFieldWorker.TRAFFIC_COLLISION ~= nil and
			(AIFieldWorker.TRAFFIC_COLLISION ~= 0 and spec.aiTrafficCollision == nil)) then

			spec.aiTrafficCollision = clone(AIFieldWorker.TRAFFIC_COLLISION, true, false, true)
		end
		local cpSpec = self.vehicle.spec_cpAIFieldWorker
		--- Remembers the last lane offset setting value that was used.
        cpSpec.cpJobStartAtLastWp:getCpJobParameters().laneOffset:setValue(self.job:getCpJobParameters().laneOffset:getValue())
		if spec.driveStrategies ~= nil then
			-- This deletion code could be removed, but just to be sure we let it stay here for now.
			for i = #spec.driveStrategies, 1, -1 do
				spec.driveStrategies[i]:delete()
				table.remove(spec.driveStrategies, i)
			end
			spec.driveStrategies = {}
		end
		local cpDriveStrategy
		if self.startPosition and g_vineScanner:hasVineNodesCloseBy(self.startPosition.x, self.startPosition.z) then
			--- Checks if there are any vine nodes close to the starting point.
			self:debug('Found a vine course, install CP vine fieldwork drive strategy for it')
			cpDriveStrategy = AIDriveStrategyVineFieldWorkCourse(self, self.job)
		elseif AIUtil.hasChildVehicleWithSpecialization(self.vehicle, Plow) then
			self:debug('Found a plow, install CP plow drive strategy for it')
			cpDriveStrategy = AIDriveStrategyPlowCourse(self, self.job)
		else
			local combine = AIUtil.getImplementOrVehicleWithSpecialization(self.vehicle, Combine)
			local pipe = combine and SpecializationUtil.hasSpecialization(Pipe, combine.specializations)
			if combine and pipe then
				-- Default harvesters with a pipe.
				self:debug('Found a combine with pipe, install CP combine drive strategy for it')
				cpDriveStrategy = AIDriveStrategyCombineCourse(self, self.job)
				cpSpec.combineDriveStrategy = cpDriveStrategy
			end
			if not cpDriveStrategy then
				self:debug('Installing default CP fieldwork drive strategy')
				cpDriveStrategy = AIDriveStrategyFieldWorkCourse(self, self.job)
			end
		end
		cpDriveStrategy:setAIVehicle(self.vehicle, self.job:getCpJobParameters())
		cpSpec.driveStrategy = cpDriveStrategy
		--- Only the last driving strategy can stop the helper, while it is running.
		table.insert(spec.driveStrategies, cpDriveStrategy)
	else
		self.vehicle:raiseAIEvent("onAIFieldWorkerStart", "onAIImplementStart")
	end
	CpAITask.start(self)
end

function CpAITaskFieldWork:stop(wasJobStopped)
	local wasWaitingForRefilling = self.waitingForRefillingActive

	-- Always clear temporary refill/hold states first, especially when stopping mid-refill.
	self.waitingForFieldBoundary = false
	self.drivingToLoaderActive = false

	if wasWaitingForRefilling then
		local cpSpec = self.vehicle.spec_cpAIFieldWorker
		cpSpec.driveStrategy:raiseControllerEvent(
				AIDriveStrategyCourse.onStopRefillingEvent)
	end
	self.waitingForRefillingActive = false

	if self.refillStrategy and self.refillStrategy.finishRefilling then
		self.refillStrategy:finishRefilling()
	end
	self.refillStrategy = nil
	if self.isServer then
		self:debug("Field work task stopped.")
		self.vehicle:stopFieldWorker()
		-- Important: stop and delete CP drive strategy as well to release any active hold/freeze states.
		self.vehicle:stopCpDriver(wasJobStopped)
		self.vehicle:cpBrakeToStop()
	end
	CpAITask.stop(self, wasJobStopped)
end
