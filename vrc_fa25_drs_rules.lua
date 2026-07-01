-- VRC FA25 DRS Rules — V2.0 (FIA timing-loop model, cooperative suppression)
-- ============================================================================
-- WHY THIS REWRITE:
--   * The VRC25 car ships its own per-frame DRS controller (data.acd/script_DRS.lua,
--     dispatched every physics frame by script_ecu.lua). For AI it does, each frame:
--         ac.allowCarDRS(false); ac.setDRS(car.drsAvailable and onThrottle)
--     so any AI in a DRS zone opens DRS regardless of the 1s gap.
--   * V1.x fought that by force-closing/force-opening AND by suppressing EVERY managed
--     car for ~95% of the lap (everywhere outside a DRS window). On this CSP build that
--     left cars stuck in "scripted" state -> DRS never opened for anyone. Wrong approach.
--
-- THE MODEL THIS VERSION IMPLEMENTS (matches real F1):
--   1. DETECTION POINT != zone start. Each zone has a DETECTION spline (upstream) and a
--      [START..END] activation window (downstream). The gap is sampled ONCE, the instant a
--      car crosses the DETECTION line, and LATCHED for the zone(s) fed by that point.
--   2. GAP = FIA timing-loop method: time between the car AHEAD crossing the detection point
--      and THIS car crossing the SAME point (seconds). Speed-independent, accurate. (Not the
--      distance/speed approximation getGapBetweenCars uses, which inflates at low-speed points.)
--      "Car ahead" = the car that physically crossed that point most recently before us.
--   3. AUTHORIZATION ONLY. We decide one thing: does this car have DRS in this zone (gap<=1s)?
--      The open/close/re-open dynamics inside an authorized zone (brake closes it, re-press
--      reopens) stay with VRC/native — that's already realistic, we don't touch it.
--   4. COOPERATIVE SUPPRESSION. To deny DRS we call physics.allowCarDRS(i,false): this makes
--      DRS UNAVAILABLE, so VRC's own script reads drsAvailable=false and closes the wing ITSELF.
--      No tug-of-war. We NEVER force DRS open (that direction doesn't hold on this build).
--   5. MINIMAL TOUCH. We only ever call physics on a car that is INSIDE a window AND failed the
--      gate. Authorized cars, cars outside windows, and cars on tracks with no zones are NEVER
--      touched -> native/VRC controls them -> nothing gets stuck.
--
-- Read-only w.r.t. car/track files (no checksum impact) -> league-safe.
-- Requires the track's surfaces.ini [_SCRIPTING_PHYSICS] ALLOW_APPS=1 (Suzuka & Melbourne have it).
-- ============================================================================

local APPLY_AI   = 0
local APPLY_USER = 1
local APPLY_ALL  = 2
local APPLY_LABELS = { [APPLY_AI] = 'AI only', [APPLY_USER] = 'Player only', [APPLY_ALL] = 'All cars' }

local cfg = ac.storage({
  enabled       = true,        -- master switch
  applyTo       = APPLY_ALL,   -- who the rule is enforced on (default: everyone, FIA-fair)
  gapSeconds    = 1.0,         -- max time gap to car ahead at the detection line (FIA = 1.0s)
  firstDrsLap   = 2,           -- DRS permitted from this lap onward (current lap >= this)
  raceOnly      = true,        -- only enforce in the Race session
  lappedTrigger = true,        -- ON = a lapped car ahead can still grant DRS (FIA). OFF = same-lap only.
  leaderNoDrs   = false,       -- optional league flavour: race leader (P1) never gets DRS
  debug         = false,       -- live debug table in the window
  logToFile     = false,       -- write every detection crossing + suppress anomalies to the CSP log (ac.log)
})

-- ---- track-derived state (rebuilt per session) -----------------------------
local zones       = nil   -- { {det,start,finish}, ... } from drs_zones.ini
local detList     = nil   -- distinct detection points: { {val=spline, zones={zi,...}}, ... }
local haveZones   = false

-- ---- per-car / per-detection-point runtime state ---------------------------
local now         = 0     -- monotonic sim clock (accumulated dt), seconds
local prevSpline  = {}     -- [carIdx] = spline last frame (for crossing detection)
local armed       = {}     -- [carIdx][zoneIdx] = latched decision from that zone's detection line
local lastCross   = {}     -- [detIdx] = { t=time, car=idx, lap=lapCount } of the last car over that point
local touched     = {}     -- set of car indices we are currently suppressing (for clean release)
local detGapDbg   = {}     -- [carIdx] = gap (s) captured at last detection crossing (debug)
local detCntDbg   = {}     -- [carIdx] = number of detection crossings (debug)
local suppOpenLogged = {}  -- [carIdx] dedupe flag: already logged a "BLOCK but wing open" anomaly

local debugRows   = {}
local statusText  = 'Idle'

-- Load and pre-process the current track's DRS zones (trackData is slow -> cache).
local function loadZones()
  zones, detList, haveZones = {}, {}, false
  local ini = ac.INIConfig.trackData('drs_zones.ini')
  if ini == nil then return end
  local byDet = {}           -- "%.4f"(det) -> detList entry, to merge zones that share a point
  local z = 0
  while true do
    local det = ini:get('ZONE_' .. z, 'DETECTION', -1)
    if det == nil or det < 0 then break end
    local start  = ini:get('ZONE_' .. z, 'START', -1)
    local finish = ini:get('ZONE_' .. z, 'END', -1)
    zones[#zones + 1] = { det = det, start = start, finish = finish }
    local zi = #zones
    local key = string.format('%.4f', det)
    local e = byDet[key]
    if e == nil then
      e = { val = det, zones = {} }
      byDet[key] = e
      detList[#detList + 1] = e
    end
    e.zones[#e.zones + 1] = zi   -- this detection point feeds zone zi
    z = z + 1
  end
  haveZones = #zones > 0
end

-- spline crossed point p between last frame (a) and this frame (b)? wrap-safe.
local function crossed(a, b, p)
  if b >= a then return a < p and p <= b end
  return p > a or p <= b
end

-- spline s inside [a..b]? wrap-safe.
local function inRange(s, a, b)
  if a <= b then return s >= a and s <= b end
  return s >= a or s <= b
end

-- Shortest spline arc between two positions (0..0.5). Real per-frame motion is tiny (<0.03 even at low fps);
-- a value far above that means the car teleported -- a DNF/pit reset, or a bugged "ghost" car whose spline
-- oscillates across the track. We skip detection-crossing logic on such frames so a ghost cannot fire
-- spurious crossings, poison the timing loop (granting real cars false DRS), or flood the log.
local MAX_SPLINE_STEP = 0.1
local function splineStep(a, b)
  local d = math.abs(b - a)
  if d > 0.5 then d = 1 - d end
  return d
end

local function enforcedCar(c)
  if cfg.applyTo == APPLY_ALL then return true end
  if cfg.applyTo == APPLY_AI then return c.isAIControlled end
  return not c.isAIControlled
end

-- Decision at a detection point: is THIS car within the gap of the car that last crossed it?
-- Returns ok(boolean), gap(number|nil).
local function decideAtDetection(carIdx, carLap, detEntryIdx)
  local lc = lastCross[detEntryIdx]
  if lc == nil or lc.car == carIdx then return false, nil end   -- no valid car ahead (covers P1 / solo)
  local gap = now - lc.t                                         -- true time interval over the same point
  if gap <= 0 or gap > cfg.gapSeconds then return false, gap end -- >1s (subsumes any "too far / lapped-away")
  if not cfg.lappedTrigger and carLap ~= lc.lap then return false, gap end  -- OFF: ignore cross-lap cars
  return true, gap
end

local function releaseAll()
  for i in pairs(touched) do physics.allowCarDRS(i, false) end  -- false = ALLOW here (hand back to native)
  touched = {}
end

local function resetSession()
  releaseAll()                  -- hand any suppressed car back to native BEFORE wiping touched
  zones = nil; detList = nil; haveZones = false
  now = 0
  prevSpline = {}; armed = {}; lastCross = {}
  detGapDbg = {}; detCntDbg = {}; suppOpenLogged = {}
end

function script.update(dt)
  if not cfg.enabled then releaseAll(); statusText = 'Disabled'; return end

  local sim = ac.getSim()
  if cfg.raceOnly and sim.raceSessionType ~= ac.SessionType.Race then
    releaseAll(); statusText = 'Idle (not a race session — DRS left free)'; return
  end

  if zones == nil then loadZones() end
  now = now + dt
  local n = sim.carsCount

  -- ===== PASS 1: detection-line crossings for ALL cars (the car ahead may be unmanaged) =====
  if haveZones then
    for i = 0, n - 1 do
      local c = ac.getCar(i)
      if c ~= nil and c.isActive then
        local s  = c.splinePosition
        local ps = prevSpline[i]
        if ps ~= nil and not c.isInPitlane and splineStep(ps, s) <= MAX_SPLINE_STEP then
          for di, d in ipairs(detList) do
            if crossed(ps, s, d.val) then
              local prevLc = lastCross[di]   -- the car that armed us (captured before we overwrite)
              local ok, gap = decideAtDetection(i, c.lapCount, di)
              for _, zi in ipairs(d.zones) do
                armed[i] = armed[i] or {}
                armed[i][zi] = ok
              end
              lastCross[di] = { t = now, car = i, lap = c.lapCount }  -- we are now the "ahead car"
              detGapDbg[i] = gap
              detCntDbg[i] = (detCntDbg[i] or 0) + 1
              if cfg.logToFile then
                local nm = (ac.getDriverName and ac.getDriverName(i)) or ('Car' .. i)
                local ah = prevLc
                  and (((ac.getDriverName and ac.getDriverName(prevLc.car)) or ('Car' .. prevLc.car)) .. ' L' .. prevLc.lap)
                  or 'none'
                ac.log(string.format(
                  '[VRCDRS] t=%.2f det#%d@%.3f  %s P%d L%d %s  gap=%s  ahead=%s  -> %s  zones{%s}',
                  now, di, d.val, nm, c.racePosition, c.lapCount,
                  c.isAIControlled and 'AI' or 'HUMAN',
                  gap and string.format('%.2f', gap) or 'nil', ah,
                  ok and 'ARM' or 'deny', table.concat(d.zones, ',')))
              end
            end
          end
        end
        prevSpline[i] = s
      end
    end
  end

  -- ===== PASS 2: act only on managed cars that are inside a window and not authorized =====
  if cfg.debug then debugRows = {} end
  local governed = 0

  -- race-order map so the debug overlay can show the LIVE gap to the car ahead (for comparison
  -- against Det@, the gap captured upstream at the detection line — they legitimately differ).
  local posToIndex = {}
  if cfg.debug then
    for j = 0, n - 1 do
      local cj = ac.getCar(j)
      if cj ~= nil and cj.isActive then posToIndex[cj.racePosition] = j end
    end
  end

  for i = 0, n - 1 do
    local c = ac.getCar(i)
    if c ~= nil and c.isActive then
      local suppress = false
      if enforcedCar(c) then
        governed = governed + 1
        local s = c.splinePosition

        -- which DRS window (if any) is this car currently in?
        local winZone = nil
        if haveZones then
          for zi, z in ipairs(zones) do
            if inRange(s, z.start, z.finish) then winZone = zi; break end
          end
        end

        local authorized = false
        if winZone ~= nil then
          local lapOK = (c.lapCount + 1) >= cfg.firstDrsLap
          local armOK = armed[i] ~= nil and armed[i][winZone] == true
          authorized = armOK and lapOK and not c.isInPitlane
                       and not (cfg.leaderNoDrs and c.racePosition == 1)
          suppress = not authorized
        end

        if cfg.debug then
          local aheadIdx = posToIndex[c.racePosition - 1]
          local live = aheadIdx and math.abs(ac.getGapBetweenCars(i, aheadIdx)) or nil
          debugRows[#debugRows + 1] = {
            pos   = c.racePosition,
            ai    = c.isAIControlled,
            name  = (ac.getDriverName and ac.getDriverName(i)) or ('Car ' .. i),
            live  = live,                 -- gap to car ahead RIGHT NOW (what you feel on the straight)
            detg  = detGapDbg[i],         -- gap captured upstream at the detection line (what decides DRS)
            dets  = detCntDbg[i] or 0,
            inWin = winZone ~= nil,
            auth  = authorized,
            supp  = suppress,
          }
        end
      end

      -- Control = V2.5: allowCarDRS ONLY, MINIMAL TOUCH. OBSERVED semantics on this VRC/CSP build are
      -- INVERTED from the SDK doc: allowCarDRS(i,true) FORBIDS DRS (it becomes unavailable -> the "DRS"
      -- light never lights, the clean way to deny); allowCarDRS(i,false) hands the car back to native/VRC
      -- (VRC re-allows AI every physics frame anyway). We never use setCarDRS (it leaves DRS "available"
      -- -> the ugly "light on, press flashes off" the user rejected).
      -- We assert FORBID only while actively suppressing, and release a car exactly ONCE when it stops
      -- being suppressed (authorized / left the window / no longer enforced by Apply-to). This keeps
      -- `touched` == the currently-suppressed set, so releaseAll()/resetSession() stay correct and a
      -- mid-race Apply-to change cannot strand a previously-forbidden car.
      if suppress then
        physics.allowCarDRS(i, true)
        touched[i] = true
      elseif touched[i] then
        physics.allowCarDRS(i, false)
        touched[i] = nil
      end

      if cfg.logToFile then
        if suppress and c.drsActive then          -- we are forbidding, yet the wing is open
          if not suppOpenLogged[i] then
            suppOpenLogged[i] = true
            local nm = (ac.getDriverName and ac.getDriverName(i)) or ('Car' .. i)
            ac.log(string.format('[VRCDRS] !! BLOCK-BUT-OPEN t=%.2f %s P%d L%d %s (drsActive while suppressed)',
              now, nm, c.racePosition, c.lapCount, c.isAIControlled and 'AI' or 'HUMAN'))
          end
        else
          suppOpenLogged[i] = nil
        end
      end
    end
  end

  statusText = string.format('%s | %d car(s) | %d zone%s / %d det | gap %.2fs | from lap %d%s',
    haveZones and 'Enforcing' or 'No DRS zones on this track',
    governed, zones and #zones or 0, (zones and #zones == 1) and '' or 's',
    detList and #detList or 0, cfg.gapSeconds, cfg.firstDrsLap,
    cfg.lappedTrigger and '' or ' | same-lap only')
end

if ac.onSessionStart then ac.onSessionStart(resetSession) end

function script.windowMain(dt)
  ui.text('VRC FA25 DRS Rules — V2.6 (ghost-guard + diag log)')
  ui.separator()

  if ui.checkbox('Enable DRS rules', cfg.enabled) then cfg.enabled = not cfg.enabled end

  if ui.combo('Apply to', APPLY_LABELS[cfg.applyTo], ui.ComboFlags.None, function()
    for _, k in ipairs({ APPLY_AI, APPLY_USER, APPLY_ALL }) do
      if ui.selectable(APPLY_LABELS[k], cfg.applyTo == k) then cfg.applyTo = k end
    end
  end) then end

  cfg.gapSeconds  = ui.slider('##gap', cfg.gapSeconds, 0.3, 3.0, 'DRS gap: %.2f s')
  cfg.firstDrsLap = math.floor(ui.slider('##firstlap', cfg.firstDrsLap, 1, 5, 'First DRS lap: %.0f', true) + 0.5)

  if ui.checkbox('Race session only', cfg.raceOnly) then cfg.raceOnly = not cfg.raceOnly end
  if ui.checkbox('Lapped car can grant DRS (FIA)', cfg.lappedTrigger) then cfg.lappedTrigger = not cfg.lappedTrigger end
  if ui.checkbox('Race leader never gets DRS', cfg.leaderNoDrs) then cfg.leaderNoDrs = not cfg.leaderNoDrs end
  if ui.checkbox('Debug overlay', cfg.debug) then cfg.debug = not cfg.debug end
  if ui.checkbox('Log detections to CSP log', cfg.logToFile) then cfg.logToFile = not cfg.logToFile end

  ui.separator()
  ui.textColored(statusText, cfg.enabled and rgbm(0.6, 1, 0.6, 1) or rgbm(1, 0.7, 0.5, 1))

  if cfg.debug and #debugRows > 0 then
    ui.separator()
    ui.text('P  Driver         Live  Det@  #d Win DRS')
    for _, r in ipairs(debugRows) do
      local live = r.live and string.format('%5.2f', r.live) or '  -- '
      local detg = r.detg and string.format('%5.2f', r.detg) or '  -- '
      local drs  = r.supp and 'BLOCK' or (r.inWin and (r.auth and 'FREE*' or 'free') or 'free')
      local line = string.format('%2d %-13s %s %s %2d  %s %s',
        r.pos, (r.name or ''):sub(1, 13), live, detg, r.dets,
        r.inWin and 'IN' or '..', drs)
      ui.textColored(line, r.supp and rgbm(1, 0.55, 0.45, 1)
                     or (r.inWin and r.auth and rgbm(0.6, 1, 0.6, 1) or rgbm(0.8, 0.8, 0.8, 1)))
    end
    ui.text('Live = gap NOW; Det@ = gap at detection line (upstream). They differ -> that is normal.')
    ui.text('DRS decision uses Det@, NOT Live. FREE* = authorised & in zone.')
  end
end
