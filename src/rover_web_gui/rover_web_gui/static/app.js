// Main ground-control dashboard. Talks to the FastAPI backend over a
// single WebSocket ("/ws"): telemetry flows down as
// {type:"telemetry", data:{...}}, and this page sends control messages
// up as {type:"drive"|"arm"|"mast"|"microscope", data:{...}}.

const BOARD_NAMES = [
  ["base_mega1", "BASE / MEGA #1"],
  ["arm_mega2", "ARM / MEGA #2"],
  ["mast_uno3", "MAST / UNO #3"],
  ["microscope_uno4", "SCOPE / UNO #4"],
  ["antenna_uno5", "ANTENNA / UNO #5"],
  ["power_uno6", "POWER / UNO #6"],
];

let ws = null;
let latestTelemetry = null;
let currentMode = "drive";

// Arm jog state held client-side, mirroring the joint targets we last sent.
const armTargets = [0, 0, 0, 0, 0];

function connect() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${window.location.host}/ws`);

  ws.onopen = () => setLinkStatus(true);
  ws.onclose = () => {
    setLinkStatus(false);
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "telemetry") {
      latestTelemetry = msg.data;
      renderTelemetry(latestTelemetry);
    }
  };
}

function send(type, data) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type, data }));
  }
}

function setLinkStatus(live) {
  const el = document.getElementById("ws-status");
  el.textContent = live ? "\u25CF LINK LIVE" : "\u25CF LINK DOWN";
  el.classList.toggle("live", live);
}

// -------------------------------------------------------------- clock ---
function tickClock() {
  document.getElementById("clock").textContent = new Date().toISOString().substr(11, 8) + " UTC";
}
setInterval(tickClock, 1000);
tickClock();

// --------------------------------------------------------- mode switch ---
function setupModeSwitch() {
  const switchEl = document.getElementById("mode-switch");
  const buttons = Array.from(switchEl.querySelectorAll("button"));
  const highlight = switchEl.querySelector(".mode-switch-highlight");

  function moveHighlight(btn) {
    highlight.style.left = btn.offsetLeft + "px";
    highlight.style.width = btn.offsetWidth + "px";
  }

  function activate(mode) {
    currentMode = mode;
    buttons.forEach((b) => b.classList.toggle("active", b.dataset.mode === mode));
    document.querySelectorAll(".mode-panel").forEach((p) => {
      p.hidden = p.dataset.modePanel !== mode;
    });
    moveHighlight(buttons.find((b) => b.dataset.mode === mode));
  }

  buttons.forEach((b) => b.addEventListener("click", () => activate(b.dataset.mode)));
  window.addEventListener("resize", () => moveHighlight(buttons.find((b) => b.dataset.mode === currentMode)));
  activate("drive");
}

// -------------------------------------------------------------- drive ---
const DRIVE_GEOMETRY_NAMES = ["ACKERMANN", "POINT-TURN", "STOP"];
let driveGeometryMode = 0; // mirrors rover_msgs/DriveMode constants

// Fallback defaults, overwritten by /api/config on load - kept so the
// joystick is usable immediately rather than blocking on the fetch,
// and matching rover_teleop/config/drive_sensitivity.yaml's own
// defaults so behavior is identical either way. transport_head_*
// mirror rover_mast/config/mast_topology.yaml's own defaults for the
// same reason.
let driveConfig = {
  max_linear_mps: 0.65,
  max_angular_radps: 1.5,
  deadzone: 0.12,
  transport_head_yaw_deg: 0.0,
  transport_head_pitch_deg: 0.0,
  min_azimuth_deg: 15.0,
  max_azimuth_deg: 285.0,
  min_elevation_deg: 0.0,
  max_elevation_deg: 180.0,
};

async function fetchDriveConfig() {
  try {
    const res = await fetch("/api/config");
    if (res.ok) driveConfig = await res.json();
  } catch (e) {
    // Network hiccup or bridge not ready yet - keep the fallback
    // defaults above; not fatal, just means the on-screen joystick
    // might not exactly match the backend's real config until reload.
  }
}

function applyDeadzone(value, threshold) {
  return Math.abs(value) < threshold ? 0.0 : value;
}

function updateDriveAxisLabels() {
  const label1 = document.getElementById("drive-axis-1-label");
  const label2 = document.getElementById("drive-axis-2-label");
  if (driveGeometryMode === 1) { // POINT_TURN
    label1.style.display = "none";
    label2.firstChild.textContent = "ANGULAR Z ";
    label2.style.display = "";
  } else if (driveGeometryMode === 2) { // STOP
    label1.style.display = "none";
    label2.style.display = "none";
  } else { // ACKERMANN
    label1.style.display = "";
    label1.firstChild.textContent = "LINEAR X ";
    label2.firstChild.textContent = "ANGULAR Z ";
    label2.style.display = "";
  }
}

function setupDrivePanel() {
  fetchDriveConfig();

  const geometryButtons = Array.from(document.querySelectorAll(".geometry-btn"));
  const joystickBase = document.getElementById("joystick-base");
  const joystickHandle = document.getElementById("joystick-handle");

  function sendDriveCommand(a1, a2) {
    document.getElementById("drive-linear-out").textContent = (driveGeometryMode === 1 ? "n/a" : a1.toFixed(2) + " m/s");
    document.getElementById("drive-angular-out").textContent = a2.toFixed(2) + " rad/s";

    if (driveGeometryMode === 2) {
      return; // STOP: nothing to send, rover_base ignores Twist entirely in this mode
    } else if (driveGeometryMode === 1) {
      send("point_turn", { angular_z: a2 });
    } else {
      send("drive", { linear_x: a1, angular_z: a2 });
    }
  }

  function resetJoystick() {
    dragging = false;
    joystickBase.classList.remove("dragging");
    joystickHandle.style.transform = "translate(0px, 0px)";
  }

  geometryButtons.forEach((btn) => {
    btn.addEventListener("click", () => {
      driveGeometryMode = parseInt(btn.dataset.geometry, 10);
      geometryButtons.forEach((b) => b.classList.toggle("active", b === btn));
      document.getElementById("drive-geometry-out").textContent = DRIVE_GEOMETRY_NAMES[driveGeometryMode];
      updateDriveAxisLabels();
      joystickBase.classList.toggle("disabled", driveGeometryMode === 2);
      resetJoystick();
      sendDriveCommand(0, 0);
      send("drive_mode", { mode: driveGeometryMode });
    });
  });

  // Virtual joystick: proportional drag control mirroring the physical
  // controller's own stick -> deadzone -> scale pipeline (see
  // rover_teleop.joy_mapping.compute_drive_twist / compute_point_turn_rate)
  // so the two control surfaces feel the same, not just command the
  // same topics.
  let dragging = false;
  let centerPoint = { x: 0, y: 0 };
  let maxDragPx = 0;
  let lastSendTime = 0;
  const SEND_INTERVAL_MS = 50; // ~20Hz, matching typical joy_node publish rates

  function pointFromEvent(e) {
    if (e.touches && e.touches.length > 0) return { x: e.touches[0].clientX, y: e.touches[0].clientY };
    return { x: e.clientX, y: e.clientY };
  }

  function applyJoystickInput(normX, normY) {
    const x = applyDeadzone(normX, driveConfig.deadzone);
    const y = applyDeadzone(normY, driveConfig.deadzone);
    // Positive angular_z means turn LEFT (rover_base/kinematics.py) -
    // dragging right should turn the rover right, so x is negated,
    // matching the intuitive "push right = turn right" a steering
    // wheel or RC transmitter gives, independent of whatever raw sign
    // convention the physical gamepad's own driver happens to report.
    if (driveGeometryMode === 1) { // POINT_TURN: left/right only
      sendDriveCommand(0, -x * driveConfig.max_angular_radps);
    } else { // ACKERMANN
      sendDriveCommand(y * driveConfig.max_linear_mps, -x * driveConfig.max_angular_radps);
    }
  }

  function onDragStart(e) {
    if (driveGeometryMode === 2) return; // STOP: joystick disabled
    e.preventDefault();
    dragging = true;
    joystickBase.classList.add("dragging");
    const rect = joystickBase.getBoundingClientRect();
    centerPoint = { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2 };
    maxDragPx = rect.width / 2 - joystickHandle.offsetWidth / 2;
    onDragMove(e);
  }

  function onDragMove(e) {
    if (!dragging) return;
    e.preventDefault();
    const p = pointFromEvent(e);
    let dx = p.x - centerPoint.x;
    let dy = p.y - centerPoint.y;
    const dist = Math.hypot(dx, dy);
    if (dist > maxDragPx && maxDragPx > 0) {
      dx = (dx / dist) * maxDragPx;
      dy = (dy / dist) * maxDragPx;
    }
    joystickHandle.style.transform = `translate(${dx}px, ${dy}px)`;

    const normX = maxDragPx > 0 ? dx / maxDragPx : 0;
    const normY = maxDragPx > 0 ? -dy / maxDragPx : 0; // screen Y grows downward; "up" should mean forward

    const now = performance.now();
    if (now - lastSendTime < SEND_INTERVAL_MS) return;
    lastSendTime = now;
    applyJoystickInput(normX, normY);
  }

  function onDragEnd() {
    if (!dragging) return;
    resetJoystick();
    sendDriveCommand(0, 0);
  }

  joystickBase.addEventListener("mousedown", onDragStart);
  window.addEventListener("mousemove", onDragMove);
  window.addEventListener("mouseup", onDragEnd);
  joystickBase.addEventListener("touchstart", onDragStart, { passive: false });
  window.addEventListener("touchmove", onDragMove, { passive: false });
  window.addEventListener("touchend", onDragEnd);

  updateDriveAxisLabels();
}

// ---------------------------------------------------------------- arm ---
function setupArmPanel() {
  const sliders = Array.from(document.querySelectorAll("[data-joint-index]"));
  sliders.forEach((slider) => {
    const idx = parseInt(slider.dataset.jointIndex, 10);
    const out = document.getElementById(`arm-joint-${idx}-out`);
    slider.addEventListener("input", () => {
      armTargets[idx] = parseInt(slider.value, 10);
      out.textContent = armTargets[idx];
      send("arm", { joint_target_steps: armTargets, enable: true });
    });
  });

  document.getElementById("arm-disable").addEventListener("click", () => {
    send("arm", { joint_target_steps: armTargets, enable: false });
  });
  document.getElementById("arm-enable").addEventListener("click", () => {
    send("arm", { joint_target_steps: armTargets, enable: true });
  });

  for (let i = 0; i < 5; i++) {
    document.getElementById(`arm-cal-${i}`).addEventListener("click", async () => {
      const res = await fetch(`/api/arm/home/${i}`, { method: "POST" });
      const result = await res.json();
      flashStatus("arm-action-status", result.message, result.accepted);
    });
  }

  document.getElementById("arm-cal-all").addEventListener("click", async () => {
    const res = await fetch("/api/arm/home/all", { method: "POST" });
    const result = await res.json();
    flashStatus("arm-action-status", result.message, result.accepted);
  });

  // The three preset buttons intentionally do NOT try to sync the
  // sliders afterward the way the old, single "return home" button
  // used to (it hardcoded a jump to all-zero, since that's what it
  // always sent). These presets are firmware-owned constants now
  // (arm_mega2.ino's own kInitialPoseSteps/kTransportPoseSteps/
  // kServicePoseSteps) - the web GUI genuinely doesn't know their
  // actual values, only that a request was accepted, so guessing a
  // slider position here would be wrong the moment those constants
  // become real, non-zero, bench-calibrated poses. The telemetry
  // panel's own live joint positions are the honest source of truth
  // for where the arm actually ends up; the sliders just reflect
  // whatever the operator's own last direct input was, which a preset
  // move doesn't change.
  async function requestPreset(endpoint) {
    const res = await fetch(endpoint, { method: "POST" });
    const result = await res.json();
    flashStatus("arm-action-status", result.message, result.accepted);
  }
  document.getElementById("arm-preset-initial").addEventListener("click", () => requestPreset("/api/arm/preset/initial"));
  document.getElementById("arm-preset-transport").addEventListener("click", () => requestPreset("/api/arm/preset/transport"));
  document.getElementById("arm-preset-service").addEventListener("click", () => requestPreset("/api/arm/preset/service"));

  // Two explicit buttons, not a single toggle that relabels itself -
  // same reasoning as every other binary-state pair in this project
  // (mast's ERECT/STOW, the microscope's OPEN/CLOSE COVER). The arm's
  // actual e-stop state is shown via estopStatusEl below, driven by
  // the telemetry panel's own estop_active field - not tracked
  // locally here, so a page reload or another operator's own action
  // stays in sync automatically rather than needing this tab to have
  // been the one that triggered it.
  document.getElementById("arm-estop-engage").addEventListener("click", async () => {
    const res = await fetch("/api/arm/estop/engage", { method: "POST" });
    const result = await res.json();
    flashStatus("arm-estop-status", result.message, result.accepted);
  });
  document.getElementById("arm-estop-clear").addEventListener("click", async () => {
    const res = await fetch("/api/arm/estop/clear", { method: "POST" });
    const result = await res.json();
    flashStatus("arm-estop-status", result.message, result.accepted);
  });
}

// --------------------------------------------------------------- mast ---
function setupMastPanel() {
  const yawSlider = document.getElementById("mast-yaw");
  const pitchSlider = document.getElementById("mast-pitch");
  const driverBtn = document.getElementById("mast-driver");
  // Starts false to match the firmware's own eventual resting state -
  // homing needs the drivers energized, and mast_uno3.ino keeps them
  // that way regardless of this value until its own automatic
  // post-calibration sequence finishes and disables them - see
  // mast_protocol.py for the full reasoning on why this can't just be
  // applied unconditionally the way the arm's enable field is.
  let driverEnabled = false;

  function apply() {
    const yawDecideg = parseInt(yawSlider.value, 10);
    const pitchDecideg = parseInt(pitchSlider.value, 10);
    document.getElementById("mast-yaw-out").textContent = (yawDecideg / 10).toFixed(1) + "\u00B0";
    document.getElementById("mast-pitch-out").textContent = (pitchDecideg / 10).toFixed(1) + "\u00B0";
    send("mast", {
      head_yaw_decideg: yawDecideg,
      head_pitch_decideg: pitchDecideg,
      lift_mode: 0,
      driver_enable: driverEnabled,
    });
  }
  yawSlider.addEventListener("input", apply);
  pitchSlider.addEventListener("input", apply);

  driverBtn.addEventListener("click", () => {
    driverEnabled = !driverEnabled;
    driverBtn.textContent = driverEnabled ? "OPEN DRIVER (DISABLE)" : "CLOSE DRIVER (ENABLE)";
    driverBtn.classList.toggle("toggled", driverEnabled);
    apply();
  });

  document.getElementById("mast-erect").addEventListener("click", () => {
    send("mast", {
      head_yaw_decideg: parseInt(yawSlider.value, 10),
      head_pitch_decideg: parseInt(pitchSlider.value, 10),
      lift_mode: 1,
      driver_enable: driverEnabled,
    });
  });
  document.getElementById("mast-stow").addEventListener("click", () => {
    send("mast", {
      head_yaw_decideg: parseInt(yawSlider.value, 10),
      head_pitch_decideg: parseInt(pitchSlider.value, 10),
      lift_mode: -1,
      driver_enable: driverEnabled,
    });
  });

  // Both buttons below only touch yaw/pitch - deliberately not the
  // lift, so sequencing (e.g. re-center the head before stowing, so
  // it isn't wherever it last happened to be pointed while the lift
  // lowers) is the operator's own choice via the Stow button above,
  // not baked into either of these.
  function goTo(yawDeg, pitchDeg) {
    const yawDecideg = Math.round(yawDeg * 10);
    const pitchDecideg = Math.round(pitchDeg * 10);
    yawSlider.value = yawDecideg;
    pitchSlider.value = pitchDecideg;
    document.getElementById("mast-yaw-out").textContent = yawDeg.toFixed(1) + "\u00B0";
    document.getElementById("mast-pitch-out").textContent = pitchDeg.toFixed(1) + "\u00B0";
    send("mast", {
      head_yaw_decideg: yawDecideg,
      head_pitch_decideg: pitchDecideg,
      lift_mode: 0,
      driver_enable: driverEnabled,
    });
    flashStatus("mast-action-status", `sent to ${yawDeg.toFixed(1)}\u00B0 / ${pitchDeg.toFixed(1)}\u00B0`, true);
  }

  document.getElementById("mast-return-home").addEventListener("click", () => goTo(0, 0));
  document.getElementById("mast-transport-position").addEventListener("click", () => {
    goTo(driveConfig.transport_head_yaw_deg, driveConfig.transport_head_pitch_deg);
  });
}

// --------------------------------------------------------- microscope ---
function setupMicroscopePanel() {
  const focusSlider = document.getElementById("scope-focus");
  const ledSlider = document.getElementById("scope-led");
  const coverOpenBtn = document.getElementById("scope-cover-open");
  const coverCloseBtn = document.getElementById("scope-cover-close");
  const ledOnBtn = document.getElementById("scope-led-on");
  const ledOffBtn = document.getElementById("scope-led-off");
  const driverBtn = document.getElementById("scope-driver");
  let coverOpen = false;
  // Starts false to match the firmware's own boot state - it starts
  // disabled until a command explicitly enables it (see
  // microscope_uno4.ino's setup()).
  let driverEnabled = false;

  function apply() {
    send("microscope", {
      focus_target_steps: parseInt(focusSlider.value, 10),
      led_pwm: parseInt(ledSlider.value, 10),
      cover_open: coverOpen,
      driver_enable: driverEnabled,
    });
  }

  focusSlider.addEventListener("input", () => {
    document.getElementById("scope-focus-out").textContent = focusSlider.value;
    apply();
  });
  ledSlider.addEventListener("input", () => {
    document.getElementById("scope-led-out").textContent = formatLedVoltage(parseInt(ledSlider.value, 10));
    apply();
  });
  // Two explicit buttons, not a single toggle - matching this
  // project's own established pattern for a binary physical state
  // (see mast's own ERECT/STOW buttons for the precedent this
  // follows). The cover's actual current state is shown in the
  // telemetry panel's own COVER field, not tracked by which of these
  // two buttons was clicked last.
  coverOpenBtn.addEventListener("click", () => {
    coverOpen = true;
    apply();
  });
  coverCloseBtn.addEventListener("click", () => {
    coverOpen = false;
    apply();
  });
  // Same two-explicit-buttons reasoning as the cover above - "on"
  // sets the slider to full brightness, "off" to zero, and both keep
  // the slider itself in sync so it always reflects whatever was
  // actually just sent, whether that came from dragging it directly
  // or from one of these two quick-action buttons.
  ledOnBtn.addEventListener("click", () => {
    ledSlider.value = 255;
    document.getElementById("scope-led-out").textContent = formatLedVoltage(255);
    apply();
  });
  ledOffBtn.addEventListener("click", () => {
    ledSlider.value = 0;
    document.getElementById("scope-led-out").textContent = formatLedVoltage(0);
    apply();
  });
  // "Close driver" energizes it (enable) - "open driver" de-energizes
  // it (disable), matching the electrical open-circuit/closed-circuit
  // convention: a closed circuit is the one current actually flows
  // through. Labeled with the action in parentheses too since that
  // mapping isn't the first thing everyone reaches for.
  driverBtn.addEventListener("click", () => {
    driverEnabled = !driverEnabled;
    driverBtn.textContent = driverEnabled ? "OPEN DRIVER (DISABLE)" : "CLOSE DRIVER (ENABLE)";
    driverBtn.classList.toggle("toggled", driverEnabled);
    apply();
  });

  // Focus/zoom presets - 3 remembered positions, purely client-side
  // (no firmware or protocol involvement - see microscope_uno4.ino's
  // header comment). "Record" saves the current slider value into a
  // slot; "go to" moves there. Lost on page reload by design - this
  // is a convenience for the current session, not a persistent
  // calibration reference.
  const presets = [null, null, null];
  for (let i = 0; i < 3; i++) {
    const statusEl = document.getElementById(`scope-preset-status-${i}`);
    document.getElementById(`scope-preset-record-${i}`).addEventListener("click", () => {
      presets[i] = parseInt(focusSlider.value, 10);
      statusEl.textContent = presets[i];
    });
    document.getElementById(`scope-preset-goto-${i}`).addEventListener("click", () => {
      if (presets[i] === null) return;
      focusSlider.value = presets[i];
      document.getElementById("scope-focus-out").textContent = presets[i];
      apply();
    });
  }

  document.getElementById("scope-snapshot").addEventListener("click", async () => {
    const res = await fetch("/api/microscope/snapshot", { method: "POST" });
    const result = await res.json();
    flashStatus("scope-action-status", result.message, result.success);
  });

  const recordBtn = document.getElementById("scope-record");
  let recording = false;
  recordBtn.addEventListener("click", async () => {
    const res = await fetch("/api/microscope/recording/toggle", { method: "POST" });
    const result = await res.json();
    if (result.success) {
      recording = !recording;
      recordBtn.textContent = recording ? "STOP RECORDING" : "START RECORDING";
      recordBtn.classList.toggle("toggled", recording);
    }
    flashStatus("scope-action-status", result.message, result.success);
  });
}

// ------------------------------------------------------------ antenna ---
function setupAntennaPanel() {
  const azimuthSlider = document.getElementById("antenna-azimuth");
  const elevationSlider = document.getElementById("antenna-elevation");
  const driverBtn = document.getElementById("antenna-driver");
  // Starts false to match the firmware's own boot state - it starts
  // enabled internally for homing's own sake, but antenna_uno5.ino
  // ignores driver_enable entirely until homed is true, so nothing
  // reaches the motors until this is explicitly turned on anyway.
  let driverEnabled = false;

  // Slider min/max come from driveConfig (populated by /api/config,
  // see fetchDriveConfig()) rather than the HTML's own hardcoded
  // min/max attributes, which are just a reasonable-looking fallback
  // for before that fetch resolves - matching the real operational
  // range (rover_antenna/config/antenna_topology.yaml) keeps the
  // slider from silently drifting out of sync with what the firmware
  // actually enforces.
  azimuthSlider.min = Math.round(driveConfig.min_azimuth_deg * 10);
  azimuthSlider.max = Math.round(driveConfig.max_azimuth_deg * 10);
  azimuthSlider.value = azimuthSlider.min;
  elevationSlider.min = Math.round(driveConfig.min_elevation_deg * 10);
  elevationSlider.max = Math.round(driveConfig.max_elevation_deg * 10);

  function apply() {
    const azimuthDecideg = parseInt(azimuthSlider.value, 10);
    const elevationDecideg = parseInt(elevationSlider.value, 10);
    document.getElementById("antenna-azimuth-out").textContent = (azimuthDecideg / 10).toFixed(1) + "\u00B0";
    document.getElementById("antenna-elevation-out").textContent = (elevationDecideg / 10).toFixed(1) + "\u00B0";
    send("antenna", {
      azimuth_decideg: azimuthDecideg,
      elevation_decideg: elevationDecideg,
      driver_enable: driverEnabled,
    });
  }
  azimuthSlider.addEventListener("input", apply);
  elevationSlider.addEventListener("input", apply);

  driverBtn.addEventListener("click", () => {
    driverEnabled = !driverEnabled;
    driverBtn.textContent = driverEnabled ? "OPEN DRIVER (DISABLE)" : "CLOSE DRIVER (ENABLE)";
    driverBtn.classList.toggle("toggled", driverEnabled);
    apply();
  });
}

function flashStatus(elId, message, ok) {
  const el = document.getElementById(elId);
  el.textContent = message;
  el.style.color = ok ? "var(--cyan-live)" : "var(--rust)";
}

// ---------------------------------------------------------- telemetry ---
// FIXED: this function existed, correctly written, but was never
// actually called anywhere - every lamp rendered off a simpler
// connected-only check instead, so a board silently producing
// nothing but checksum errors (e.g. /dev/rover/X resolved to the
// wrong physical device - a real risk anywhere two boards share a
// VID:PID, see the udev rules' own collision notes) would show
// exactly the same plain green as a board that's actually working,
// with no visible distinction at all. Now wired into the actual
// rendering loop below.
function boardStatusPillClass(status) {
  if (!status) return "fault";
  if (!status.connected) return "fault";
  if (status.checksum_error_count > 0) return "warn";
  return "ok";
}

function formatVoltage(mv) {
  // No color-coded low-battery warning here deliberately - that would
  // need a real threshold, which depends on the actual battery
  // chemistry/cell count this rover ends up using (LiPo cell-count,
  // lead-acid, etc.), not something to guess at. A clean, easy follow-up
  // once that's known; a wrong guess here is worse than no warning.
  if (typeof mv !== "number") return "?";
  return (mv / 1000).toFixed(2) + "V";
}

function formatTemperature(deciC) {
  // -9999 is the firmware's "sensor didn't respond on its most recent
  // read" sentinel (see base_mega1.ino's own comment on the constant,
  // and DallasTemperature's DEVICE_DISCONNECTED_C that produces it -
  // checked fresh every cycle, not just once at boot) - shown as a
  // plain, unmissable "N/A" rather than a bogus -999.9C, which could
  // otherwise be mistaken for a real (if extreme) reading.
  if (typeof deciC !== "number") return "?";
  if (deciC === -9999) return "N/A";
  return (deciC / 10).toFixed(1) + "\u00B0C";
}

function formatCurrent(ma) {
  if (typeof ma !== "number") return "?";
  return (ma / 1000).toFixed(2) + "A";
}

// The microscope LED is still driven by plain PWM (analogWrite(),
// 0-255) - there's no DAC or true analog output, and the wire
// protocol/firmware still send/expect that same 0-255 raw value. This
// is display-only: reformats that existing range as its volt-
// equivalent for the slider's own label, since 0-255 duty cycle on a
// 5V logic pin IS 0-5V of effective average output - the underlying
// value sent to the firmware is unchanged either way.
function formatLedVoltage(pwm) {
  return ((pwm / 255) * 5).toFixed(2) + "V";
}

function renderTelemetry(data) {
  if (typeof data.drive_mode === "number" && data.drive_mode !== driveGeometryMode) {
    driveGeometryMode = data.drive_mode;
    document.querySelectorAll(".geometry-btn").forEach((b) => {
      b.classList.toggle("active", parseInt(b.dataset.geometry, 10) === driveGeometryMode);
    });
    document.getElementById("drive-geometry-out").textContent = DRIVE_GEOMETRY_NAMES[driveGeometryMode] ?? "?";
    updateDriveAxisLabels();
    // Keep the joystick's visual state in sync even when the mode
    // changed remotely (e.g. the physical Xbox controller forced
    // STOP) - otherwise it would look draggable while silently doing
    // nothing, since rover_base already ignores Twist in STOP either way.
    const joystickBase = document.getElementById("joystick-base");
    if (joystickBase) joystickBase.classList.toggle("disabled", driveGeometryMode === 2);
  }

  const lampList = document.getElementById("board-lamps");
  lampList.innerHTML = "";
  for (const [key, label] of BOARD_NAMES) {
    const status = data.board_status[key];
    const row = document.createElement("div");
    row.className = "lamp-row";
    const pillClass = boardStatusPillClass(status);
    const metaText = status
      ? `${status.rx_frame_count} rx` + (status.checksum_error_count > 0 ? `, ${status.checksum_error_count} bad` : "")
      : "no data";
    row.innerHTML = `
      <span class="lamp ${pillClass}"></span>
      <span class="lamp-label">${label}</span>
      <span class="lamp-meta">${metaText}</span>
    `;
    lampList.appendChild(row);
  }

  const telBase = document.getElementById("telemetry-base");
  if (data.base) {
    telBase.innerHTML =
      data.base.encoder_ticks.map((t, i) => `<dt>ENC[${i}]</dt><dd>${t}</dd>`).join("") +
      `<dt>DRIVE SUPPLY</dt><dd>${formatVoltage(data.base.drive_voltage_mv)}</dd>` +
      `<dt>STEER SUPPLY</dt><dd>${formatVoltage(data.base.steering_voltage_mv)}</dd>` +
      `<dt>TEMP</dt><dd>${formatTemperature(data.base.board_temperature_decic)}</dd>` +
      `<dt>FAN</dt><dd>${data.base.fan_duty_percent > 0 ? data.base.fan_duty_percent + "%" : "OFF"}</dd>`;
  }

  const telArm = document.getElementById("telemetry-arm");
  if (data.arm) {
    const jointHomed = data.arm.joint_homed || [];
    telArm.innerHTML =
      data.arm.joint_position_steps
        .map((p, i) => `<dt>J${i + 1}</dt><dd>${p} ${jointHomed[i] ? "\u2713" : "\u2717"}</dd>`)
        .join("") +
      `<dt>HOMED</dt><dd>${data.arm.homed ? "YES" : "NO"}</dd>` +
      `<dt>E-STOP</dt><dd${data.arm.estop_active ? ' class="estop-active"' : ""}>${data.arm.estop_active ? "ACTIVE" : "clear"}</dd>` +
      `<dt>SUPPLY</dt><dd>${formatVoltage(data.arm.supply_voltage_mv)}</dd>` +
      `<dt>TEMP</dt><dd>${formatTemperature(data.arm.board_temperature_decic)}</dd>` +
      `<dt>FAN</dt><dd>${data.arm.fan_duty_percent > 0 ? data.arm.fan_duty_percent + "%" : "OFF"}</dd>`;
  }

  const telMast = document.getElementById("telemetry-mast");
  if (data.mast) {
    const liftLabels = ["UNKNOWN", "TRANSPORT", "SERVICE", "MOVING"];
    telMast.innerHTML = `
      <dt>YAW</dt><dd>${(data.mast.head_yaw_decideg / 10).toFixed(1)}\u00B0</dd>
      <dt>PITCH</dt><dd>${(data.mast.head_pitch_decideg / 10).toFixed(1)}\u00B0</dd>
      <dt>LIFT</dt><dd>${liftLabels[data.mast.lift_state] ?? "?"}</dd>
      <dt>DRIVER</dt><dd>${data.mast.driver_enabled ? "ENABLED" : "DISABLED"}</dd>
      <dt>SUPPLY</dt><dd>${formatVoltage(data.mast.supply_voltage_mv)}</dd>
      <dt>TEMP</dt><dd>${formatTemperature(data.mast.board_temperature_decic)}</dd>
      <dt>FAN</dt><dd>${data.mast.fan_duty_percent > 0 ? data.mast.fan_duty_percent + "%" : "OFF"}</dd>
    `;
  }

  const telMicroscope = document.getElementById("telemetry-microscope");
  if (data.microscope) {
    telMicroscope.innerHTML = `
      <dt>FOCUS</dt><dd>${data.microscope.focus_position_steps}</dd>
      <dt>LED</dt><dd>${formatLedVoltage(data.microscope.led_pwm)}</dd>
      <dt>COVER</dt><dd>${data.microscope.cover_open ? "OPEN" : "CLOSED"}</dd>
      <dt>DRIVER</dt><dd>${data.microscope.driver_enabled ? "ENABLED" : "DISABLED"}</dd>
      <dt>TEMP</dt><dd>${formatTemperature(data.microscope.board_temperature_decic)}</dd>
      <dt>FAN</dt><dd>${data.microscope.fan_duty_percent > 0 ? data.microscope.fan_duty_percent + "%" : "OFF"}</dd>
    `;
  }

  const telAntenna = document.getElementById("telemetry-antenna");
  if (data.antenna) {
    telAntenna.innerHTML = `
      <dt>AZIMUTH</dt><dd>${(data.antenna.azimuth_decideg / 10).toFixed(1)}\u00B0</dd>
      <dt>ELEVATION</dt><dd>${(data.antenna.elevation_decideg / 10).toFixed(1)}\u00B0</dd>
      <dt>HOMED</dt><dd>${data.antenna.homed ? "YES" : "NO"}</dd>
      <dt>DRIVER</dt><dd>${data.antenna.driver_enabled ? "ENABLED" : "DISABLED"}</dd>
      <dt>SUPPLY</dt><dd>${formatVoltage(data.antenna.supply_voltage_mv)}</dd>
      <dt>TEMP</dt><dd>${formatTemperature(data.antenna.board_temperature_decic)}</dd>
      <dt>FAN</dt><dd>${data.antenna.fan_duty_percent > 0 ? data.antenna.fan_duty_percent + "%" : "OFF"}</dd>
    `;
  }

  const telPower = document.getElementById("telemetry-power");
  if (data.power) {
    telPower.innerHTML = `
      <dt>BATTERY 1</dt><dd>${formatVoltage(data.power.battery1_voltage_mv)} / ${formatCurrent(data.power.battery1_current_ma)}</dd>
      <dt>BATTERY 2</dt><dd>${formatVoltage(data.power.battery2_voltage_mv)} / ${formatCurrent(data.power.battery2_current_ma)}</dd>
      <dt>COMPUTER TEMP</dt><dd>${formatTemperature(data.power.computer_temperature_decic)}</dd>
      <dt>FAN</dt><dd>${data.power.fan_duty_percent > 0 ? data.power.fan_duty_percent + "%" : "OFF"}</dd>
    `;
  }

  const telImu = document.getElementById("telemetry-imu");
  if (data.imu) {
    const q = data.imu.orientation;
    telImu.innerHTML = `
      <dt>QUAT X</dt><dd>${q.x.toFixed(3)}</dd>
      <dt>QUAT Y</dt><dd>${q.y.toFixed(3)}</dd>
      <dt>QUAT Z</dt><dd>${q.z.toFixed(3)}</dd>
      <dt>QUAT W</dt><dd>${q.w.toFixed(3)}</dd>
    `;
  }

  const telGps = document.getElementById("telemetry-gps");
  if (data.gps_fix) {
    telGps.innerHTML = `
      <dt>LAT</dt><dd>${data.gps_fix.latitude.toFixed(6)}</dd>
      <dt>LON</dt><dd>${data.gps_fix.longitude.toFixed(6)}</dd>
      <dt>ALT</dt><dd>${data.gps_fix.altitude.toFixed(1)} m</dd>
      <dt>FIX</dt><dd>${data.gps_fix.status >= 0 ? "YES" : "NO"}</dd>
    `;
  }
}

document.addEventListener("DOMContentLoaded", () => {
  setupModeSwitch();
  setupDrivePanel();
  setupArmPanel();
  setupMastPanel();
  setupMicroscopePanel();
  setupAntennaPanel();
  connect();
});
