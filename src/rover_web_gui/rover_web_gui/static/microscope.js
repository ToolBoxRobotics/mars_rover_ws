// Standalone microscope tab: same "/ws" connection as the main
// dashboard, but only sends "microscope" commands and only renders
// the microscope's own telemetry slice.

let ws = null;

function connect() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${window.location.host}/ws`);
  ws.onclose = () => setTimeout(connect, 1500);
  ws.onerror = () => ws.close();
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "telemetry" && msg.data.microscope) {
      const m = msg.data.microscope;
      document.getElementById("tel-focus").textContent = m.focus_position_steps;
      document.getElementById("tel-led").textContent = m.led_pwm;
      document.getElementById("tel-cover").textContent = m.cover_open ? "OPEN" : "CLOSED";
      document.getElementById("tel-driver").textContent = m.driver_enabled ? "ENABLED" : "DISABLED";
    }
  };
}

function send(data) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "microscope", data }));
  }
}

document.addEventListener("DOMContentLoaded", () => {
  const focusSlider = document.getElementById("scope-focus");
  const ledSlider = document.getElementById("scope-led");
  const coverBtn = document.getElementById("scope-cover");
  const driverBtn = document.getElementById("scope-driver");
  let coverOpen = false;
  // Starts false to match the firmware's own boot state - it starts
  // disabled until a command explicitly enables it (see
  // microscope_uno4.ino's setup()).
  let driverEnabled = false;

  function apply() {
    send({
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
    document.getElementById("scope-led-out").textContent = ledSlider.value;
    apply();
  });
  coverBtn.addEventListener("click", () => {
    coverOpen = !coverOpen;
    coverBtn.textContent = coverOpen ? "CLOSE COVER" : "OPEN COVER";
    coverBtn.classList.toggle("toggled", coverOpen);
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
    const el = document.getElementById("scope-action-status");
    el.textContent = result.message;
    el.style.color = result.success ? "var(--cyan-live)" : "var(--rust)";
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
    const el = document.getElementById("scope-action-status");
    el.textContent = result.message;
    el.style.color = result.success ? "var(--cyan-live)" : "var(--rust)";
  });

  connect();
});
