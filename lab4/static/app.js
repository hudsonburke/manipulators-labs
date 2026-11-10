"use strict";

const $ = (id) => document.getElementById(id);
const token = document.querySelector('meta[name="lab-token"]').content;
const camera = $("camera");
const canvas = $("capture");
const context = canvas.getContext("2d");
let stream = null;
let session = "";
let generation = 0;

async function request(path, body, contentType = "application/json", options = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "X-Lab-Token": token, "X-Camera-Session": session, "Content-Type": contentType },
    body: body ?? "{}",
    signal: AbortSignal.timeout(5000),
    ...options,
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `Request failed (${response.status}).`);
  return result;
}

function showState(state) {
  if (!state.controller) return;
  const robot = state.controller;
  $("robot-status").textContent = `${robot.enabled ? "TRACKING ENABLED — " : "Disabled — "}${robot.message}`;
  $("actual").textContent = "Actual robot joints [rad]: " + (robot.actual_q ? robot.actual_q.map((q) => q.toFixed(3)).join(", ") : "—");
  $("calibrate").disabled = !stream || !state.tracking || robot.enabled || robot.busy;
  $("initialize").disabled = !stream || robot.busy || robot.enabled;
  $("enable").disabled = !stream || !state.tracking || !state.calibrated || !robot.initialized || robot.busy || robot.enabled;
  if (state.angles) {
    const degrees = (radians) => (radians * 180 / Math.PI).toFixed(1);
    $("angles").textContent = `Human angles [deg]: elbow ${degrees(state.angles.elbow)}, shoulder ${degrees(state.angles.shoulder)}`;
  } else {
    $("angles").textContent = "Human angles: no valid left-arm observation.";
  }
  $("targets").textContent = "Robot targets [rad]: " + (state.target_q ? state.target_q.map((q) => q.toFixed(3)).join(", ") : "Calibrate first.");
}

async function stopCamera(message = "Camera is off. Robot control disabled.") {
  generation += 1;
  if (stream) stream.getTracks().forEach((track) => track.stop());
  stream = null;
  camera.srcObject = null;
  $("start").disabled = false;
  $("stop-camera").disabled = true;
  $("calibrate").disabled = $("initialize").disabled = $("enable").disabled = true;
  $("camera-status").textContent = message;
  try { showState(await request("/api/disable")); }
  catch (error) { $("notice").textContent = `Could not confirm robot stop: ${error.message}. Check PolyScope; the stale-frame watchdog should disable tracking.`; }
}

async function processFrames(currentGeneration) {
  while (stream && currentGeneration === generation) {
    try {
      context.drawImage(camera, 0, 0, canvas.width, canvas.height);
      const frame = await new Promise((resolve) => canvas.toBlob(resolve, "image/jpeg", 0.8));
      if (!frame || currentGeneration !== generation) return;
      const started = performance.now();
      const state = await request("/api/frame", frame, "image/jpeg");
      if (currentGeneration !== generation) return;
      $("processed").src = state.image;
      $("processed").hidden = false;
      camera.hidden = true;
      $("save").disabled = false;
      $("tracking-status").textContent = `${state.message} | server ${state.processing_ms} ms; round trip ${Math.round(performance.now() - started)} ms`;
      showState(state);
      // One request in flight: capture a new frame after processing, never queue old video.
      await new Promise((resolve) => setTimeout(resolve, 100));
    } catch (error) {
      $("notice").textContent = error.message;
      await stopCamera("Frame processing stopped. Fix the error and start the camera again.");
      return;
    }
  }
}

$("start").addEventListener("click", async () => {
  $("start").disabled = true;
  $("notice").textContent = "";
  $("camera-status").textContent = "Waiting for camera permission…";
  const currentGeneration = ++generation;
  try {
    if (!navigator.mediaDevices?.getUserMedia) throw new Error("Camera access requires HTTPS or localhost. Open this app in a normal browser tab, not an embedded preview.");
    const acquired = await navigator.mediaDevices.getUserMedia({ video: { width: { ideal: 640 }, height: { ideal: 480 } }, audio: false });
    if (currentGeneration !== generation) { acquired.getTracks().forEach((track) => track.stop()); return; }
    stream = acquired;
    camera.hidden = false;
    $("processed").hidden = true;
    camera.srcObject = stream;
    await camera.play();
    const ratio = camera.videoHeight / camera.videoWidth;
    canvas.width = Math.min(camera.videoWidth, 640);
    canvas.height = Math.round(canvas.width * ratio);
    if (canvas.height > 960) throw new Error("Unsupported camera aspect ratio. Choose a standard landscape camera.");
    const state = await request("/api/session");
    session = state.session;
    stream.getVideoTracks()[0].addEventListener("ended", () => { if (stream) void stopCamera("Camera access ended. Robot control disabled."); });
    $("stop-camera").disabled = false;
    $("camera-status").textContent = "Camera active. Frames are processed in your Python server; no audio is captured.";
    showState(state);
    void processFrames(currentGeneration);
  } catch (error) {
    $("notice").textContent = `${error.name}: ${error.message}. Check browser and operating-system camera permissions; close other apps using the camera.`;
    await stopCamera("Camera could not start.");
  }
});

$("stop-camera").addEventListener("click", () => void stopCamera());
for (const action of ["calibrate", "initialize", "enable", "disable"]) {
  $(action).addEventListener("click", async () => {
    $("notice").textContent = "";
    try { showState(await request(`/api/${action}`)); }
    catch (error) { $("notice").textContent = error.message; }
  });
}
$("save").addEventListener("click", async () => {
  try { $("notice").textContent = (await request("/api/evidence", JSON.stringify({ task: $("task").value }))).message; }
  catch (error) { $("notice").textContent = error.message; }
});

let polling = false;
setInterval(async () => {
  if (polling) return;
  polling = true;
  try {
    const response = await fetch("/api/status", { headers: { "X-Lab-Token": token }, signal: AbortSignal.timeout(3000) });
    if (response.ok) showState(await response.json());
    else $("robot-status").textContent = "Status unavailable. Reload the app if authentication expired.";
  } catch {
    $("robot-status").textContent = "Server unreachable. Do not assume the robot has stopped; check PolyScope.";
  } finally { polling = false; }
}, 500);

function disableOnLeave() {
  void request("/api/disable", null, "application/json", { keepalive: true }).catch(() => {});
}
document.addEventListener("visibilitychange", () => { if (document.hidden) disableOnLeave(); });
window.addEventListener("pagehide", disableOnLeave);
