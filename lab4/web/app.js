'use strict';

(() => {
  const byId = (id) => document.getElementById(id);
  const startButton = byId('start-camera');
  const stopCameraButton = byId('stop-camera');
  const taskSelect = byId('task');
  const preview = byId('preview');
  const status = byId('status');
  const error = byId('error');
  const connection = byId('connection');
  const actions = Object.fromEntries(
    ['initialize', 'calibrate', 'enable', 'stop', 'save'].map((name) => [name, byId(name)]),
  );
  const video = document.createElement('video');
  video.muted = true;
  video.playsInline = true;
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('2d');
  const evidenceNames = new Set([
    'task1_preview.jpg', 'task2_preview.jpg', 'task3_preview.jpg', 'task3_control.csv',
  ]);
  let stream = null;
  let socket = null;
  let starting = false;
  let generation = 0;
  let taskGeneration = 0;
  let credit = false;
  let encoding = false;
  let captureTimer = null;
  let frameCallback = null;
  let lastCapture = -Infinity;
  let previewUrl = null;
  let state = null;

  const connected = () => socket !== null && socket.readyState === WebSocket.OPEN;
  const text = (value) => typeof value === 'string' ? value : '';
  const number = (value) => Number.isFinite(value) ? value.toFixed(3) : '—';
  const vector = (value) => Array.isArray(value) ? value.map(number).join(', ') : '—';

  function updateControls() {
    const online = connected();
    const robot = state?.robot || {};
    const task3 = taskSelect.value === '3';
    startButton.disabled = starting || socket !== null || stream !== null;
    stopCameraButton.disabled = !starting && socket === null && stream === null;
    actions.initialize.disabled = !online || !task3 || Boolean(robot.busy);
    actions.calibrate.disabled = !online || !task3 || !robot.initialized || Boolean(robot.busy);
    actions.enable.disabled = !online || !task3 || !robot.initialized || !state?.calibrated || Boolean(robot.busy) || Boolean(robot.enabled);
    actions.stop.disabled = !online;
    actions.save.disabled = !online;
  }

  function renderState(next) {
    if (next.task !== Number(taskSelect.value)) return;
    state = next;
    const robot = next.robot || {};
    const mode = robot.busy ? 'HOME / action in progress' : robot.enabled
      ? (next.tracking ? 'ENABLED — tracking' : 'ENABLED — PAUSED: waiting for fresh tracking')
      : 'DISARMED';
    byId('motion-mode').textContent = `Task ${next.task} | ${mode}`;
    status.textContent = [
      `Task ${next.task} | ${mode}`,
      `Tracking: ${next.tracking ? 'pose detected' : 'missing / paused'}`,
      `Initialized: ${Boolean(robot.initialized)} | Calibrated: ${Boolean(next.calibrated)} | Enabled: ${Boolean(robot.enabled)} | Busy: ${Boolean(robot.busy)}`,
      `Elbow (rad): ${number(next.angles?.elbow)} | Shoulder (rad): ${number(next.angles?.shoulder)}`,
      `Target (rad): ${vector(next.target)}`,
      `Actual joints (rad): ${vector(robot.actual_q)}`,
      `Command joints (rad): ${vector(robot.command_q)}`,
      `Processing: ${number(next.processing_ms)} ms`,
      text(robot.message), text(next.feedback), text(next.message),
    ].filter(Boolean).join('\n');
    updateControls();
  }

  function sendAction(action) {
    if (!connected() || actions[action]?.disabled) return;
    error.textContent = '';
    socket.send(JSON.stringify({ type: 'action', action }));
  }

  function stopCamera(message = 'Disconnected — camera off. Control stop requested.') {
    generation += 1;
    starting = false;
    clearTimeout(captureTimer);
    captureTimer = null;
    if (frameCallback !== null) video.cancelVideoFrameCallback(frameCallback);
    frameCallback = null;
    credit = false;
    encoding = false;
    const oldSocket = socket;
    socket = null;
    if (oldSocket) {
      if (oldSocket.readyState === WebSocket.OPEN) {
        oldSocket.send(JSON.stringify({ type: 'action', action: 'camera_stop' }));
      }
      oldSocket.close(1000, 'Camera stopped');
    }
    if (stream) stream.getTracks().forEach((track) => track.stop());
    stream = null;
    video.pause();
    video.srcObject = null;
    state = null;
    preview.hidden = true;
    byId('preview-placeholder').hidden = false;
    if (previewUrl) URL.revokeObjectURL(previewUrl);
    previewUrl = null;
    preview.removeAttribute('src');
    connection.textContent = message;
    status.textContent = 'Disconnected. Camera stopped; server cleanup disarms control. If the connection was lost, verify URSim has stopped.';
    byId('motion-mode').textContent = 'Camera off — stop requested';
    updateControls();
  }

  function fail(message) {
    stopCamera();
    error.textContent = message;
  }

  function cameraError(cause) {
    switch (cause?.name) {
      case 'NotAllowedError':
      case 'SecurityError':
        return 'Camera permission was denied. Open the private HTTPS forwarded port in its own top-level browser tab (not the VS Code preview), allow camera access in site settings, then press Start camera.';
      case 'NotFoundError':
        return 'No camera was found. Connect a camera and check your browser camera settings, then press Start camera.';
      case 'NotReadableError':
        return 'The camera could not be opened. Close other apps using it, check OS camera permissions, then press Start camera.';
      default:
        return `Could not start the camera: ${text(cause?.message) || 'unknown camera error'}. Check camera permissions in a top-level private HTTPS tab, then press Start camera.`;
    }
  }

  function scheduleCapture() {
    if (!credit || encoding || !connected() || !stream || captureTimer !== null || frameCallback !== null) return;
    captureTimer = setTimeout(() => {
      captureTimer = null;
      frameCallback = video.requestVideoFrameCallback(() => {
        frameCallback = null;
        capture();
      });
    }, Math.max(0, 100 - (performance.now() - lastCapture)));
  }

  function capture() {
    if (!credit || encoding || !connected() || !stream) return;
    if (stream.getVideoTracks().some((track) => track.muted || track.readyState !== 'live')) {
      scheduleCapture();
      return;
    }
    if (video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA || !video.videoWidth || !video.videoHeight) {
      captureTimer = setTimeout(() => { captureTimer = null; scheduleCapture(); }, 50);
      return;
    }
    const owner = generation;
    const task = taskGeneration;
    const currentSocket = socket;
    const scale = Math.min(1, 640 / video.videoWidth, 480 / video.videoHeight);
    canvas.width = Math.max(1, Math.floor(video.videoWidth * scale));
    canvas.height = Math.max(1, Math.floor(video.videoHeight * scale));
    encoding = true;
    lastCapture = performance.now();
    try {
      context.drawImage(video, 0, 0, canvas.width, canvas.height);
      canvas.toBlob((blob) => {
        if (owner !== generation || currentSocket !== socket) return;
        encoding = false;
        if (task !== taskGeneration) { scheduleCapture(); return; }
        if (!blob || blob.size > 256 * 1024) {
          fail('Camera JPEG exceeded the safe frame limit or could not be encoded. Stop other camera applications and press Start camera to retry.');
          return;
        }
        if (!connected() || !credit) return;
        if (socket.bufferedAmount !== 0) { scheduleCapture(); return; }
        credit = false;
        socket.send(blob);
      }, 'image/jpeg', 0.7);
    } catch (cause) {
      fail(`Camera capture failed: ${text(cause?.message)}. Press Start camera to retry.`);
    }
  }

  function showDownloads(files) {
    const downloads = byId('downloads');
    downloads.replaceChildren();
    if (!Array.isArray(files)) return;
    for (const file of files) {
      if (!file || !evidenceNames.has(file.name) || typeof file.url !== 'string') continue;
      const url = new URL(file.url, window.location.href);
      if (url.origin !== window.location.origin || url.pathname !== `/outputs/${file.name}` || url.search || url.hash) continue;
      const link = document.createElement('a');
      link.href = url.href;
      link.download = file.name;
      link.textContent = `Download ${file.name}`;
      downloads.append(link);
    }
  }

  function receive(event, currentSocket) {
    if (socket !== currentSocket) return;
    if (event.data instanceof Blob) {
      const oldUrl = previewUrl;
      previewUrl = URL.createObjectURL(event.data);
      preview.src = previewUrl;
      preview.hidden = false;
      byId('preview-placeholder').hidden = true;
      if (oldUrl) URL.revokeObjectURL(oldUrl);
      return;
    }
    try {
      const message = JSON.parse(event.data);
      switch (message.type) {
        case 'ready':
          credit = true;
          scheduleCapture();
          break;
        case 'state':
          renderState(message);
          break;
        case 'error':
          error.textContent = text(message.message) || 'The server rejected this action. Check the live state before trying again.';
          break;
        case 'saved':
          showDownloads(message.files);
          break;
        default:
          fail('Unexpected server message. Camera stopped; reload the page to use matching browser and server versions.');
      }
    } catch {
      fail('Invalid server response. Camera stopped; check the Codespace server log before restarting.');
    }
  }

  async function startCamera() {
    if (starting || socket || stream) return;
    error.textContent = '';
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {
      error.textContent = 'Camera access requires a secure browser context. Open the private HTTPS forwarded port in a full browser tab, not an HTTP link or VS Code preview. Keep port visibility Private.';
      return;
    }
    if (window.top !== window.self) {
      error.textContent = 'Open this private HTTPS forwarded port in its own browser tab. Camera control is not supported inside the VS Code preview iframe.';
      return;
    }
    if (!video.requestVideoFrameCallback || !video.cancelVideoFrameCallback) {
      error.textContent = 'Update your desktop browser: fresh video-frame callbacks are required so a frozen camera cannot keep supplying old poses.';
      return;
    }
    starting = true;
    const owner = ++generation;
    connection.textContent = 'Requesting camera permission…';
    updateControls();
    try {
      const acquired = await navigator.mediaDevices.getUserMedia({
        video: { width: { ideal: 640, max: 640 }, height: { ideal: 480, max: 480 }, frameRate: { ideal: 10, max: 10 } },
        audio: false,
      });
      if (owner !== generation) { acquired.getTracks().forEach((track) => track.stop()); return; }
      stream = acquired;
      for (const track of stream.getTracks()) {
        track.addEventListener('ended', () => {
          if (owner === generation) fail('Camera access ended. Check camera permissions and press Start camera to reconnect; verify URSim has stopped.');
        });
      }
      video.srcObject = stream;
      await video.play();
      if (owner !== generation) return;
      const url = new URL('/ws', window.location.href);
      url.protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const currentSocket = new WebSocket(url);
      socket = currentSocket;
      currentSocket.binaryType = 'blob';
      connection.textContent = 'Camera ready — connecting to the private Codespace…';
      currentSocket.addEventListener('open', () => {
        if (socket !== currentSocket) return;
        starting = false;
        state = null;
        connection.textContent = 'Connected — camera on. A new session starts disarmed.';
        currentSocket.send(JSON.stringify({ type: 'task', task: Number(taskSelect.value) }));
        updateControls();
      });
      currentSocket.addEventListener('message', (event) => receive(event, currentSocket));
      currentSocket.addEventListener('error', () => {
        if (socket === currentSocket) fail('Connection failed. Keep the forwarded port Private, open its HTTPS URL in a top-level tab, and check that web_lab.py is running. Press Start camera to retry.');
      });
      currentSocket.addEventListener('close', () => {
        if (socket !== currentSocket) return;
        const detail = error.textContent;
        stopCamera();
        error.textContent = detail || 'The server disconnected. Camera tracks are stopped; server cleanup disarms control. Verify URSim has stopped, check the server log, and close any other controlling tab before reconnecting.';
      });
    } catch (cause) {
      if (owner === generation) fail(cameraError(cause));
    }
  }

  startButton.addEventListener('click', startCamera);
  stopCameraButton.addEventListener('click', () => stopCamera());
  for (const [action, button] of Object.entries(actions)) {
    button.addEventListener('click', () => sendAction(action));
  }
  taskSelect.addEventListener('change', () => {
    taskGeneration += 1;
    state = null;
    error.textContent = '';
    byId('downloads').replaceChildren();
    if (connected()) {
      socket.send(JSON.stringify({ type: 'task', task: Number(taskSelect.value) }));
      status.textContent = `Switching to Task ${taskSelect.value}; control is disarmed.`;
    }
    updateControls();
  });
  document.addEventListener('keydown', (event) => {
    if (event.repeat || event.altKey || event.ctrlKey || event.metaKey) return;
    if (event.target instanceof Element && event.target.closest('input, textarea, select, [contenteditable]:not([contenteditable="false"])')) return;
    const key = event.key.toLowerCase();
    if (key === 'q' || key === 'escape') {
      event.preventDefault();
      stopCamera();
    } else if (event.code === 'Space' || event.key === ' ') {
      event.preventDefault();
      sendAction('stop');
    } else if (taskSelect.value === '3') {
      const action = { i: 'initialize', c: 'calibrate', e: 'enable' }[key];
      if (action) { event.preventDefault(); sendAction(action); }
    }
  });
  window.addEventListener('pagehide', () => {
    stopCamera();
  });
  updateControls();
})();
