function addToLog(message, type) {
  const log = document.getElementById('websocket-log');
  const entry = document.createElement('div');
  const timestamp = new Date().toLocaleTimeString();

  entry.innerHTML = `<span class="text-muted">[${timestamp}]</span> <span class="${type === 'sent' ? 'text-danger' : 'text-success'}">${type === 'sent' ? 'App' : 'Pico'}:</span> ${JSON.stringify(message)}`;
  log.appendChild(entry);
  log.scrollTop = log.scrollHeight;
}

document.getElementById('clear-log-btn').addEventListener('click', () => {
  document.getElementById('websocket-log').innerHTML = '';
});

const { invoke } = window.__TAURI__.core;
import { shortestPath } from "./backtrack.js"

let ws = null;

function updateConnectButton(connected) {
  const connectBtn = document.getElementById("connect-btn");
  const connectBtnText = document.getElementById("connect-btn-text");

  if (connected) {
    connectBtn.classList.remove("btn-success");
    connectBtn.classList.add("btn-danger");
    connectBtnText.textContent = "Close WebSocket Connection";
  } else {
    connectBtn.classList.remove("btn-danger");
    connectBtn.classList.add("btn-success");
    connectBtnText.textContent = "Open WebSocket Connection";
  }
}

async function connectWs(url) {
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById("status").textContent = "Connected";
    statusDot.classList.add("bg-success");
    statusDot.classList.remove("bg-danger");
    updateConnectButton(true);
    addToLog('Connection established', 'received');
  };

  ws.onclose = () => {
    console.log('WebSocket connection closed');
    statusDot.classList.remove("bg-success");
    statusDot.classList.add("bg-danger");
    document.getElementById("status").textContent = "Disconnected";
    updateConnectButton(false);
    ws = null;
  };

  ws.onmessage = event => {
    const data = JSON.parse(event.data);
    addToLog(data, 'received');
    if (data.action == "setup") {
      console.log("received setup data", data)
      let threshold = { L: data["L"], R: data["R"], B: data["B"] }
      localStorage.setItem("thresholds", JSON.stringify(threshold));
      setThresholds(threshold.L, threshold.R, threshold.B)
      let speed = { speed: data["speed"] }
      localStorage.setItem("speed", JSON.stringify(speed));
      setSpeed(speed.speed)
      let pid = { P: data["P"], I: data["I"], D: data["D"] }
      localStorage.setItem("pid", JSON.stringify(pid));
      setPID(pid.P, pid.I, pid.D)
    } else if (data.action == "next_step") {
      console.log(data.step)
      step.textContent = `${data.step}`
    } else if (data.action == "position_updated") {
      let newPos = JSON.stringify(data.position)
      let previousPos = JSON.parse(position.textContent)
      if (previousPos != newPos) {
        let greenDot = dots.find((dot) => dot.x == data.position.x && dot.y == data.position.y && dot.color == "green")
        console.log("green", dots, data, score, greenDot)
        if (greenDot) {
          score += 100
          document.querySelector("#score").innerText = score
        }
        let redDot = dots.find((dot) => dot.x == data.position.x && dot.y == data.position.y && dot.color == "red")
        console.log("red", dots, data, score, redDot)
        if (redDot) {
          score -= 50
          document.querySelector("#score").innerText = score
        }
      }

      position.textContent = newPos
      heading.textContent = data.heading
      console.log("heading", data.heading, "position", data.position)
      drawDot(data.position.x, data.position.y, data.heading)
    } else if (data.action == "finished") {
      console.log("Finished")
      if ((timer / 1000) / 60 < 5) {
        score += 200
      } else {
        score += (timer / 1000) * 3
      }
      document.querySelector("#score").innerText = score
      stopTimer()
    } else if (data.action == "sensor_values") {
      console.log("Sensor values", data)
      document.querySelector("#left-sensor-value").textContent = data.L
      document.querySelector("#right-sensor-value").textContent = data.R
      document.querySelector("#back-sensor-value").textContent = data.B
    }

  };
  ws.onerror = error => {
    console.error(error);
    document.getElementById("status").textContent = "Disconnected";
    statusDot.classList.remove("bg-success");
    statusDot.classList.add("bg-danger");
    updateConnectButton(false);
    ws = null;
  };
  const originalSend = ws.send;
  ws.send = function(data) {
    addToLog(JSON.parse(data), 'sent');
    originalSend.call(this, data);
  };
};
async function scan() {
  const loading = document.getElementById('loading');
  const noNetworks = document.getElementById('no-networks');
  const ssidsList = document.getElementById('wifi-ssids');
  const template = document.getElementById('wifi-network-template');

  loading.classList.remove('d-none');
  ssidsList.innerHTML = '';
  noNetworks.classList.add('d-none');

  try {
    const wifiSSIDs = await invoke("scan");
    loading.classList.add('d-none');

    if (wifiSSIDs.length === 0) {
      noNetworks.classList.remove('d-none');
      return;
    }

    wifiSSIDs.forEach(ssid => {
      const networkItem = template.content.cloneNode(true);
      networkItem.querySelector('.network-name').textContent = ssid;

      const connectBtn = networkItem.querySelector('button');
      connectBtn.addEventListener('click', async () => {
        connectToWifi(ssid)
      });

      ssidsList.appendChild(networkItem);
    });

  } catch (error) {
    console.error('Error scanning networks:', error);
    loading.classList.add('d-none');
    noNetworks.textContent = 'Error scanning networks. Please try again.';
    noNetworks.classList.remove('d-none');
  }
}

async function connectToWifi(ssid) {
  const res = await invoke("connect", { ssid })
  console.log(res)
};



let timer;
let score
let milliseconds = 0;
let isRunning = false;

function formatTime(ms) {
  let mins = Math.floor(ms / 60000);
  let secs = Math.floor((ms % 60000) / 1000);
  let millis = ms % 1000;
  return (
    String(mins).padStart(2, '0') + " : " +
    String(secs).padStart(2, '0') + " : " +
    String(millis).padStart(2, '0')
  );
}

function updateDisplay() {
  document.getElementById('timer').textContent = formatTime(milliseconds);
}

function startTimer() {
  if (!isRunning) {
    isRunning = true;
    let startTime = Date.now() - milliseconds;
    timer = setInterval(() => {
      milliseconds = Date.now() - startTime;
      updateDisplay();
    }, 10);
  }
}

function stopTimer() {
  clearInterval(timer);
  isRunning = false;
}

function resetTimer() {
  stopTimer();
  milliseconds = 0;
  updateDisplay();
}

let ssids, loading, ipInput, position, heading, step, statusDot

let canvas, ctx;
const COLS = 7;
const ROWS = 5;
let cellSize;
let dots = [];
let path = []
let solution = null
let manual_control = false

let currentPosition = {
  x: null,
  y: null,
  direction: null
};

function calculateCellSize() {
  const horizontalSize = canvas.width / COLS;
  const verticalSize = canvas.height / ROWS;
  return Math.min(horizontalSize, verticalSize);
}

function resizeCanvas() {
  canvas = document.getElementById('gridCanvas');

  const parent = canvas.parentElement;

  const width = parent.clientWidth;
  const height = parent.clientHeight;

  canvas.width = width;
  canvas.height = height;

  cellSize = calculateCellSize();

  drawGrid();

  if (currentPosition.x !== null && currentPosition.y !== null && currentPosition.direction !== null) {
    drawDot(currentPosition.x, currentPosition.y, currentPosition.direction);
  }
}

function drawGrid() {
  ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Center the grid
  const gridWidth = COLS * cellSize;
  const gridHeight = ROWS * cellSize;
  const offsetX = (canvas.width - gridWidth) / 2;
  const offsetY = (canvas.height - gridHeight) / 2;

  ctx.strokeStyle = "black";
  ctx.lineWidth = 3;

  // Draw vertical lines
  for (let i = 1; i <= COLS - 1; i++) {
    ctx.beginPath();
    ctx.moveTo(offsetX + i * cellSize, offsetY);
    ctx.lineTo(offsetX + i * cellSize, offsetY + gridHeight);
    ctx.stroke();
  }

  // Draw horizontal lines
  for (let i = 1; i <= ROWS - 1; i++) {
    ctx.beginPath();
    ctx.moveTo(offsetX, offsetY + i * cellSize);
    ctx.lineTo(offsetX + gridWidth, offsetY + i * cellSize);
    ctx.stroke();
  }

  drawPath()

  dots.forEach(dot => {
    if (dot.color == "start") {
      ctx.fillStyle = "blue";
    } else {
      ctx.fillStyle = dot.color;
    }
    ctx.beginPath();
    ctx.arc(dot.x * cellSize, (ROWS - dot.y) * cellSize, cellSize / 6, 0, Math.PI * 2);
    ctx.fill();
  });

}

function drawPath() {
  const gridWidth = COLS * cellSize;
  const gridHeight = ROWS * cellSize;
  const offsetX = (canvas.width - gridWidth) / 2;
  const offsetY = (canvas.height - gridHeight) / 2;

  for (let i = 0; i < path.length - 1; i++) {
    let from = path[i], to = path[i + 1]
    const fromX = offsetX + from.x * cellSize, fromY = offsetY + (ROWS - from.y) * cellSize;
    const toX = offsetX + to.x * cellSize, toY = offsetY + (ROWS - to.y) * cellSize;

    ctx.strokeStyle = "purple";
    ctx.lineWidth = 6;
    ctx.beginPath();

    ctx.moveTo(fromX, fromY); // Start at circle edge
    ctx.lineTo(toX, toY); // Base of arrow at circle edge
    ctx.stroke()

    // ctx.fillStyle = "purple";
    // ctx.beginPath();
    // const arrowSize = cellSize / 4; // Slightly smaller arrow
    // const dotX = (fromX + toX) / 2
    // const dotY = (fromY + toY) / 2
    // const dotRadius = cellSize / 4;
    // ctx.moveTo(dotX, dotY - dotRadius - arrowSize); // Start at circle edge
    // ctx.lineTo(dotX - arrowSize / 2, dotY - dotRadius); // Base of arrow at circle edge
    // ctx.lineTo(dotX + arrowSize / 2, dotY - dotRadius);
    // ctx.fill()
  };
}


function drawDot(x, y, direction) {
  // Update current position
  currentPosition.x = x;
  currentPosition.y = y;
  currentPosition.direction = direction;

  const gridWidth = COLS * cellSize;
  const gridHeight = ROWS * cellSize;
  const offsetX = (canvas.width - gridWidth) / 2;
  const offsetY = (canvas.height - gridHeight) / 2;

  drawGrid();

  // Calculate dot center position
  const dotX = offsetX + x * cellSize;
  const dotY = offsetY + (ROWS - y) * cellSize;

  const dotRadius = cellSize / 4;

  // Draw the dot
  ctx.fillStyle = "blue";
  ctx.beginPath();
  ctx.arc(dotX, dotY, dotRadius, 0, Math.PI * 2);
  ctx.fill();

  // Draw direction indicator
  ctx.fillStyle = "blue";
  ctx.beginPath();
  const arrowSize = cellSize / 4; // Slightly smaller arrow

  // Update arrow drawing with new offsets and positioning on circle edge
  switch (direction) {
    case "N":
      ctx.moveTo(dotX, dotY - dotRadius - arrowSize); // Start at circle edge
      ctx.lineTo(dotX - arrowSize / 2, dotY - dotRadius); // Base of arrow at circle edge
      ctx.lineTo(dotX + arrowSize / 2, dotY - dotRadius);
      break;
    case "E":
      ctx.moveTo(dotX + dotRadius + arrowSize, dotY); // Point
      ctx.lineTo(dotX + dotRadius, dotY - arrowSize / 2); // Top of base
      ctx.lineTo(dotX + dotRadius, dotY + arrowSize / 2); // Bottom of base
      break;
    case "S":
      ctx.moveTo(dotX, dotY + dotRadius + arrowSize);
      ctx.lineTo(dotX - arrowSize / 2, dotY + dotRadius);
      ctx.lineTo(dotX + arrowSize / 2, dotY + dotRadius);
      break;
    case "W":
      ctx.moveTo(dotX - dotRadius - arrowSize, dotY);
      ctx.lineTo(dotX - dotRadius, dotY - arrowSize / 2);
      ctx.lineTo(dotX - dotRadius, dotY + arrowSize / 2);
      break;
  }
  ctx.fill();
}

function addDot(x, y, color) {
  if (x > 0 && y > 0 && x < COLS && y < ROWS) {
    if (color == "start" && dots.find((dot => dot.color == "start"))) {
      const index = dots.findIndex(dot => dot.color == "start");
      dots.splice(index, 1);
    }

    const index = dots.findIndex(dot => dot.x === x && dot.y === y);
    if (index !== -1) {
      dots.splice(index, 1);
    } else {
      dots.push({ x, y, color });
    }
    drawGrid();
  }
}

// Add event listener for window resize with debouncing
let resizeTimeout;
window.addEventListener('resize', () => {
  clearTimeout(resizeTimeout);
  resizeTimeout = setTimeout(resizeCanvas, 100);
});

// Initial setup when the page loads
window.addEventListener("DOMContentLoaded", () => {
  canvas = document.getElementById("gridCanvas");

  canvas.addEventListener("click", function(event) {
    const rect = canvas.getBoundingClientRect();
    const x = Math.round((event.clientX - rect.left) / cellSize);
    const y = Math.round(ROWS - (event.clientY - rect.top) / cellSize);
    addDot(x, y, "green");
  });

  canvas.addEventListener("contextmenu", function(event) {
    event.preventDefault(); // Voorkom contextmenu
    const rect = canvas.getBoundingClientRect();
    const x = Math.round((event.clientX - rect.left) / cellSize);
    const y = Math.round(ROWS - (event.clientY - rect.top) / cellSize);
    addDot(x, y, "red");
  });
  canvas.addEventListener("dblclick", function(event) {
    event.preventDefault(); // Voorkom contextmenu
    const rect = canvas.getBoundingClientRect();
    const x = Math.round((event.clientX - rect.left) / cellSize);
    const y = Math.round(ROWS - (event.clientY - rect.top) / cellSize);
    addDot(x, y, "start");
  });

  resizeCanvas(); // This will set up the initial canvas size and draw the grid
  loadThresholds()
  loadPID()
  loadSpeed()

  const connectBtn = document.querySelector("#connect-btn");
  const scanBtn = document.querySelector("#scan-btn")

  const startBtn = document.querySelector("#start");
  const stopBtn = document.querySelector("#stop");
  const resetBtn = document.querySelector("#reset");
  const sensorsBtn = document.querySelector("#monitor-btn")
  const applyThresholdBtn = document.querySelector("#apply-thresholds-btn")
  const applySpeedBtn = document.querySelector("#apply-speed-btn")
  const applyPIDBtn = document.querySelector("#apply-pid-btn")
  const solveBtn = document.querySelector("#solve")
  const clearBtn = document.querySelector("#clear-btn")
  const sidebarBtn = document.querySelector("#sidebar-hide")
  const manualBtn = document.querySelector("#manual-control-btn")

  loading = document.querySelector("#loading")
  loading.classList.add("d-none")

  ssids = document.querySelector("#wifi-ssids")
  ipInput = document.querySelector("#ip-input")
  position = document.querySelector("#position")
  heading = document.querySelector("#heading_")
  step = document.querySelector("#step")
  statusDot = document.querySelector("#status-dot")

  sidebarBtn.addEventListener("click", () => {
    resizeCanvas()
    setTimeout(resizeCanvas, 500)
  })
  scanBtn.addEventListener("click", () => {
    scan()
  })
  connectBtn.addEventListener("click", async () => {
    if (ws) {
      ws.close();
      return;
    }
    const wsUrl = `ws://${ipInput.value}/ws`;
    console.log("Connect to", wsUrl);
    connectWs(wsUrl);
  });

  startBtn.addEventListener("click", async () => {
    const startPos = dots.find(dot => dot.color == "start")
    if (startPos)
      drawDot(startPos.x, startPos.y, "N");

    if (!ws) return

    if (solution)
      ws.send(JSON.stringify({ action: "start", path: solution.directions, heading: "N", startX: startPos?.x, startY: startPos?.y }))
    else
      ws.send(JSON.stringify({ action: "start" }))

    startTimer()
  });

  stopBtn.addEventListener("click", async () => {
    if (!ws) return
    ws.send(JSON.stringify({ action: "stop" }))
    stopTimer()
  });

  resetBtn.addEventListener("click", async () => {
    if (!ws) return
    ws.send(JSON.stringify({ action: "reset" }))
    stopTimer()
    resetTimer()
    score = 0
    currentPosition = {
      x: 6,
      y: 0,
      direction: null
    };
    drawGrid()
  });

  solveBtn.addEventListener("click", async () => {
    solution = shortestPath(COLS - 1, ROWS - 1, dots)
    console.log("solution", solution)
    if (solution) {
      path = solution.path
    } else {
      path = []
    }
    drawGrid()
  });

  sensorsBtn.addEventListener("click", async () => {
    if (!ws) return
    console.log("start monitor sensor")
    ws.send(JSON.stringify({ action: "monitor_sensor" }))
  });

  applyThresholdBtn.addEventListener("click", async () => {
    if (!ws) return
    let L = Number(document.querySelector("#left-sensor-threshold").value)
    let R = Number(document.querySelector("#right-sensor-threshold").value)
    let B = Number(document.querySelector("#back-sensor-threshold").value)
    // Save to localStorage
    localStorage.setItem("thresholds", JSON.stringify({ L, R, B }));

    ws.send(JSON.stringify({ action: "set_threshold", L, R, B }))
  });

  applyPIDBtn.addEventListener("click", async () => {
    if (!ws) return
    let P = Number(document.querySelector("#kp-value").value)
    let I = Number(document.querySelector("#ki-value").value)
    let D = Number(document.querySelector("#kd-value").value)
    // Save to localStorage
    localStorage.setItem("pid", JSON.stringify({ P, I, D }));

    ws.send(JSON.stringify({ action: "update_pid", P, I, D }))
  });

  applySpeedBtn.addEventListener("click", async () => {
    if (!ws) return
    let speed = Number(document.querySelector("#speed-value").value)
    // Save to localStorage
    localStorage.setItem("speed", JSON.stringify({ speed }));

    ws.send(JSON.stringify({ action: "update_speed", speed }))
  });
  clearBtn.addEventListener("click", async () => {
    path = []
    dots = []
    solution = null
    drawGrid()
  });
  window.addEventListener("keydown", keyPressed);
  window.addEventListener("keyup", keyUp);

  manualBtn.addEventListener("click", async () => {
    if (!ws) return
    manual_control = !manual_control

    if (manualBtn.classList.contains("btn-primary")) {
      manualBtn.innerText = "Stop Takeover"
      manualBtn.classList.remove("btn-primary");
      manualBtn.classList.add("btn-danger");
    } else {
      manualBtn.innerText = "Takeover"
      manualBtn.classList.add("btn-primary");
      manualBtn.classList.remove("btn-danger");
    }

    ws.send(JSON.stringify({ action: "manual_control" }))
  });
});


function calculateMotorSpeeds() {
  let leftSpeed = 0, rightSpeed = 0

  if (document.querySelector(`div[data-key="87"]`).classList.contains("activeKey")) {
    leftSpeed += 50
    rightSpeed += 50
  }
  if (document.querySelector(`div[data-key="83"]`).classList.contains("activeKey")) {
    leftSpeed -= 50
    rightSpeed -= 50
  }
  if (document.querySelector(`div[data-key="65"]`).classList.contains("activeKey")) {
    rightSpeed += 50
  }
  if (document.querySelector(`div[data-key="68"]`).classList.contains("activeKey")) {
    leftSpeed += 50
  }

  if (!ws || manual_control) return
  ws.send(JSON.stringify({ action: "manual_control", speeds: { left: leftSpeed, right: rightSpeed } }))
}

function keyPressed(e) {
  // Assigns key "div" to key
  const key = document.querySelector(`div[data-key="${e.keyCode}"]`);
  // Only applies activeKey to the keys displayed in browser
  if (
    e.keyCode === 87 ||
    e.keyCode === 65 ||
    e.keyCode === 83 ||
    e.keyCode === 68
  ) {
    // Adds class activeKey
    key.classList.add("activeKey");
  }

  calculateMotorSpeeds()
}

function keyUp(e) {
  // Assigns key "div" to key
  const key = document.querySelector(`div[data-key="${e.keyCode}"]`);
  // Only applies activeKey to the keys displayed in browser
  if (
    e.keyCode === 87 ||
    e.keyCode === 65 ||
    e.keyCode === 83 ||
    e.keyCode === 68
  ) {
    // Adds class activeKey
    key.classList.remove("activeKey");
  }
  calculateMotorSpeeds()
}

function loadThresholds() {
  let savedThresholds = localStorage.getItem("thresholds");
  console.log(savedThresholds)

  if (savedThresholds) {
    let { L, R, B } = JSON.parse(savedThresholds);
    setThresholds(L, R, B)
  }
}

function setThresholds(L, R, B) {
  document.querySelector("#left-sensor-threshold").value = L;
  document.querySelector("#right-sensor-threshold").value = R;
  document.querySelector("#back-sensor-threshold").value = B;
}

function loadPID() {
  let savedPID = localStorage.getItem("pid");
  console.log(savedPID)

  if (savedPID) {
    let { P, I, D } = JSON.parse(savedPID);
    setPID(P, I, D)
  }
}

function setPID(P, I, D) {
  document.querySelector("#kp-value").value = P
  document.querySelector("#ki-value").value = I
  document.querySelector("#kd-value").value = D
}

function loadSpeed() {
  let savedSpeed = localStorage.getItem("speed");
  console.log(savedSpeed)

  if (savedSpeed) {
    let { speed } = JSON.parse(savedSpeed);
    setSpeed(speed)
  }
}

function setSpeed(speed) {
  document.querySelector("#speed-value").value = speed
}

