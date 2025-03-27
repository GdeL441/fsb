const { invoke } = window.__TAURI__.core;

let ws = null
async function connectWs(url) {
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById("status").textContent = "Connected";
    statusDot.classList.add("bg-success")
    statusDot.classList.remove("bg-danger")
    console.log('WebSocket connection opened')

  };
  ws.onclose = () => {
    console.log('WebSocket connection closed')
    statusDot.classList.remove("bg-success")
    statusDot.classList.add("bg-danger")
    document.getElementById("status").textContent = "Disconnected";
    ws = null
  };
  ws.onmessage = event => {
    const data = JSON.parse(event.data)
    console.log("data received from websocket", data)
    if (data.action == "next_step") {
      console.log(data.step)
      step.textContent = `${data.step}`
    } else if (data.action == "position_updated") {
      position.textContent = JSON.stringify(data.position)
      heading.textContent = data.heading
      console.log("heading", data.heading, "position", data.position)
      drawDot(data.position.x, data.position.y, data.heading)
    } else if (data.action == "finished") {
      console.log("Finished")
      stopTimer()
    } else if (data.action == "sensor_values") {
      console.log("Sensor values", data)
      // sensors.textContent = `Left: ${data.L}, Right: ${data.R}, Back: ${data.B} `
      document.querySelector("#left-sensor-value").textContent = data.L
      document.querySelector("#right-sensor-value").textContent = data.R
      document.querySelector("#back-sensor-value").textContent = data.B
    }

  };
  ws.onerror = error => {
    console.error(error)
    document.getElementById("status").textContent = "Disconnected";
    statusDot.classList.remove("bg-success")
    statusDot.classList.add("bg-danger")
    ws = null
  };
};

let timer;
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

let ssids, loading, ipInput, position, heading, step, sensors, statusDot

let canvas, ctx;
const COLS = 7; // Aantal rijen en kolommen
const ROWS = 5; // Aantal rijen en kolommen
let cellSize; // Make cellSize dynamic

// Add state variables to track current position
let currentPosition = {
  x: null,
  y: null,
  direction: null
};

function calculateCellSize() {
  // Calculate cell size based on canvas dimensions and grid size
  const horizontalSize = canvas.width / COLS;
  const verticalSize = canvas.height / ROWS;
  // Use the smaller value to ensure squares fit both dimensions
  return Math.min(horizontalSize, verticalSize);
}

function resizeCanvas() {
  canvas = document.getElementById('gridCanvas');
  const parent = canvas.parentElement;

  // Get the parent's computed width
  const width = parent.clientWidth;
  const height = parent.clientHeight;

  // Set canvas size to match CSS size
  canvas.width = width;
  canvas.height = height;

  // Recalculate cell size
  cellSize = calculateCellSize();

  // Redraw everything
  drawGrid();

  // If there's a current position, redraw the dot using currentPosition state
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
  for (let i = 0; i <= COLS; i++) {
    ctx.beginPath();
    ctx.moveTo(offsetX + i * cellSize, offsetY);
    ctx.lineTo(offsetX + i * cellSize, offsetY + gridHeight);
    ctx.stroke();
  }

  // Draw horizontal lines
  for (let i = 0; i <= ROWS; i++) {
    ctx.beginPath();
    ctx.moveTo(offsetX, offsetY + i * cellSize);
    ctx.lineTo(offsetX + gridWidth, offsetY + i * cellSize);
    ctx.stroke();
  }
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

// Add event listener for window resize with debouncing
let resizeTimeout;
window.addEventListener('resize', () => {
  clearTimeout(resizeTimeout);
  resizeTimeout = setTimeout(resizeCanvas, 100);
});

// Initial setup when the page loads
window.addEventListener("DOMContentLoaded", () => {
  canvas = document.getElementById("gridCanvas");
  resizeCanvas(); // This will set up the initial canvas size and draw the grid

  const connectBtn = document.querySelector("#connect-btn");
  const disconnectBtn = document.querySelector("#disconnect-btn");

  const startBtn = document.querySelector("#start");
  const stopBtn = document.querySelector("#stop");
  const resetBtn = document.querySelector("#reset");

  ssids = document.querySelector("#wifi-ssids")
  ipInput = document.querySelector("#ip-input")
  position = document.querySelector("#position")
  heading = document.querySelector("#heading_")
  step = document.querySelector("#step")
  statusDot = document.querySelector("#status-dot")

  connectBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    // const url = await invoke("get_url");

    const wsUrl = `ws://${ipInput.value}/ws`
    console.log("Connect to", wsUrl)
    connectWs(wsUrl)
  });
  disconnectBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    if (!ws) return
    ws.close()
    ws = null
  });

  startBtn.addEventListener("click", async (e) => {
    // drawDot(1, 1, "S");
    // startTimer()
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "start" }))
    startTimer()
  });

  stopBtn.addEventListener("click", async (e) => {
    // drawDot(1, 1, "W");
    // stopTimer()
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "stop" }))
    stopTimer()
  });

  resetBtn.addEventListener("click", async (e) => {
    // drawDot(2, 1, "E");
    // stopTimer()
    // resetTimer()
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "reset" }))
    stopTimer()
    resetTimer()
  });

  sensorsBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "monitor_sensor" }))
  });
});
