const { invoke } = window.__TAURI__.core;

let ws = null
async function connectWs(url) {
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById("status").textContent = "Connected";
    statusDot.classList.add("bg-success")
    statusDot.classList.remove("bg-danger")
    console.log('WebSocket connection opened')

    hideConnectView()
  };
  ws.onclose = () => {
    console.log('WebSocket connection closed')
    statusDot.classList.remove("bg-success")
    statusDot.classList.add("bg-danger")
    document.getElementById("status").textContent = "Disconnected";
    ws = null
    showConnectView()
  };
  ws.onmessage = event => {
    const data = JSON.parse(event.data)
    console.log("data received from websocket", data)
    if (data.action == "next_step") {
      console.log(data.step)
      step.textContent = `${data.step}`
    } else if (data.action == "position_updated") {
      position.textContent = JSON.stringify(data.position)
      heading.textContent = `Heading: ${data.heading} `
      console.log("heading", data.heading, "position", data.position)
      drawDot(data.position.x, data.position.y, data.heading)
    } else if (data.action == "finished") {
      console.log("Finished")
      stopTimer()
    } else if (data.action == "sensor_values") {
      console.log("Sensor values", data)
      sensors.textContent = `Left: ${data.L}, Right: ${data.R}, Back: ${data.B} `
    }

  };
  ws.onerror = error => {
    console.error(error)
    document.getElementById("status").textContent = "Disconnected";
    statusDot.classList.remove("bg-success")
    statusDot.classList.add("bg-danger")
    ws = null
    showConnectView()
  };
};

function toggleConnectView() {
  document.getElementById("connect-view").classList.toggle("hide")
  document.getElementById("dashboard").classList.toggle("hide")
};
function showConnectView() {
  document.getElementById("connect-view").classList.remove("hide")
  document.getElementById("dashboard").classList.add("hide")
};
function hideConnectView() {
  document.getElementById("connect-view").classList.add("hide")
  document.getElementById("dashboard").classList.remove("hide")
};

async function scan() {
  loading.style.display = "block"
  const wifiSSIDs = await invoke("scan");
  loading.style.display = "none"

  // Show result in html
  ssids.innerHTML = ""
  for (const ssid of wifiSSIDs) {
    const li = document.createElement("li")
    const button = document.createElement("button")
    const span = document.createElement("span")
    span.innerText = `${ssid} `
    button.innerText = "Connect"
    button.addEventListener("click", async () => {
      // const _wsUrl = await invoke("connect", { ssid })
      const wsUrl = `ws://${ipInput.value}/ws`
      console.log("connected to wifi", wsUrl)
      if (wsUrl) connectWs(wsUrl)
    });
    li.appendChild(span)
    li.appendChild(button)
    ssids.appendChild(li)
  };
}

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

let canvas, ctx
const COLS = 7; // Aantal rijen en kolommen
const ROWS = 5; // Aantal rijen en kolommen
const cellSize = 100; // Grootte van een cel in pixels


function drawGrid() {
  canvas = document.getElementById("gridCanvas");
  ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  canvas.width = COLS * cellSize;
  canvas.height = ROWS * cellSize;
  ctx.strokeStyle = "black";
  ctx.lineWidth = 3;

  for (let i = 0; i <= COLS; i++) {
    // Verticale lijnen
    ctx.beginPath();
    ctx.moveTo(i * cellSize, 0);
    ctx.lineTo(i * cellSize, canvas.height);
    ctx.stroke();
  }
  for (let i = 0; i <= ROWS; i++) {
    // Horizontale lijnen
    ctx.beginPath();
    ctx.moveTo(0, i * cellSize);
    ctx.lineTo(canvas.width, i * cellSize);
    ctx.stroke();
  }
}

function drawDot(x, y, direction) {
  drawGrid();
  ctx.fillStyle = "blue";
  ctx.beginPath();
  ctx.arc(x * cellSize, (ROWS - y) * cellSize, cellSize / 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "blue";
  ctx.beginPath();
  const arrowSize = cellSize / 3;

  switch (direction) {
    case "N":
      ctx.moveTo(x * cellSize, (ROWS - y - 0.85) * cellSize + arrowSize);
      ctx.lineTo(x * cellSize - arrowSize / 2, (ROWS - y - 0.85) * cellSize + arrowSize * 2);
      ctx.lineTo(x * cellSize + arrowSize / 2, (ROWS - y - 0.85) * cellSize + arrowSize * 2);
      break;
    case "E":
      ctx.moveTo((x + 0.85) * cellSize - arrowSize, (ROWS - y) * cellSize);
      ctx.lineTo((x + 0.85) * cellSize - arrowSize * 2, (ROWS - y) * cellSize - arrowSize / 2);
      ctx.lineTo((x + 0.85) * cellSize - arrowSize * 2, (ROWS - y) * cellSize + arrowSize / 2);
      break;
    case "S":
      ctx.moveTo(x * cellSize, (ROWS - y + 0.85) * cellSize - arrowSize);
      ctx.lineTo(x * cellSize - arrowSize / 2, (ROWS - y + 0.85) * cellSize - arrowSize * 2);
      ctx.lineTo(x * cellSize + arrowSize / 2, (ROWS - y + 0.85) * cellSize - arrowSize * 2);
      break;
    case "W":
      ctx.moveTo((x - 0.85) * cellSize + arrowSize, (ROWS - y) * cellSize);
      ctx.lineTo((x - 0.85) * cellSize + arrowSize * 2, (ROWS - y) * cellSize - arrowSize / 2);
      ctx.lineTo((x - 0.85) * cellSize + arrowSize * 2, (ROWS - y) * cellSize + arrowSize / 2);
      break;
  }
  ctx.fill()
}


window.addEventListener("DOMContentLoaded", () => {
  const scanBtn = document.querySelector("#scan-btn");
  const connectBtn = document.querySelector("#connect-btn");
  const startMotorBtn = document.querySelector("#start-motor");
  const speedInput = document.querySelector("#speed");
  const disconnectBtn = document.querySelector("#disconnect-btn");
  const stopMotorBtn = document.querySelector("#stop-motor");
  const sensorsBtn = document.querySelector("#sensors-btn");

  const startBtn = document.querySelector("#start");
  const stopBtn = document.querySelector("#stop");
  const resetBtn = document.querySelector("#reset");

  drawGrid()

  ssids = document.querySelector("#wifi-ssids")
  ipInput = document.querySelector("#ip-input")
  position = document.querySelector("#position")
  heading = document.querySelector("#heading_")
  step = document.querySelector("#step")
  sensors = document.querySelector("#sensors")
  statusDot = document.querySelector("#status-dot")

  // loading = document.querySelector("#loading")
  // loading.style.display = "none"

  // scanBtn.addEventListener("click", (e) => {
  //   e.preventDefault();
  //   scan();
  // });
  // connectBtn.addEventListener("click", async (e) => {
  //   e.preventDefault();
  //   // const url = await invoke("get_url");

  //   const wsUrl = `ws://${ipInput.value}/ws`
  //   console.log("Connect to", wsUrl)
  //   connectWs(wsUrl)
  // });
  disconnectBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    if (!ws) return
    ws.close()
    ws = null
    showConnectView()
  });

  // startMotorBtn.addEventListener("click", async (e) => {
  //   e.preventDefault();
  //   if (!ws) return
  //   ws.send(JSON.stringify({ action: "move", speedL: 100, speedR: 100 }))
  // });
  // stopMotorBtn.addEventListener("click", async (e) => {
  //   e.preventDefault();
  //   if (!ws) return
  //   ws.send(JSON.stringify({ action: "move", speedL: 0, speedR: 0 }))
  // });

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

  // sensorsBtn.addEventListener("click", async (e) => {
  //   e.preventDefault();
  //   if (!ws) return
  //   ws.send(JSON.stringify({ action: "monitor_sensor" }))
  // });

});
