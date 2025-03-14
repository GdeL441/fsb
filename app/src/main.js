const { invoke } = window.__TAURI__.core;

let ws = null
async function connectWs(url) {
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById("status").textContent = "Status: Connected";
    console.log('WebSocket connection opened')

    hideConnectView()
  };
  ws.onclose = () => {
    console.log('WebSocket connection closed')
    document.getElementById("status").textContent = "Status: Disconnected";
    ws = null
    showConnectView()
  };
  ws.onmessage = event => {
    const data = JSON.parse(event.data)
    console.log(data)
    if (data.action == "next_step") {
      console.log(data.step)
    } else if (data.action == "finished") {
      console.log("Finished")
    }
    console.log(event, data)
  };
  ws.onerror = error => {
    console.error(error)
    document.getElementById("status").textContent = "Status: Disconnected";
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
    span.innerText = `${ssid}`
    button.innerText = "Connect"
    button.addEventListener("click", async () => {
      const _wsUrl = await invoke("connect", { ssid })
      const wsUrl = `ws://${ipInput.value}/ws`
      console.log("connected to wifi", wsUrl)
      if (wsUrl) connectWs(wsUrl)
    });
    li.appendChild(span)
    li.appendChild(button)
    ssids.appendChild(li)
  };
}

let ssids
let loading
let ipInput

window.addEventListener("DOMContentLoaded", () => {
  const btn = document.querySelector("#scan-btn");
  const connectBtn = document.querySelector("#connect-btn");
  const startMotorBtn = document.querySelector("#start-motor");
  const speedInput = document.querySelector("#speed");
  const disconnectBtn = document.querySelector("#disconnect-btn");
  const stopMotorBtn = document.querySelector("#stop-motor");
  ssids = document.querySelector("#wifi-ssids")
  ipInput = document.querySelector("#ip-input")

  loading = document.querySelector("#loading")
  loading.style.display = "none"

  btn.addEventListener("click", (e) => {
    e.preventDefault();
    scan();
  });
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
    showConnectView()
  });
  startMotorBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "start", speed: 100, direction: "forward" }))
  });
  stopMotorBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "stop" }))
  });
  speedInput.oninput = function() {
    if (!ws) return
    if (this.value > 0) {
      ws.send(JSON.stringify({ action: "start_motor", speed: this.value, direction: "forward" }))
    } else {
      ws.send(JSON.stringify({ action: "start_motor", speed: Math.abs(this.value), direction: "backward" }))
    }
  }
});
