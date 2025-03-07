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
    console.log(event, data)
  };
  ws.onerror = error => {
    console.error(error)
    document.getElementById("status").textContent = "Status: Disconnected";
    ws = null
    showConnectView()
  };
};

const toggleConnectView = () => {
  document.getElementById("connect-view").classList.toggle("hide")
  document.getElementById("dashboard").classList.toggle("hide")
};
const showConnectView = () => {
  document.getElementById("connect-view").classList.remove("hide")
  document.getElementById("dashboard").classList.add("hide")
};
const hideConnectView = () => {
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
      const wsUrl = await invoke("connect", { ssid })
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

window.addEventListener("DOMContentLoaded", () => {
  const btn = document.querySelector("#scan-btn");
  const connectBtn = document.querySelector("#connect-btn");
  const startMotorBtn = document.querySelector("#start-motor");
  ssids = document.querySelector("#wifi-ssids")

  loading = document.querySelector("#loading")
  loading.style.display = "none"

  btn.addEventListener("click", (e) => {
    e.preventDefault();
    scan();
  });
  connectBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    const url = await invoke("get_url");
    connectWs(url)
  });
  startMotorBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    if (!ws) return
    ws.send(JSON.stringify({ action: "start_motor", speed: 100 }))
  });
});
