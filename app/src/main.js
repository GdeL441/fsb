const { invoke } = window.__TAURI__.core;

async function connectWs(url) {
  const ws = new WebSocket(url);

  ws.onopen = () => console.log('WebSocket connection opened');
  ws.onclose = () => console.log('WebSocket connection closed');
  ws.onmessage = event => {
    console.log(event)
  };
  ws.onerror = error => {
    console.error(error)
  };

  // Connection opened
  socket.addEventListener("open", (event) => {
    document.getElementById("status").textContent = "Status: Connected";
  });

  socket.addEventListener("close", (event) => {
    socket = undefined;
    document.getElementById("status").textContent = "Status: Disconnected";
  });

  socket.addEventListener("message", (event) => {
    console.log(event.data)
  });

  socket.addEventListener("error", (event) => {
    socket = undefined;
    document.getElementById("status").textContent = "Status: Disconnected";
  });
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
});
