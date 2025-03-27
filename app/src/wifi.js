const { invoke } = window.__TAURI__.core;


let ws = null
async function connectWs(url) {
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById("status").textContent = "Connected";
    console.log('WebSocket connection opened')
  };
  ws.onclose = () => {
    console.log('WebSocket connection closed')
    statusDot.classList.remove("bg-success")
    statusDot.classList.add("bg-danger")
    document.getElementById("status").textContent = "Disconnected";
    ws = null
  };
  ws.onerror = error => {
    console.error(error)
    document.getElementById("status").textContent = "Disconnected";
    statusDot.classList.remove("bg-success")
    statusDot.classList.add("bg-danger")
    ws = null
  };
};

async function connectToWifi(ssid) {
  const res = await invoke("connect", { ssid })
  console.log(res)
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

let loading, ipInput, statusDot

// Initial setup when the page loads
window.addEventListener("DOMContentLoaded", () => {
  const scanBtn = document.querySelector("#scan-btn")
  const connectBtn = document.querySelector("#connect-btn")

  statusDot = document.querySelector("#status-dot")
  ipInput = document.querySelector("#ip-input")
  loading = document.querySelector("#loading")
  loading.classList.add("d-none")

  connectBtn.addEventListener("click", async (e) => {
    e.preventDefault();
    // const url = await invoke("get_url");

    const wsUrl = `ws://${ipInput.value}/ws`
    console.log("Connect to", wsUrl)
    connectWs(wsUrl)
  });
  scanBtn.addEventListener("click", (e) => {
    scan()
  })
});
