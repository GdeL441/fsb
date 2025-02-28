const { invoke } = window.__TAURI__.core;

async function scan() {
  loading.style.display = "block"
  const wifiSSIDs = await invoke("scan");
  loading.style.display = "none"

  ssids.innerHTML = ""
  for (const ssid of wifiSSIDs) {
    const li = document.createElement("li")
    const button = document.createElement("button")
    const span = document.createElement("span")
    span.innerText = `${ssid}`
    button.innerText = "Connect"
    button.addEventListener("click", async () => {
      const res = await invoke("connect", { ssid })
      console.log("connected to wifi", res)
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
  ssids = document.querySelector("#wifi-ssids")

  loading = document.querySelector("#loading")
  loading.style.display = "none"

  btn.addEventListener("click", (e) => {
    e.preventDefault();
    scan();
  });
});
