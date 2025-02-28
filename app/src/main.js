const { invoke } = window.__TAURI__.core;

async function greet() {
  const wifiSSIDs = await invoke("greet");
  ssids.innerText = wifiSSIDs.join(", ")
}

let ssids

window.addEventListener("DOMContentLoaded", () => {
  const btn = document.querySelector("#scan-btn");
  ssids = document.querySelector("#wifi-ssids")
  btn.addEventListener("click", (e) => {
    e.preventDefault();
    greet();
  });
});
