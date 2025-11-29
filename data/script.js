// ==================== WEBSOCKET ====================
var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var gaugeTemp = null;
var gaugeHumi = null;

window.addEventListener('load', onLoad);

function onLoad(event) {
    initWebSocket();
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function Send_Data(data) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(data);
        console.log("📤 Gửi:", data);
    } else {
        console.warn("⚠️ WebSocket chưa sẵn sàng!");
        alert("⚠️ WebSocket chưa kết nối!");
    }
}

function onMessage(event) {
    console.log("📩 Nhận:", event.data);
    try {
        var data = JSON.parse(event.data);
        var page = data.page;
        var value = data.value;

        // 1. Xử lý dữ liệu Sensor (Task 3)
        if (page === "home" && value) {
            const temp = value.temp;
            const humi = value.humi;

            if (gaugeTemp) gaugeTemp.refresh(temp);
            if (gaugeHumi) gaugeHumi.refresh(humi);

            if (typeof updateGaugeTextColor === "function") {
                updateGaugeTextColor();
            }
        } 
        
        // 2. Xử lý phản hồi trạng thái thiết bị (Task 4)
        else if (page === "device_update" && value) {
            const receivedGPIO = value.gpio;
            const receivedStatus = (value.status === "ON");
            
            // Tìm và cập nhật trạng thái trong mảng relayList
            const relay = relayList.find(r => r.gpio == receivedGPIO);
            if (relay) {
                relay.state = receivedStatus;
                renderRelays(); // Vẽ lại giao diện để hiển thị trạng thái mới
            }
        }

        // 3. Xử lý phản hồi Cài đặt (Task 6 - Tùy chọn)
        else if (page === "settings_status") {
             // Nếu ESP32 gửi lại trạng thái kết nối
             alert("Trạng thái kết nối WiFi: " + value.message);
        }

    } catch (e) {
        console.warn("⚠️ Không phải JSON hợp lệ:", event.data);
        console.error("Lỗi phân tích JSON:", e);
    }
}


// ==================== UI NAVIGATION ====================
let relayList = [];
let deleteTarget = null;

function showSection(id, event) {
    document.querySelectorAll('.section').forEach(sec => sec.style.display = 'none');
    document.getElementById(id).style.display = id === 'settings' ? 'flex' : 'block';
    document.querySelectorAll('.nav-item').forEach(i => i.classList.remove('active'));
    event.currentTarget.classList.add('active');
}


// ==================== HOME GAUGES ====================
window.onload = function () {
    gaugeTemp = new JustGage({
        id: "gauge_temp",
        value: 0,
        min: -10,
        max: 50,
        title: "Nhiệt độ",
        label: "°C",
        donut: true,
        pointer: false,
        gaugeWidthScale: 0.25,
        gaugeColor: "transparent",
        levelColorsGradient: true,
        levelColors: ["#00BCD4", "#4CAF50", "#FFC107", "#F44336"]
    });

    gaugeHumi = new JustGage({
        id: "gauge_humi",
        value: 0,
        min: 0,
        max: 100,
        title: "Độ ẩm",
        label: "%",
        donut: true,
        pointer: false,
        gaugeWidthScale: 0.25,
        gaugeColor: "transparent",
        levelColorsGradient: true,
        levelColors: ["#42A5F5", "#00BCD4", "#0288D1"]
    });
    // --- Hàm cập nhật màu chữ trong gauge ---
    function updateGaugeTextColor() {
        const textColor = getComputedStyle(document.documentElement)
            .getPropertyValue('--text-color')
            .trim();

        // chọn text SVG bên trong gauge và đổi fill
        document.querySelectorAll('#gauge_temp text, #gauge_humi text')
            .forEach(el => el.setAttribute('fill', textColor));
    }

    // cập nhật màu ban đầu
    updateGaugeTextColor();

    // --- lưu hàm để dùng lại trong toggleTheme ---
    window.updateGaugeTextColor = updateGaugeTextColor;
};

// ==================== DEVICE FUNCTIONS ====================
function openAddRelayDialog() {
    document.getElementById('addRelayDialog').style.display = 'flex';
}
function closeAddRelayDialog() {
    document.getElementById('addRelayDialog').style.display = 'none';
}
function saveRelay() {
    const name = document.getElementById('relayName').value.trim();
    const gpio = document.getElementById('relayGPIO').value.trim();
    if (!name || !gpio) return alert("⚠️ Please fill all fields!");
    relayList.push({ id: Date.now(), name, gpio, state: false });
    renderRelays();
    closeAddRelayDialog();
}
function renderRelays() {
    const container = document.getElementById('relayContainer');
    container.innerHTML = "";
    relayList.forEach(r => {
        const card = document.createElement('div');
        card.className = 'device-card';
        card.innerHTML = `
      <i class="fa-solid fa-bolt device-icon"></i>
      <h3>${r.name}</h3>
      <p>GPIO: ${r.gpio}</p>
      <button class="toggle-btn ${r.state ? 'on' : ''}" onclick="toggleRelay(${r.id})">
        ${r.state ? 'ON' : 'OFF'}
      </button>
      <i class="fa-solid fa-trash delete-icon" onclick="showDeleteDialog(${r.id})"></i>
    `;
        container.appendChild(card);
    });
}
function toggleRelay(id) {
    const relay = relayList.find(r => r.id === id);
    if (relay) {
        relay.state = !relay.state;
        const relayJSON = JSON.stringify({
            page: "device",
            value: {
                name: relay.name,
                status: relay.state ? "ON" : "OFF",
                gpio: relay.gpio
            }
        });
        Send_Data(relayJSON);
        renderRelays();
    }
}
function showDeleteDialog(id) {
    deleteTarget = id;
    document.getElementById('confirmDeleteDialog').style.display = 'flex';
}
function closeConfirmDelete() {
    document.getElementById('confirmDeleteDialog').style.display = 'none';
}
function confirmDelete() {
    relayList = relayList.filter(r => r.id !== deleteTarget);
    renderRelays();
    closeConfirmDelete();
}


// ==================== SETTINGS FORM (BỔ SUNG) ====================
document.getElementById("settingsForm").addEventListener("submit", function (e) {
    e.preventDefault();

    const ssid = document.getElementById("ssid").value.trim();
    const password = document.getElementById("password").value.trim();
    const token = document.getElementById("token").value.trim();
    const server = document.getElementById("server").value.trim();
    const port = document.getElementById("port").value.trim();

    const settingsJSON = JSON.stringify({
        page: "setting",
        value: {
            ssid: ssid,
            password: password,
            token: token,
            server: server,
            port: port
        }
    });

    Send_Data(settingsJSON);
    alert("✅ Cấu hình đã được gửi đến thiết bị!");
});

// TRONG script.js (Phần logic toggleTheme)
function toggleTheme() {  
    const root = document.documentElement;
    const isCurrentlyLight = root.classList.contains('light-mode');

    if (isCurrentlyLight) {
        root.classList.remove('light-mode');
        root.classList.add('dark-mode');
        localStorage.setItem('theme', 'dark-mode');
        console.log("Chế độ Tối đã bật");
    } else {
        root.classList.remove('dark-mode');
        root.classList.add('light-mode');
        localStorage.setItem('theme', 'light-mode');
        console.log("Chế độ Sáng đã bật");
    }

    // cập nhật màu chữ cho gauge mà không render lại
    if (typeof window.updateGaugeTextColor === 'function') {
        window.updateGaugeTextColor();
    }
}

document.addEventListener('DOMContentLoaded', () => {
    const switchInput = document.getElementById('darkModeSwitch');
    const savedMode = localStorage.getItem('theme');

    if (savedMode) {
        document.documentElement.classList.add(savedMode);
        if (switchInput) switchInput.checked = (savedMode === 'dark-mode');
    } else {
        document.documentElement.classList.add('light-mode');
        if (switchInput) switchInput.checked = false;
    }

    if (switchInput) {
        switchInput.addEventListener('change', toggleTheme);
    }

    // === XỬ LÝ LED 1 (BLINKY) ===
    const led1Switch = document.getElementById('led1Switch');
    const led1Label = document.getElementById('led1StatusLabel');
    const LED1_GPIO = 48; 

    if (led1Switch) {
        led1Switch.checked = true;
        led1Label.innerText = "AUTO (RUNNING)";

        led1Switch.addEventListener('change', function() {
            const isManualOn = this.checked;
            
            led1Label.innerText = isManualOn ? "AUTO" : "OFF";

            const cmd = {
                page: "device",
                value: {
                    gpio: LED1_GPIO,
                    status: isManualOn ? "ON" : "OFF"
                }
            };
            Send_Data(JSON.stringify(cmd));
        });
    } 

    // === XỬ LÝ NEOPIXEL ===
    const neoSwitch = document.getElementById('neoModeSwitch');
    const neoLabel = document.getElementById('neoModeLabel');
    const NEO_GPIO = 45; 

    if (neoSwitch) {
        neoSwitch.checked = true;
        neoLabel.innerText = "AUTO (COLOR)";

        neoSwitch.addEventListener('change', function() {
            const isManualMode = this.checked;

            neoLabel.innerText = isManualMode ? "AUTO" : "OFF";
            
            const cmd = {
                page: "device",
                value: {
                    gpio: NEO_GPIO,
                    status: isManualMode ? "ON" : "OFF"
                }
            };
            Send_Data(JSON.stringify(cmd));
        });
    }
});