#!/usr/bin/env python3
import os, json, threading, time, glob, re, asyncio
from datetime import datetime
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
from starlette.responses import StreamingResponse
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState
import asyncpg
import aiohttp
import subprocess

# ---------- Глобальные переменные ----------
app = FastAPI()
mission_status = {"running": False, "result_file": "", "progress": "", "mode": "manual"}
DB_POOL = None
mission_start_time = None
collector_process = None
collision_detected = False

DATA_DIR = os.environ.get('DATA_DIR', os.path.expanduser('~/deepscan_sonar_data'))
MAP_DIR = os.path.join(DATA_DIR, 'maps')

# ---------- ROS2 ноды ----------
class MissionStarter(Node):
    def __init__(self):
        super().__init__('mission_starter')
        self.publisher = self.create_publisher(String, '/mission_command', 10)
        self.sub = self.create_subscription(String, '/mission_status', self.status_callback, 10)
        self.stop_publisher = self.create_publisher(String, '/mission_stop', 10)

    def start_mission(self, area):
        msg = String()
        msg.data = json.dumps(area)
        self.publisher.publish(msg)

    def stop_mission(self):
        msg = String()
        msg.data = "stop"
        self.stop_publisher.publish(msg)
    def status_callback(self, msg):
        mission_status["progress"] = msg.data
        if "Файл:" in msg.data:
            import re
            match = re.search(r'Файл:\s*(\S+)', msg.data)
            if match:
                mission_status["result_file"] = match.group(1)
                print(f"✅ Backend получил файл: {mission_status['result_file']}")
        if "Миссия завершена" in msg.data or "Миссия остановлена" in msg.data:
            mission_status["running"] = False

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('web_twist_publisher')
        self.pub = self.create_publisher(Twist, '/deepscan_boat/cmd_vel', 10)

    def send(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)


class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.sub = self.create_subscription(
            ContactsState,
            '/deepscan_boat/bumper',
            self.collision_callback,
            10
        )

    def collision_callback(self, msg):
        global collision_detected
        if len(msg.states) > 0 and not collision_detected:
            collision_detected = True
            # Пишем в БД
            asyncio.create_task(log_to_db("ERROR", "Collision", "⚠ Произошло столкновение! Авто-миссия прервана."))
            # Останавливаем катер
            twist_node.send(0.0, 0.0)
            # Прерываем авто-миссию
            mission_status["running"] = False
            mission_status["mode"] = "manual"
            mission_status["progress"] = "⚠ Столкновение! Авто-миссия прервана."
            # Через 2 секунды сбрасываем флаг, чтобы можно было продолжить
            threading.Timer(2.0, lambda: globals().update(collision_detected=False)).start()


# ---------- Единственный вызов rclpy.init() ----------
rclpy.init()
mission_node = MissionStarter()
twist_node = TwistPublisher()
collision_node = CollisionDetector()


# ---------- Работа с БД ----------
async def log_to_db(level, source, message):
    global DB_POOL
    if not DB_POOL:
        return
    try:
        async with DB_POOL.acquire() as conn:
            await conn.execute(
                "INSERT INTO logs (level, source, message) VALUES ($1, $2, $3)",
                level, source, message
            )
    except:
        pass


async def connect_db():
    global DB_POOL
    try:
        DB_POOL = await asyncpg.create_pool(
            user='deepscan', password='deepscan',
            database='deepscan_db', host='localhost'
        )
        await log_to_db("INFO", "DB", "Подключение к PostgreSQL успешно")
        print("✅ Подключено к PostgreSQL")
        return True
    except Exception as e:
        print(f"❌ Ошибка подключения к PostgreSQL: {e}")
        return False


async def init_db():
    async with DB_POOL.acquire() as conn:
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS detections (
                id SERIAL PRIMARY KEY,
                mission_id INT,
                timestamp TIMESTAMPTZ DEFAULT NOW(),
                world_x DOUBLE PRECISION,
                world_y DOUBLE PRECISION,
                world_z DOUBLE PRECISION,
                radius DOUBLE PRECISION,
                width DOUBLE PRECISION,
                length DOUBLE PRECISION,
                height DOUBLE PRECISION,
                point_count INT
            );
            CREATE TABLE IF NOT EXISTS missions (
                id SERIAL PRIMARY KEY,
                start_time TIMESTAMPTZ DEFAULT NOW(),
                end_time TIMESTAMPTZ,
                map_file TEXT
            );
            CREATE TABLE IF NOT EXISTS logs (
                id SERIAL PRIMARY KEY,
                timestamp TIMESTAMPTZ DEFAULT NOW(),
                level TEXT,
                source TEXT,
                message TEXT
            );
        ''')


@app.on_event("startup")
async def startup():
    if await connect_db():
        await init_db()


# ---------- Главная страница ----------
@app.get("/", response_class=HTMLResponse)
async def index():
    html = """
    <html>
    <head>
        <title>Deepscan Control</title>
        <style>
            body { margin:0; font-family: Arial; }
            .tab { overflow: hidden; background: #333; }
            .tab button { background: inherit; color: white; padding: 14px 20px; border: none; cursor: pointer; float: left; }
            .tab button:hover { background: #555; }
            .tab button.active { background: #4CAF50; }
            .tabcontent { display: none; padding: 20px; height: calc(100vh - 60px); }
            table { border-collapse: collapse; width:100%; }
            th, td { border:1px solid #ccc; padding:6px; text-align:left; }
            select { padding:5px; margin-right:10px; }
            canvas { border:1px solid black; background:#f0f0f0; }
        </style>
    </head>
    <body>
        <div class="tab">
            <button class="tablinks active" onclick="openTab(event, 'mission')">Миссия</button>
            <button class="tablinks" onclick="openTab(event, '3dview')">3D карта</button>
            <button class="tablinks" onclick="openTab(event, 'camera')">Камера</button>
            <button class="tablinks" onclick="openTab(event, 'logs')">Логи</button>
            <button class="tablinks" onclick="openTab(event, 'database')">База данных</button>
        </div>

        <!-- ========== ВКЛАДКА МИССИЯ ========== -->
        <div id="mission" class="tabcontent" style="display:block;">
            <div style="display:flex; height:85vh; gap:10px;">
                <!-- ЛЕВАЯ ПАНЕЛЬ: камера + холст -->
                <div style="flex:1; padding:10px; overflow-y:auto;">
                    <h2>Вид с камеры</h2>
                    <img src="http://localhost:8080/stream?topic=/camera/front_cam/image_raw"
                         style="width:100%; max-height:35vh; border:1px solid #333;" />

                    <h2>Автономная миссия</h2>
                    <canvas id="canvas" width="400" height="300" style="cursor:crosshair;"></canvas><br>
                    <button onclick="startMission()" style="background:#2196F3;color:white;padding:10px;margin-top:8px;">🚀 Запустить авто-миссию</button>
                    <button onclick="stopAutoMission()" style="background:orange;color:white;padding:10px;margin-top:5px;">⏸ Остановить авто-миссию</button>
                    <div id="missionProgress" style="display:none; margin-top:10px;">
                        <p id="statusText">Статус: ожидание...</p>
                        <div style="height:15px; background:#eee; border-radius:5px;">
                            <div class="progress-bar" style="height:100%; background:#4CAF50; width:0%; border-radius:5px;"></div>
                        </div>
                    </div>
                </div>

                <!-- ПРАВАЯ ПАНЕЛЬ: ручное управление + статус + карта -->
                <div style="flex:1; padding:10px; border-left:2px solid #ccc; overflow-y:auto;">
                    <h2>Ручное управление</h2>
                    <p id="modeIndicator" style="font-weight:bold; color:green;">Режим: ручной</p>
                    <button onclick="startManual()" style="background:green;color:white;padding:10px;width:100%;font-size:14px;">▶ Начать запись миссии</button>
                    <br><br>
                    <button onclick="stopManual()" style="background:red;color:white;padding:10px;width:100%;font-size:14px;">⏹ Закончить запись</button>
                    <br><br>
                    <canvas id="joystick" width="200" height="200" style="border:1px solid black; border-radius:50%; background:#eee;"></canvas>
                    <p id="joyStatus">Ожидание...</p>

                    <div style="margin-top:15px; padding:10px; background:#f5f5f5; border-radius:8px;">
                        <h3>Статус миссии</h3>
                        <p>Запись: <b id="recStatus">остановлена</b></p>
                        <p>Точек: <b id="pointCount">0</b></p>
                        <p>Время: <b id="duration">0 с</b></p>
                    </div>

                    <div style="margin-top:15px;">
                        <h3>Карта дна</h3>
                        <select id="mapSelector" onchange="loadMap()"></select>
                        <button onclick="refreshMap()">🔄</button>
                        <iframe id="mapFrame" src="about:blank" style="width:100%; height:250px; border:1px solid #333; margin-top:5px;"></iframe>
                    </div>

                    <div style="margin-top:15px;">
                        <h3>Экспорт</h3>
                        <button onclick="downloadCurrentMap()">📥 PNG</button>
                        <button onclick="downloadPoints()">📦 XYZ</button>
                    </div>
                </div>
            </div>
        </div>

        <!-- ========== ВКЛАДКА 3D ========== -->
        <div id="3dview" class="tabcontent">
            <h2>3D визуализация</h2>
            <select id="glbSelector" onchange="load3D()"></select>
            <button onclick="refresh3D()">🔄 Обновить</button>
            <div id="threejs-container" style="width:100%; height:80vh;"></div>

            <script type="importmap">
                {
                    "imports": {
                        "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
                        "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
                    }
                }
            </script>
            <script type="module">
                import * as THREE from 'three';
                import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
                import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

                let scene3D, camera3D, renderer3D, controls3D;
                let is3dInitialized = false;

                window.init3D = function() {
                    if (is3dInitialized) return;
                    const container = document.getElementById('threejs-container');
                    if (!container) return;
                    container.innerHTML = '';
                    scene3D = new THREE.Scene();
                    camera3D = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 1000);
                    camera3D.position.set(0, 0, 50);
                    renderer3D = new THREE.WebGLRenderer();
                    renderer3D.setSize(container.clientWidth, container.clientHeight);
                    container.appendChild(renderer3D.domElement);
                    controls3D = new OrbitControls(camera3D, renderer3D.domElement);
                    function animate() {
                        requestAnimationFrame(animate);
                        controls3D.update();
                        renderer3D.render(scene3D, camera3D);
                    }
                    animate();
                    is3dInitialized = true;
                };

                window.load3D = function () {
                    const sel = document.getElementById('glbSelector');
                    if (!sel || !sel.value) return;
                    const url = '/model/' + sel.value;
                    window.init3D();
                    const loader = new GLTFLoader();
                    loader.load(url, (gltf) => {
                        while (scene3D.children.length > 0) scene3D.remove(scene3D.children[0]);
                        scene3D.add(gltf.scene);
                        gltf.scene.traverse((child) => {
                            if (child.isPoints) child.material.size = 2.0;
                        });
                    });
                };

                window.refresh3D = function () {
                    const sel = document.getElementById('glbSelector');
                    if (sel && sel.value) window.load3D();
                };

                const tab3d = document.querySelector("button.tablinks[onclick*='3dview']");
                if (tab3d) {
                    tab3d.addEventListener('click', () => {
                        setTimeout(() => {
                            window.init3D();
                            const sel = document.getElementById('glbSelector');
                            if (sel && sel.options.length) window.load3D();
                        }, 100);
                    });
                }
            </script>
        </div>

        <!-- ========== ВКЛАДКА КАМЕРА ========== -->
        <div id="camera" class="tabcontent">
            <h2>Вид с камеры</h2>
            <img src="http://localhost:8080/stream?topic=/camera/front_cam/image_raw"
                 style="width:100%; max-width:1200px;" />
        </div>

        <!-- ========== ВКЛАДКА ЛОГИ ========== -->
        <div id="logs" class="tabcontent">
            <h2>Логи системы</h2>
            <div id="log-content">Загрузка...</div>
        </div>

        <!-- ========== ВКЛАДКА БД ========== -->
        <div id="database" class="tabcontent">
            <h2>Обнаруженные объекты (из БД)</h2>
            <div id="db-content">Загрузка...</div>
        </div>

        <script>
            window.openTab = function(evt, tabName) {
                document.querySelectorAll('.tabcontent').forEach(el => el.style.display = 'none');
                document.querySelectorAll('.tablinks').forEach(el => el.classList.remove('active'));
                document.getElementById(tabName).style.display = 'block';
                evt.currentTarget.classList.add('active');
            };

            // ================== ХОЛСТ АВТОНОМНОЙ МИССИИ ==================
            const canvas = document.getElementById('canvas');
            const ctx = canvas.getContext('2d');
            let drawing = false, startX, startY, currentX, currentY, rect = null, scale = 0.1;

            canvas.addEventListener('mousedown', e => {
                drawing = true;
                const rc = canvas.getBoundingClientRect();
                startX = e.clientX - rc.left; startY = e.clientY - rc.top;
            });
            canvas.addEventListener('mousemove', e => {
                if (!drawing) return;
                const rc = canvas.getBoundingClientRect();
                currentX = e.clientX - rc.left; currentY = e.clientY - rc.top;
                drawRect();
            });
            canvas.addEventListener('mouseup', e => {
                drawing = false;
                const rc = canvas.getBoundingClientRect();
                const endX = e.clientX - rc.left, endY = e.clientY - rc.top;
                rect = { xmin: Math.min(startX,endX)*scale, ymin: Math.min(startY,endY)*scale,
                         xmax: Math.max(startX,endX)*scale, ymax: Math.max(startY,endY)*scale };
                drawRect();
            });

            function drawRect() {
                ctx.clearRect(0,0,canvas.width,canvas.height);
                if (drawing || rect) {
                    const x = Math.min(startX, currentX);
                    const y = Math.min(startY, currentY);
                    const w = Math.abs(currentX - startX);
                    const h = Math.abs(currentY - startY);
                    ctx.strokeStyle = 'red';
                    ctx.lineWidth = 2;
                    ctx.strokeRect(x, y, w, h);
                }
            }

            async function startMission() {
                if (!rect) { alert("Нарисуйте прямоугольник"); return; }
                const resp = await fetch('/start_mission', {
                    method:'POST', headers:{'Content-Type':'application/json'},
                    body: JSON.stringify(rect)
                });
                const data = await resp.json();
                document.getElementById('missionProgress').style.display = 'block';
                document.getElementById('statusText').innerText = data.message;
                document.getElementById('modeIndicator').innerText = 'Режим: авто';
                document.getElementById('modeIndicator').style.color = 'orange';
                if (data.status === 'ok') checkProgress();
            }

            async function stopAutoMission() {
                const resp = await fetch('/stop_auto_mission', {method:'POST'});
                const data = await resp.json();
                alert(data.message);
                document.getElementById('missionProgress').style.display = 'none';
                document.getElementById('modeIndicator').innerText = 'Режим: ручной';
                document.getElementById('modeIndicator').style.color = 'green';
            }

            function checkProgress() {
                fetch('/mission_progress').then(r=>r.json()).then(d=>{
                    document.getElementById('statusText').innerText = d.progress;
                    const pct = d.percent || 0;
                    document.querySelector('.progress-bar').style.width = pct + '%';
                    if (d.progress && d.progress.includes('Столкновение')) {
                        document.getElementById('modeIndicator').innerText = 'Режим: ручной (столкновение)';
                        document.getElementById('modeIndicator').style.color = 'red';
                        document.getElementById('missionProgress').style.display = 'none';
                    }
                    if (!d.running && d.result_file) {
                        document.getElementById('missionProgress').style.display = 'none';
                        document.getElementById('mapFrame').src = '/viewer?file=' + d.result_file + '&t=' + Date.now();
                        populateSelectors();
                    } else if (d.running) {
                        setTimeout(checkProgress, 2000);
                    }
                });
            }

            // ================== SELECTORS ==================
            async function populateSelectors() {
                try {
                    const maps = await fetch('/list_maps').then(r => r.json());
                    const glbs = await fetch('/list_glbs').then(r => r.json());
                    ['mapSelector', 'glbSelector'].forEach(id => {
                        const sel = document.getElementById(id);
                        if (!sel) return;
                        sel.innerHTML = '';
                        const items = id === 'mapSelector' ? maps : glbs;
                        items.forEach(f => {
                            const opt = document.createElement('option');
                            opt.value = f; opt.textContent = f;
                            sel.appendChild(opt);
                        });
                        if (items.length) sel.value = items[items.length - 1];
                    });
                    if (maps.length) loadMap();
                    if (glbs.length && window.load3D) window.load3D();
                } catch (e) { console.error(e); }
            }

            function loadMap() {
                const sel = document.getElementById('mapSelector');
                if (sel && sel.value) {
                    document.getElementById('mapFrame').src = '/viewer?file=' + sel.value + '&t=' + Date.now();
                }
            }
            function refreshMap() { loadMap(); }

            function downloadCurrentMap() {
                const sel = document.getElementById('mapSelector');
                if (sel && sel.value) window.open('/download/' + sel.value, '_blank');
                else alert("Нет доступной карты.");
            }

            async function downloadPoints() {
                const resp = await fetch('/download_last_xyz');
                if (resp.ok) {
                    const blob = await resp.blob();
                    const url = window.URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = 'points.xyz';
                    a.click();
                } else alert("Нет файлов точек.");
            }

            // ================== РУЧНОЕ УПРАВЛЕНИЕ ==================
            async function startManual() {
                const resp = await fetch('/start_manual_mission', {method:'POST'});
                const data = await resp.json();
                alert(data.message);
            }
	    async function stopManual() {
    const resp = await fetch('/stop_manual_mission', {method: 'POST'});
    const data = await resp.json();
    alert(data.message);
    // Ждём 5 секунд, чтобы mission успел обработать данные
    setTimeout(() => {
        // Принудительно запрашиваем статус
        fetch('/mission_progress').then(r=>r.json()).then(d=>{
            if (d.result_file) {
                document.getElementById('mapFrame').src = '/viewer?file=' + d.result_file + '&t=' + Date.now();
                populateSelectors();
            }
        });
    }, 5000);
}
            async function updateStatus() {
                try {
                    const resp = await fetch('/manual_status');
                    const data = await resp.json();
                    document.getElementById('recStatus').innerText = data.recording ? 'идёт' : 'остановлена';
                    document.getElementById('pointCount').innerText = data.points;
                    document.getElementById('duration').innerText = data.duration + ' с';
                } catch (e) {}
            }
            setInterval(updateStatus, 2000);

            // ================== БД И ЛОГИ ==================
            async function loadDB() {
                try {
                    const resp = await fetch('/detections');
                    const data = await resp.json();
                    let html = '<table><tr><th>ID</th><th>Миссия</th><th>Время</th>' +
                        '<th>X</th><th>Y</th><th>Z</th><th>Радиус</th>' +
                        '<th>Ширина</th><th>Длина</th><th>Высота</th><th>Точек</th></tr>';
                    data.forEach(d => {
                        html += `<tr>
                            <td>${d.id}</td>
                            <td>${d.mission_id}</td>
                            <td>${d.timestamp ? d.timestamp.substring(0,19) : ''}</td>
                            <td>${d.world_x?.toFixed(2)}</td>
                            <td>${d.world_y?.toFixed(2)}</td>
                            <td>${d.world_z?.toFixed(2)}</td>
                            <td>${d.radius?.toFixed(2)}</td>
                            <td>${d.width?.toFixed(2)}</td>
                            <td>${d.length?.toFixed(2)}</td>
                            <td>${d.height?.toFixed(2)}</td>
                            <td>${d.point_count}</td>
                        </tr>`;
                    });
                    html += '</table>';
                    document.getElementById('db-content').innerHTML = html;
                } catch (e) {
                    document.getElementById('db-content').innerText = 'Ошибка загрузки: ' + e;
                }
            }

            async function loadLogs() {
                try {
                    const resp = await fetch('/logs');
                    const data = await resp.json();
                    let html = '<table><tr><th>Время</th><th>Уровень</th><th>Источник</th><th>Сообщение</th></tr>';
                    data.forEach(d => {
                        const color = d.level === 'ERROR' ? 'red' : 'black';
                        html += `<tr>
                            <td>${d.timestamp ? d.timestamp.substring(0,19) : ''}</td>
                            <td style="color:${color}; font-weight:bold;">${d.level}</td>
                            <td>${d.source}</td>
                            <td>${d.message}</td>
                        </tr>`;
                    });
                    html += '</table>';
                    document.getElementById('log-content').innerHTML = html;
                } catch (e) {
                    document.getElementById('log-content').innerText = 'Ошибка загрузки: ' + e;
                }
            }

            document.querySelector("button.tablinks[onclick*='database']").addEventListener('click', loadDB);
            document.querySelector("button.tablinks[onclick*='logs']").addEventListener('click', loadLogs);

            // ================== ДЖОЙСТИК ==================
            const joyCanvas = document.getElementById('joystick');
            const joyCtx = joyCanvas.getContext('2d');
            let joyActive = false;
            let joyX = 100, joyY = 100;
            const joyRadius = 80;
            const knobRadius = 20;

            function drawJoystick() {
                joyCtx.clearRect(0, 0, 200, 200);
                joyCtx.beginPath();
                joyCtx.arc(100, 100, joyRadius, 0, 2 * Math.PI);
                joyCtx.strokeStyle = '#333';
                joyCtx.lineWidth = 3;
                joyCtx.stroke();
                joyCtx.beginPath();
                joyCtx.arc(joyX, joyY, knobRadius, 0, 2 * Math.PI);
                joyCtx.fillStyle = '#4CAF50';
                joyCtx.fill();
            }
            drawJoystick();

            joyCanvas.addEventListener('mousedown', e => { joyActive = true; moveJoy(e); });
            joyCanvas.addEventListener('mousemove', e => { if (joyActive) moveJoy(e); });
            joyCanvas.addEventListener('mouseup', () => { joyActive = false; joyX = 100; joyY = 100; drawJoystick(); sendControl(0, 0); });
            joyCanvas.addEventListener('mouseleave', () => { joyActive = false; joyX = 100; joyY = 100; drawJoystick(); sendControl(0, 0); });
            joyCanvas.addEventListener('touchstart', e => { joyActive = true; moveJoy(e.touches[0]); e.preventDefault(); });
            joyCanvas.addEventListener('touchmove', e => { if (joyActive) moveJoy(e.touches[0]); e.preventDefault(); });
            joyCanvas.addEventListener('touchend', () => { joyActive = false; joyX = 100; joyY = 100; drawJoystick(); sendControl(0, 0); });

            function moveJoy(e) {
                const rect = joyCanvas.getBoundingClientRect();
                let x = e.clientX - rect.left;
                let y = e.clientY - rect.top;
                const dx = x - 100, dy = y - 100;
                const dist = Math.sqrt(dx*dx + dy*dy);
                if (dist > joyRadius) {
                    x = 100 + dx / dist * joyRadius;
                    y = 100 + dy / dist * joyRadius;
                }
                joyX = x; joyY = y;
                drawJoystick();
                const linear_x = -(y - 100) / joyRadius;
                const angular_z = -(x - 100) / joyRadius;
                sendControl(linear_x, angular_z);
                document.getElementById('joyStatus').innerText = `linear=${linear_x.toFixed(2)}, angular=${angular_z.toFixed(2)}`;
            }

            async function sendControl(linear_x, angular_z) {
                const resp = await fetch('/manual_control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({linear_x: linear_x, angular_z: angular_z})
                });
                const data = await resp.json();
                if (data.status === 'error') {
                    document.getElementById('joyStatus').innerText = '⚠ ' + data.message;
                }
            }

            // ================== ИНИЦИАЛИЗАЦИЯ ==================
            populateSelectors();
            updateStatus();
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html)


# ---------- API ----------
@app.post("/start_mission")
async def start_mission(request: Request):
    data = await request.json()
    x_min = data["xmin"]; y_min = data["ymin"]; x_max = data["xmax"]; y_max = data["ymax"]
    mission_node.start_mission({"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max})
    mission_status["running"] = True
    mission_status["mode"] = "auto"
    mission_status["progress"] = "Миссия запущена..."
    await log_to_db("INFO", "Mission", f"Старт авто-миссии: {x_min},{y_min} -> {x_max},{y_max}")
    return {"status": "ok", "message": "Миссия запущена"}


@app.post("/stop_auto_mission")
async def stop_auto_mission():
    mission_status["running"] = False
    mission_status["mode"] = "manual"
    mission_status["progress"] = "Авто-миссия остановлена оператором"
    mission_node.stop_mission()   # ← публикуем в ROS2
    twist_node.send(0.0, 0.0)
    await log_to_db("INFO", "Mission", "Авто-миссия остановлена оператором")
    return {"status": "ok", "message": "Авто-миссия остановлена"}


@app.get("/mission_progress")
async def mission_progress_endpoint():
    # Если result_file пустой — берём последнюю карту из папки
    if not mission_status.get("result_file"):
        maps = sorted(glob.glob(os.path.join(MAP_DIR, "seabed_2d_ai_*.png")))
        if maps:
            mission_status["result_file"] = os.path.basename(maps[-1])
    return {
        "running": mission_status["running"],
        "progress": mission_status["progress"],
        "percent": extract_percent(mission_status["progress"]),
        "result_file": mission_status["result_file"]
    }

def extract_percent(progress_str):
    match = re.search(r'(\d+)%', progress_str)
    if match: return int(match.group(1))
    if "завершена" in progress_str.lower(): return 100
    return 0


@app.post("/manual_control")
async def manual_control(request: Request):
    if mission_status.get("mode") == "auto":
        return {"status": "error", "message": "Авто-миссия активна, ручное управление заблокировано"}
    data = await request.json()
    twist_node.send(data.get("linear_x", 0.0), data.get("angular_z", 0.0))
    return {"status": "ok"}


@app.post("/start_manual_mission")
async def start_manual_mission():
    global collector_process, mission_start_time
    if collector_process is None or collector_process.poll() is not None:
        collector_process = subprocess.Popen(
            ["python3", os.path.expanduser("~/collect_sonar_accurate_smart.py")],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        mission_start_time = time.time()
        await log_to_db("INFO", "Mission", "Начата ручная запись миссии")
    return {"status": "ok", "message": "Запись начата"}


@app.post("/stop_manual_mission")
async def stop_manual_mission():
    global collector_process, mission_start_time
    if collector_process and collector_process.poll() is None:
        collector_process.send_signal(subprocess.signal.SIGINT)
        collector_process.wait()
        collector_process = None
        mission_start_time = None
        await log_to_db("INFO", "Mission", "Ручная запись миссии завершена")
        subprocess.run(["python3", os.path.expanduser("~/detect_on_map.py")])
    return {"status": "ok", "message": "Запись завершена, карта построена"}


@app.get("/manual_status")
async def manual_status():
    global collector_process, mission_start_time
    status = {
        "recording": collector_process is not None and collector_process.poll() is None,
        "points": 0,
        "duration": 0,
    }
    if status["recording"] and mission_start_time:
        status["duration"] = int(time.time() - mission_start_time)
    xyz_files = sorted(glob.glob(os.path.join(DATA_DIR, "sonar_accurate_smart_*.xyz")))
    if xyz_files:
        try:
            with open(xyz_files[-1], 'r') as f:
                status["points"] = sum(1 for _ in f) - 1
        except:
            pass
    return status


@app.get("/download/{filename}")
async def download(filename: str):
    file_path = os.path.join(MAP_DIR, filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Файл не найден")
    return FileResponse(file_path)


@app.get("/viewer", response_class=HTMLResponse)
async def viewer(file: str = "seabed_2d_ai.png"):
    safe_file = os.path.basename(file)
    file_path = os.path.join(MAP_DIR, safe_file)
    if not os.path.exists(file_path):
        return HTMLResponse(content="<h3>Файл не найден</h3>")
    return HTMLResponse(f"""<html><body style="margin:0; background:#000; display:flex; align-items:center; justify-content:center;">
        <img src="/download/{safe_file}?t={time.time()}" style="max-width:100%; max-height:100vh;">
    </body></html>""")


@app.get("/list_maps")
async def list_maps():
    files = sorted(glob.glob(os.path.join(MAP_DIR, "seabed_2d_ai_*.png")))
    return JSONResponse([os.path.basename(f) for f in files])


@app.get("/list_glbs")
async def list_glbs():
    files = sorted(glob.glob(os.path.join(MAP_DIR, "seabed_3d_*.glb")))
    return JSONResponse([os.path.basename(f) for f in files])


@app.get("/model/{filename}")
async def model(filename: str):
    file_path = os.path.join(MAP_DIR, filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Файл не найден")
    return FileResponse(file_path, media_type="model/gltf-binary")


@app.get("/detections")
async def detections():
    if not DB_POOL:
        return JSONResponse([])
    try:
        async with DB_POOL.acquire() as conn:
            rows = await conn.fetch("SELECT * FROM detections ORDER BY id DESC LIMIT 100")
        return JSONResponse([serialize_row(r) for r in rows])
    except Exception as e:
        print(f"Ошибка detections: {e}")
        return JSONResponse([])


@app.get("/logs")
async def logs():
    if not DB_POOL:
        return JSONResponse([])
    try:
        async with DB_POOL.acquire() as conn:
            rows = await conn.fetch("SELECT * FROM logs ORDER BY id DESC LIMIT 100")
        return JSONResponse([serialize_row(r) for r in rows])
    except Exception as e:
        print(f"Ошибка logs: {e}")
        return JSONResponse([])


@app.get("/download_last_xyz")
async def download_last_xyz():
    xyz_files = sorted(glob.glob(os.path.join(DATA_DIR, "sonar_accurate_smart_*.xyz")))
    if not xyz_files:
        xyz_files = sorted(glob.glob(os.path.join(DATA_DIR, "sonar_accurate_*.xyz")))
    if not xyz_files:
        xyz_files = sorted(glob.glob(os.path.join(DATA_DIR, "sonar_smart_*.xyz")))
    if not xyz_files:
        return JSONResponse({"error": "Нет файлов"}, status_code=404)
    last_file = xyz_files[-1]
    return FileResponse(last_file, media_type="application/octet-stream", filename=os.path.basename(last_file))


def serialize_row(row):
    d = dict(row)
    for key, value in d.items():
        if isinstance(value, datetime):
            d[key] = value.isoformat()
    return d


def update_status(running, result_file=""):
    mission_status["running"] = running
    mission_status["result_file"] = result_file


import builtins
builtins.update_mission_status = update_status


def ros_spin():
    executor = MultiThreadedExecutor()
    executor.add_node(mission_node)
    executor.add_node(twist_node)
    executor.add_node(collision_node)
    executor.spin()


if __name__ == "__main__":
    import uvicorn
    threading.Thread(target=ros_spin, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)
