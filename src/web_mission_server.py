#!/usr/bin/env python3
import os, json, threading, time, glob, re, asyncio
from datetime import datetime
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncpg
import aiohttp
from starlette.responses import StreamingResponse

app = FastAPI()
mission_status = {"running": False, "result_file": "", "progress": ""}
DB_POOL = None

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

class MissionStarter(Node):
    def __init__(self):
        super().__init__('mission_starter')
        self.publisher = self.create_publisher(String, '/mission_command', 10)
        self.sub = self.create_subscription(String, '/mission_status', self.status_callback, 10)
    def start_mission(self, area):
        msg = String()
        msg.data = json.dumps(area)
        self.publisher.publish(msg)
    def status_callback(self, msg):
        mission_status["progress"] = msg.data
        if "Миссия завершена" in msg.data:
            mission_status["running"] = False

rclpy.init()
mission_node = MissionStarter()

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
        await log_to_db("ERROR", "DB", f"Ошибка подключения: {e}")
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
        ''')

@app.on_event("startup")
async def startup():
    if await connect_db():
        await init_db()

# ---------- Главная страница с вкладками ----------
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
            canvas { border:1px solid black; background:#f0f0f0; cursor:crosshair; }
            .left-panel { width:50%; float:left; }
            .right-panel { width:50%; float:left; overflow:hidden; }
            iframe { width:100%; height:100%; border:none; }
            select { padding:5px; margin-right:10px; }
            #threejs-container { width:100%; height:80vh; }
            table { border-collapse: collapse; width:100%; }
            th, td { border:1px solid #ccc; padding:6px; text-align:left; }
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

        <div id="logs" class="tabcontent">
            <h2>Логи системы</h2>
            <div id="log-content">Загрузка...</div>
        </div>

        <div id="mission" class="tabcontent" style="display:block;">
            <div class="left-panel">
                <h2>Планирование миссии</h2>
                <canvas id="canvas" width="600" height="400"></canvas><br>
                <button onclick="startMission()">🚀 Запустить миссию</button>
                <div id="missionProgress" style="display:none;">
                    <p id="statusText">Статус: ожидание...</p>
                    <div class="progress" style="height:20px; background:#eee;"><div class="progress-bar" style="height:100%; background:#4CAF50; width:0%;"></div></div>
                </div>
            </div>
            <div class="right-panel">
                <h2>Карта дна</h2>
                <select id="mapSelector" onchange="loadMap()"></select>
                <button onclick="refreshMap()">🔄 Обновить</button>
                <iframe id="mapFrame" src="about:blank" style="height:80%;"></iframe>
                <div style="margin-top:20px;">
                    <h3>Экспорт данных</h3>
                    <button onclick="downloadCurrentMap()">📥 Скачать карту (PNG)</button>
                    <button onclick="downloadPoints()">📦 Скачать точки (XYZ)</button>
                </div>
            </div>
        </div>

        <div id="camera" class="tabcontent">
            <h2>Вид с камеры</h2>
            <img src="http://localhost:8080/stream?topic=/camera/front_cam/image_raw" 
                 style="width:100%; max-width:800px;" 
                 onerror="this.style.display='none'"/>
            <p id="cameraError" style="color:red; display:none;">Видео недоступно. Проверьте запуск web_video_server.</p>
        </div>

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

                function init3D() {
                    if (is3dInitialized) return;
                    const container = document.getElementById('threejs-container');
                    if (!container) return;
                    container.innerHTML = '';
                    scene3D = new THREE.Scene();
                    const width = container.clientWidth;
                    const height = container.clientHeight;
                    camera3D = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
                    camera3D.position.set(0, 0, 50);
                    renderer3D = new THREE.WebGLRenderer();
                    renderer3D.setSize(width, height);
                    container.appendChild(renderer3D.domElement);
                    controls3D = new OrbitControls(camera3D, renderer3D.domElement);
                    function animate() {
                        requestAnimationFrame(animate);
                        controls3D.update();
                        renderer3D.render(scene3D, camera3D);
                    }
                    animate();
                    is3dInitialized = true;
                }

                window.load3D = function () {
                    const sel = document.getElementById('glbSelector');
                    if (!sel || !sel.value) return;
                    const url = '/model/' + sel.value;
                    init3D();
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
                            init3D();
                            const sel = document.getElementById('glbSelector');
                            if (sel && sel.options.length) window.load3D();
                        }, 100);
                    });
                }
            </script>
        </div>

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
                draw();
            });
            canvas.addEventListener('mouseup', e => {
                drawing = false;
                const rc = canvas.getBoundingClientRect();
                const endX = e.clientX - rc.left, endY = e.clientY - rc.top;
                rect = { xmin: Math.min(startX,endX)*scale, ymin: Math.min(startY,endY)*scale,
                         xmax: Math.max(startX,endX)*scale, ymax: Math.max(startY,endY)*scale };
                draw();
            });
            function draw() {
                ctx.clearRect(0,0,canvas.width,canvas.height);
                if (drawing || rect) {
                    const x = Math.min(startX, currentX), y = Math.min(startY, currentY),
                          w = Math.abs(currentX-startX), h = Math.abs(currentY-startY);
                    ctx.strokeStyle='red'; ctx.lineWidth=2; ctx.strokeRect(x,y,w,h);
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
                if (data.status === 'ok') checkProgress();
            }

            function checkProgress() {
                fetch('/mission_progress').then(r=>r.json()).then(d=>{
                    document.getElementById('statusText').innerText = d.progress;
                    const pct = d.percent||0;
                    document.querySelector('.progress-bar').style.width = pct+'%';
                    if (!d.running && d.result_file) {
                        document.getElementById('missionProgress').style.display = 'none';
                        const ts = Date.now();
                        document.getElementById('mapFrame').src = '/viewer?file='+d.result_file+'&t='+ts;
                        populateSelectors();
                    } else if (d.running) setTimeout(checkProgress, 2000);
                });
            }

            function loadMap() {
                const sel = document.getElementById('mapSelector');
                if (sel.value) document.getElementById('mapFrame').src = '/viewer?file='+sel.value+'&t='+Date.now();
            }
            function refreshMap() { loadMap(); }

            async function populateSelectors() {
                const maps = await fetch('/list_maps').then(r=>r.json());
                const glbs = await fetch('/list_glbs').then(r=>r.json());
                ['mapSelector','glbSelector'].forEach(id => {
                    const sel = document.getElementById(id);
                    sel.innerHTML = '';
                    const items = id==='mapSelector' ? maps : glbs;
                    items.forEach(f => {
                        const opt = document.createElement('option');
                        opt.value = f; opt.textContent = f;
                        sel.appendChild(opt);
                    });
                    if (items.length) sel.value = items[items.length-1];
                });
                if (maps.length) loadMap();
                if (glbs.length) load3D();
            }

            async function loadDB() {
                const resp = await fetch('/detections');
                const data = await resp.json();
                let html = '<table><tr>' +
                    '<th>ID</th><th>Миссия</th><th>Время</th>' +
                    '<th>X</th><th>Y</th><th>Глубина (Z)</th>' +
                    '<th>Радиус</th><th>Ширина</th><th>Длина</th><th>Высота</th><th>Точек</th>' +
                    '</tr>';
                data.forEach(d => {
                    html += `<tr>
                        <td>${d.id}</td>
                        <td>${d.mission_id}</td>
                        <td>${d.timestamp?.substring(0, 19)}</td>
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
                document.getElementById('db-content').innerHTML = html || 'Нет данных.';
            }

            async function loadLogs() {
                const resp = await fetch('/logs');
                const data = await resp.json();
                let html = '<table><tr><th>Время</th><th>Уровень</th><th>Источник</th><th>Сообщение</th></tr>';
                data.forEach(d => {
                    html += `<tr>
                        <td>${d.timestamp?.substring(0, 19)}</td>
                        <td>${d.level}</td>
                        <td>${d.source}</td>
                        <td>${d.message}</td>
                    </tr>`;
                });
                html += '</table>';
                document.getElementById('log-content').innerHTML = html || 'Нет записей.';
            }

            function downloadCurrentMap() {
                const sel = document.getElementById('mapSelector');
                if (sel.value) {
                    window.open('/download/' + sel.value, '_blank');
                } else {
                    alert("Нет доступной карты.");
                }
            }

            async function downloadPoints() {
                const resp = await fetch('/download_last_xyz');
                if (resp.ok) {
                    const blob = await resp.blob();
                    const url = window.URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = resp.headers.get('content-disposition')?.split('filename=')[1] || 'points.xyz';
                    a.click();
                } else {
                    alert("Нет файлов точек.");
                }
            }

            document.querySelector("button.tablinks[onclick*='database']").addEventListener('click', loadDB);
            document.querySelector("button.tablinks[onclick*='logs']").addEventListener('click', loadLogs);
            populateSelectors();
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html)

@app.post("/start_mission")
async def start_mission(request: Request):
    data = await request.json()
    x_min = data["xmin"]; y_min = data["ymin"]; x_max = data["xmax"]; y_max = data["ymax"]
    mission_node.start_mission({"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max})
    mission_status["running"] = True
    await log_to_db("INFO", "Mission", f"Старт миссии: xmin={x_min}, ymin={y_min}, xmax={x_max}, ymax={y_max}")
    mission_status["progress"] = "Миссия запущена..."
    return {"status": "ok", "message": "Миссия запущена"}

@app.get("/mission_progress")
async def mission_progress_endpoint():
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

MAP_DIR = os.path.expanduser("~/deepscan_sonar_data/maps")

@app.get("/download/{filename}")
async def download(filename: str):
    return FileResponse(os.path.join(MAP_DIR, filename))

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

def serialize_row(row):
    d = dict(row)
    for key, value in d.items():
        if isinstance(value, datetime):
            d[key] = value.isoformat()
    return d

@app.get("/detections")
async def detections():
    if not DB_POOL:
        return JSONResponse([])
    async with DB_POOL.acquire() as conn:
        rows = await conn.fetch("SELECT * FROM detections ORDER BY id DESC LIMIT 100")
    return JSONResponse([serialize_row(r) for r in rows])

@app.get("/download_last_xyz")
async def download_last_xyz():
    data_dir = os.path.expanduser("~/deepscan_sonar_data")
    xyz_files = sorted(glob.glob(os.path.join(data_dir, "sonar_accurate_smart_*.xyz")))
    if not xyz_files:
        xyz_files = sorted(glob.glob(os.path.join(data_dir, "sonar_accurate_*.xyz")))
    if not xyz_files:
        xyz_files = sorted(glob.glob(os.path.join(data_dir, "sonar_smart_*.xyz")))
    if not xyz_files:
        return JSONResponse({"error": "Нет файлов"}, status_code=404)
    last_file = xyz_files[-1]
    return FileResponse(last_file, media_type="application/octet-stream", filename=os.path.basename(last_file))

def update_status(running, result_file=""):
    mission_status["running"] = running
    mission_status["result_file"] = result_file

import builtins
builtins.update_mission_status = update_status

@app.get("/logs")
async def logs():
    if not DB_POOL:
        return JSONResponse([])
    async with DB_POOL.acquire() as conn:
        rows = await conn.fetch("SELECT * FROM logs ORDER BY id DESC LIMIT 100")
    return JSONResponse([serialize_row(r) for r in rows])

def ros_spin():
    rclpy.spin(mission_node)

@app.get("/camera_stream")
async def camera_stream():
    async def stream():
        async with aiohttp.ClientSession() as session:
            async with session.get("http://localhost:8080/stream?topic=/camera/image_raw") as resp:
                async for chunk in resp.content.iter_any():
                    yield chunk
    return StreamingResponse(stream(), media_type="multipart/x-mixed-replace; boundary=--boundarydonotcross")

if __name__ == "__main__":
    import uvicorn
    threading.Thread(target=ros_spin, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)
