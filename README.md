# Deepscan — Autonomous Seabed Monitoring System

Микросервисная система для мониторинга подводного дна с безэкипажным катером (БЭК). 
Состоит из 6 независимых сервисов в Docker.

## 🏗️ Архитектура

```
┌──────────┐  ┌──────────┐  ┌───────────┐  ┌──────────┐
│ gazebo   │  │collector │  │ processor │  │ mission  │
│(симуляц.)│→ │(сбор дан.)│→ │(обработка)│  │(автопилот)│
└──────────┘  └──────────┘  └───────────┘  └──────────┘
       ↑                                        ↑
       │             ┌──────────┐              │
       └─────────────│ backend  │──────────────┘
                     │(FastAPI) │
                     └────┬─────┘
                          │
                          ▼
                     ┌──────────┐
                     │    db    │
                     │(Postgres)│
                     └──────────┘
```

## 📋 Требования

- **Ubuntu 22.04** (обязательно, ROS2 Humble только под него)
- **Docker** и **Docker Compose v2**
- **NVIDIA GPU** (для Gazebo)
- **X11** для отображения окна Gazebo

## 🚀 Установка

### 1. Установить Docker

```bash
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
# Выйти и зайти обратно
```

### 2. Установить Docker Compose v2

```bash
sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo curl -SL https://github.com/docker/compose/releases/latest/download/docker-compose-linux-x86_64 \
  -o /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
```

### 3. Установить модели Gazebo

```bash
mkdir -p ~/.gazebo/models
cp -r models/* ~/.gazebo/models/
```

Модели: `boat_mesh`, `real_seabed`.

### 4. Клонировать репозиторий

```bash
git clone https://github.com/Eduard1338/deepscan.git
cd deepscan
```

### 5. Запустить

```bash
xhost +local:docker
xhost +si:localuser:root
docker compose up -d --build
```

Первый запуск займёт 5–10 минут (сборка образов).

### 6. Открыть веб-интерфейс

В браузере: **http://localhost:8000**

- **Миссия** — холст для автономной миссии, джойстик, камера, карта, экспорт.
- **3D карта** — просмотр облака точек.
- **Камера** — вид с катера.
- **Логи** — системные события.
- **База данных** — обнаруженные объекты.

## 📦 Сервисы

| Сервис | Роль | Порт |
|--------|------|------|
| `gazebo` | Симуляция мира, катера, сонара, камеры | 11345, 8080 |
| `collector` | Сбор данных сонара | — |
| `processor` | Обработка и детекция объектов | — |
| `mission` | Автономный обход территории | — |
| `backend` | Веб-интерфейс и API | 8000 |
| `db` | PostgreSQL | 5432 |

## 🔧 Управление

```bash
# Статус
docker compose ps

# Логи конкретного сервиса
docker compose logs -f backend

# Перезапуск одного сервиса
docker compose restart mission

# Остановка
docker compose down
```

## 📁 Структура

```
deepscan-prod/
├── docker-compose.yml
├── backend/
│   ├── Dockerfile
│   ├── requirements.txt
│   └── src/web_mission_server.py
├── collector/
│   └── src/collect_sonar_accurate_smart.py
├── processor/
│   └── src/detect_on_map.py
├── mission/
│   └── src/mission_executor.py
├── gazebo/
│   ├── Dockerfile
│   ├── entrypoint.sh
│   └── deepscan_ws/
└── db/init.sql
```

## ⚠️ Возможные проблемы

### Gazebo не открывается
```bash
xhost +local:docker
xhost +si:localuser:root
```

### Порт 8080 занят
Останови старый `web_video_server` или измени порт в `gazebo/entrypoint.sh`.

### Контейнер backend в Restarting
```bash
docker compose logs backend | tail -30
```
Скорее всего, ошибка в `web_mission_server.py`.

## 📝 Лицензия

MIT

