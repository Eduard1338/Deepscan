import httpx
import pytest

BASE_URL = "http://localhost:8000"

def test_homepage():
    """Проверяет, что главная страница загружается и содержит правильный заголовок."""
    resp = httpx.get(f"{BASE_URL}/")
    assert resp.status_code == 200
    assert "text/html" in resp.headers["content-type"]
    assert "<title>Deepscan Control</title>" in resp.text

def test_start_mission():
    """Проверяет успешный запуск миссии через API."""
    payload = {"xmin": 0, "ymin": 0, "xmax": 10, "ymax": 10}
    resp = httpx.post(f"{BASE_URL}/start_mission", json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data["status"] == "ok"

def test_mission_progress():
    """Проверяет, что эндпоинт прогресса миссии возвращает корректную структуру."""
    resp = httpx.get(f"{BASE_URL}/mission_progress")
    assert resp.status_code == 200
    data = resp.json()
    for key in ["running", "progress", "percent", "result_file"]:
        assert key in data

def test_list_maps():
    """Проверяет, что список карт возвращается в виде массива."""
    resp = httpx.get(f"{BASE_URL}/list_maps")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)

def test_list_glbs():
    """Проверяет, что список 3D-моделей возвращается в виде массива."""
    resp = httpx.get(f"{BASE_URL}/list_glbs")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)

def test_detections():
    """Проверяет, что эндпоинт обнаружений возвращает массив (даже если пустой)."""
    resp = httpx.get(f"{BASE_URL}/detections")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)

def test_logs():
    """Проверяет, что эндпоинт логов возвращает массив."""
    resp = httpx.get(f"{BASE_URL}/logs")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)

def test_download_last_xyz():
    """Проверяет доступность эндпоинта скачивания последнего XYZ-файла."""
    resp = httpx.get(f"{BASE_URL}/download_last_xyz")
    assert resp.status_code in (200, 404)

def test_viewer():
    """Проверяет, что просмотрщик карт возвращает HTML даже для несуществующего файла."""
    resp = httpx.get(f"{BASE_URL}/viewer?file=nonexistent.png")
    assert resp.status_code == 200
    assert "Файл не найден" in resp.text

def test_model_endpoint():
    """Проверяет, что эндпоинт 3D-модели корректно возвращает 404 при отсутствии файла."""
    resp = httpx.get(f"{BASE_URL}/model/nonexistent.glb")
    assert resp.status_code == 404
