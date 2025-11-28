# server.py
from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import csv
from datetime import datetime
import json
import asyncio
import socket
from typing import List, Any, Dict, Optional
from pathlib import Path

import hmac
import hashlib
import time
import httpx  # pip install httpx

app = FastAPI()

# Allow cross-origin (удобно для MATLAB/GUI с другой машины)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ================= CSV =================
# 👉 ТУТ можно поменять директорию логов
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

# Теперь лог: lp, уровни, режим, коррекции
CSV_HEADER = [
    "lp",
    "level1", "level2", "level3",
    "mode",
    "correction1", "correction2", "correction3"
]

current_csv_file: Optional[Path] = None
last_log_time: Optional[float] = None  # время последней записи в CSV (time.time)
lp_counter: int = 0                    # "символические секунды" от начала записи


def create_new_csv_file() -> Path:
    """
    Создаёт новый CSV-файл с датой/временем в названии
    и записывает заголовок. Имя вида:
    logs/sensor_log_20251126_153045.csv
    """
    global current_csv_file, last_log_time, lp_counter
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = LOG_DIR / f"sensor_log_{ts}.csv"
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
    current_csv_file = filename
    last_log_time = None  # чтобы первая строка записалась сразу
    lp_counter = 0        # сброс "символических секунд"
    print(f"[CSV] New log file created: {current_csv_file}")
    return current_csv_file


# ================= WebSocket =================
clients: List[WebSocket] = []
clients_lock = asyncio.Lock()

async def push_to_clients(message: dict):
    """Асинхронная рассылка JSON всем подключённым клиентам (ESP и др)."""
    text = json.dumps(message)
    to_remove = []
    async with clients_lock:
        print(f"[SERVER] Pushing to {len(clients)} clients: {text}")
        for ws in list(clients):
            try:
                await ws.send_text(text)
            except Exception as e:
                print("Push error -> mark for removal:", e)
                to_remove.append(ws)
        for ws in to_remove:
            try:
                clients.remove(ws)
            except ValueError:
                pass

# ================= Состояние =================
MODE_AUTO   = 0
MODE_RECORD = 1
MODE_TEST   = 2
MODE_MANUAL = 3
MODE_REMOTE = 4  # новый режим: ESP не трогает Tuya, управляет Python

state = {
    "mode": 0,
    "manualLevel": [300,300,300],   # абсолют 10..1000 или 0 (0 = off)
    "enableTriac": [True,True,True],
    "correction": [0,0,0],
    "targetLux": [1000,1000,1000],
    "Kp": 0.05,
    "Ki": 0.0,
    "Kd": 0.0,
    "s": [0,0,0],
    "level": [0,0,0]
}

# ================= Tuya (Python клиент) =================

TUYA_CLIENT_ID     = "p3hea9cxpe8wxegcdmyd"
TUYA_CLIENT_SECRET = "770933630c54493c847f0e4ca19ed7de"
TUYA_REGION_HOST   = "https://openapi.tuyaeu.com"

TRIAC_1 = "bfa302be3e853f564aaaoz"
TRIAC_2 = "bf0794d9f4981c9715zygg"
TRIAC_3 = "bfc0ad553a9d0a8c07tcov"


class TuyaClient:
    def __init__(self):
        self.access_token: Optional[str] = None
        self.expire_epoch_ms: int = 0
        self.time_offset_ms: int = 0
        self._lock = asyncio.Lock()

    async def _ensure_token(self):
        async with self._lock:
            now_ms = int(time.time() * 1000)
            if self.access_token and now_ms < self.expire_epoch_ms:
                return
            await self._request_token()

    async def _request_token(self):
        path = "/v1.0/token?grant_type=1"
        t_epoch_ms = int(time.time() * 1000)
        t = str(t_epoch_ms)
        nonce = ""
        empty_sha = (
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        )
        sign_str = f"GET\n{empty_sha}\n\n{path}"
        string_to_sign = TUYA_CLIENT_ID + t + nonce + sign_str
        sign = hmac.new(
            TUYA_CLIENT_SECRET.encode("utf-8"),
            string_to_sign.encode("utf-8"),
            hashlib.sha256,
        ).hexdigest().upper()

        headers = {
            "client_id": TUYA_CLIENT_ID,
            "sign": sign,
            "t": t,
            "sign_method": "HMAC-SHA256",
        }

        async with httpx.AsyncClient(timeout=10.0) as client:
            resp = await client.get(TUYA_REGION_HOST + path, headers=headers)
        data = resp.json()
        print("[TUYA PY] token response:", data)

        if "t" in data:
            server_t_ms = int(data["t"])
            local_ms = int(time.time() * 1000)
            self.time_offset_ms = server_t_ms - local_ms

        if not data.get("success"):
            raise RuntimeError(f"Tuya token request failed: {data}")

        self.access_token = data["result"]["access_token"]
        expire_time = int(data["result"]["expire_time"])  # seconds
        now_ms = int(time.time() * 1000)
        # отнимем 10 минут как запас
        self.expire_epoch_ms = now_ms + expire_time * 1000 - 600_000
        print(f"[TUYA PY] Token OK, expires in {expire_time} s")

    async def _build_headers(self, method: str, path: str, body_json: str) -> Dict[str,str]:
        await self._ensure_token()
        if self.access_token is None:
            raise RuntimeError("No Tuya access token")

        local_ms = int(time.time() * 1000)
        t_epoch_ms = local_ms + self.time_offset_ms
        t = str(t_epoch_ms)
        nonce = ""

        body_hash = hashlib.sha256(body_json.encode("utf-8")).hexdigest()
        sign_url = f"{method}\n{body_hash}\n\n{path}"
        string_to_sign = TUYA_CLIENT_ID + self.access_token + t + nonce + sign_url
        sign = hmac.new(
            TUYA_CLIENT_SECRET.encode("utf-8"),
            string_to_sign.encode("utf-8"),
            hashlib.sha256,
        ).hexdigest().upper()

        headers = {
            "client_id": TUYA_CLIENT_ID,
            "access_token": self.access_token,
            "sign": sign,
            "t": t,
            "sign_method": "HMAC-SHA256",
            "Content-Type": "application/json",
        }
        return headers

    async def send_device_commands(self, device_id: str, commands: list[dict]) -> dict:
        """Отправка команды Tuya /v1.0/devices/{device}/commands"""
        path = f"/v1.0/devices/{device_id}/commands"
        body_obj = {"commands": commands}
        body_json = json.dumps(body_obj, separators=(",", ":"))
        headers = await self._build_headers("POST", path, body_json)

        url = TUYA_REGION_HOST + path
        async with httpx.AsyncClient(timeout=10.0) as client:
            resp = await client.post(url, headers=headers, content=body_json.encode("utf-8"))
        data = resp.json()
        print(f"[TUYA PY] POST {url} ->", data)
        return data


tuya_client = TuyaClient()

def normalize_level(val: Any) -> int:
    """
    Точно как на ESP:
    - 0 или ниже -> -1 (off)
    - 1..100 -> 20..1000
    - >100 -> уже абсолют 0..1000
    """
    try:
        v = float(val)
    except Exception:
        return -1
    if v <= 0:
        return -1
    if v <= 100:
        out = 20 + (1000 - 20) * (v / 100.0)
        return max(20, min(1000, int(round(out))))
    return max(0, min(1000, int(round(v))))

def build_tuya_commands_for_level(level: int) -> list[dict]:
    """
    Собирает список команд для Tuya:
    - switch_led
    - work_mode=white
    - bright_value_v2, если не OFF
    """
    norm = normalize_level(level)
    if norm < 0:
        switch_on = False
        bright = 0
    else:
        switch_on = True
        bright = norm
        if bright > 1000:
            bright = 1000
        if 0 < bright < 10:
            bright = 10

    cmds = [
        {"code": "switch_led", "value": switch_on},
        {"code": "work_mode", "value": "white"},
    ]
    if switch_on:
        cmds.append({"code": "bright_value_v2", "value": bright})
    return cmds

async def tuya_switch_all(on: bool, level: int = 1000):
    """
    Включить/выключить все три устройства.
    Если on=True -> bright=level, если False -> OFF.
    """
    print(f"[TUYA PY] switch_all on={on} level={level}")
    if on:
        cmds = build_tuya_commands_for_level(level)
    else:
        cmds = build_tuya_commands_for_level(0)

    tasks = [
        tuya_client.send_device_commands(TRIAC_1, cmds),
        tuya_client.send_device_commands(TRIAC_2, cmds),
        tuya_client.send_device_commands(TRIAC_3, cmds),
    ]
    results = await asyncio.gather(*tasks, return_exceptions=True)
    for idx, res in enumerate(results):
        print(f"[TUYA PY] switch_all dev{idx} ->", res)

async def tuya_set_manual_levels(levels: list[int]):
    """
    Установить уровни для трёх каналов [l1,l2,l3] в REMOTE-режиме.
    """
    if len(levels) < 3:
        return
    cmds1 = build_tuya_commands_for_level(levels[0])
    cmds2 = build_tuya_commands_for_level(levels[1])
    cmds3 = build_tuya_commands_for_level(levels[2])
    tasks = [
        tuya_client.send_device_commands(TRIAC_1, cmds1),
        tuya_client.send_device_commands(TRIAC_2, cmds2),
        tuya_client.send_device_commands(TRIAC_3, cmds3),
    ]
    results = await asyncio.gather(*tasks, return_exceptions=True)
    print("[TUYA PY] manual levels ->", results)

async def tuya_set_single_triac(idx: int, level: int):
    dev = [TRIAC_1, TRIAC_2, TRIAC_3][idx]
    cmds = build_tuya_commands_for_level(level)
    res = await tuya_client.send_device_commands(dev, cmds)
    print(f"[TUYA PY] triac {idx} -> level={level} ->", res)


# ================= PIR на Python =================
pir_config = {
    "enabled": False,
    "on_sec": 15.0,
    "off_sec": 45.0,
    "level": 1000
}
pir_task: Optional[asyncio.Task] = None
pir_lock = asyncio.Lock()

async def pir_loop():
    print("[PIR PY] started")
    state_local = "on"
    try:
        while True:
            async with pir_lock:
                if not pir_config["enabled"]:
                    break
                on_sec = float(pir_config["on_sec"])
                off_sec = float(pir_config["off_sec"])
                level = int(pir_config["level"])
            if state_local == "on":
                await tuya_switch_all(True, level)
                await asyncio.sleep(on_sec)
                state_local = "off"
            else:
                await tuya_switch_all(False, 0)
                await asyncio.sleep(off_sec)
                state_local = "on"
    except asyncio.CancelledError:
        print("[PIR PY] cancelled")
    finally:
        print("[PIR PY] stopped")

async def start_pir(on_sec: float, off_sec: float, level: int):
    global pir_task
    async with pir_lock:
        pir_config["enabled"] = True
        pir_config["on_sec"] = on_sec
        pir_config["off_sec"] = off_sec
        pir_config["level"] = level
    if pir_task is None or pir_task.done():
        pir_task = asyncio.create_task(pir_loop())
    print(f"[PIR PY] configured: on={on_sec}s off={off_sec}s level={level}")

async def stop_pir():
    global pir_task
    async with pir_lock:
        pir_config["enabled"] = False
    if pir_task and not pir_task.done():
        pir_task.cancel()
    pir_task = None
    await tuya_switch_all(False, 0)
    print("[PIR PY] disabled and lights OFF")


# ================= UTILS =================
def pct_to_abs(val: Any) -> int:
    """
    Старая функция, оставляем для совместимости.
    """
    try:
        v = float(val)
    except Exception:
        return 0
    if v <= 0:
        return 0
    if v <= 100:
        out = round(10 + (1000 - 10) * (v / 100.0))
        out = max(0, min(1000, out))
        return int(out)
    return int(max(0, min(1000, round(v))))

def prepare_cmd_for_push(cmd: Dict) -> Dict:
    """
    Возвращает копию cmd для пуша в ESP (всё как раньше).
    NOTA: это именно для WS, а не для Tuya.
    """
    out: Dict = {}

    if "switch_all" in cmd:
        try:
            v = bool(cmd["switch_all"])
        except Exception:
            v = False
        if v:
            out["manual"] = [1000, 1000, 1000]
        else:
            out["manual"] = [0, 0, 0]
        return out

    for k, v in cmd.items():
        if k == "manual" and isinstance(v, list):
            mapped = []
            for vv in v[:3]:
                try:
                    fv = float(vv)
                except:
                    fv = 0.0
                if fv <= 0:
                    mapped.append(0)
                elif fv <= 100:
                    mapped.append(pct_to_abs(fv))
                else:
                    av = int(max(0, min(1000, round(fv))))
                    mapped.append(av)
            out["manual"] = mapped

        elif k == "level" and "triac" in cmd:
            out["triac"] = int(cmd.get("triac", 0))
            out["level"] = pct_to_abs(v)

        elif k == "targetLux":
            out["targetLux"] = v

        elif k in ("Kp", "Ki", "Kd"):
            out[k] = v

        elif k == "correction":
            out["correction"] = v

        elif k == "enable":
            out["enable"] = v

        elif k == "triac" and "level" not in cmd:
            out["triac"] = v

        else:
            out[k] = v
    return out


# ================= HTTP API =================
@app.post("/command")
async def send_command(cmd: dict):
    """
    Команды от MATLAB/клиента.
    - Обновляем state
    - В режиме REMOTE (mode=4) manual/switch_all/triac-level выполняем в Tuya (Python)
    - Всегда пушим в ESP по WS (чтобы он знал режим, targetLux и т.п.)
    """
    global state

    # --- режим ---
    if "mode" in cmd:
        try:
            m = int(cmd["mode"])
        except Exception:
            m = cmd["mode"]
        if isinstance(m, int):
            if m < 0:
                m = 0
            if m > 4:
                m = 4
            state["mode"] = m
        else:
            state["mode"] = m

        # КАЖДОЕ получение команды mode=MODE_RECORD -> новый CSV
        if state["mode"] == MODE_RECORD:
            create_new_csv_file()

        print(f"[SERVER] mode set to {state['mode']}")

    # --- обработка switch_all ---
    if "switch_all" in cmd:
        v = bool(cmd["switch_all"])
        if v:
            state["manualLevel"] = [1000, 1000, 1000]
        else:
            state["manualLevel"] = [0, 0, 0]
        print(f"[SERVER] switch_all: {v} -> manualLevel={state['manualLevel']}")

    # individual triac + level
    if "triac" in cmd and "level" in cmd:
        try:
            idx = int(cmd["triac"])
            lvl = cmd["level"]
            state["manualLevel"][idx] = pct_to_abs(lvl)
        except Exception as e:
            print("Error parsing triac/level:", e)

    # manual array
    if "manual" in cmd and isinstance(cmd["manual"], list) and len(cmd["manual"]) >= 3:
        try:
            new_manual = [pct_to_abs(x) for x in cmd["manual"][:3]]
            state["manualLevel"] = new_manual
        except Exception as e:
            print("Error parsing manual:", e)

    # enable single
    if "triac" in cmd and "enable" in cmd:
        try:
            idx = int(cmd["triac"])
            state["enableTriac"][idx] = bool(cmd["enable"])
        except Exception as e:
            print("Error parsing triac/enable:", e)

    # enable array
    if "enable" in cmd and isinstance(cmd["enable"], list) and len(cmd["enable"]) >= 3:
        try:
            state["enableTriac"] = [
                bool(cmd["enable"][0]),
                bool(cmd["enable"][1]),
                bool(cmd["enable"][2])
            ]
        except Exception as e:
            print("Error parsing enable array:", e)

    # corrections
    if "correction" in cmd and isinstance(cmd["correction"], list) and len(cmd["correction"]) >= 3:
        try:
            state["correction"] = [
                float(cmd["correction"][0]),
                float(cmd["correction"][1]),
                float(cmd["correction"][2])
            ]
        except Exception as e:
            print("Error parsing correction:", e)

    # targetLux
    if "targetLux" in cmd:
        try:
            t = cmd["targetLux"]
            if isinstance(t, list) and len(t) >= 3:
                state["targetLux"] = [float(t[0]), float(t[1]), float(t[2])]
            else:
                val = float(t)
                state["targetLux"] = [val, val, val]
        except Exception as e:
            print("Error parsing targetLux:", e)

    # PID params
    if "Kp" in cmd:
        try:
            state["Kp"] = float(cmd["Kp"])
        except:
            pass
    if "Ki" in cmd:
        try:
            state["Ki"] = float(cmd["Ki"])
        except:
            pass
    if "Kd" in cmd:
        try:
            state["Kd"] = float(cmd["Kd"])
        except:
            pass

    # --- PIR управление с Python ---
    # ожидаем в команде поля вида:
    # {"pir_enable": true/false, "pir_on": 15, "pir_off": 45, "pir_level": 1000}
    if "pir_enable" in cmd:
        enable = bool(cmd["pir_enable"])
        on_sec = float(cmd.get("pir_on", 15))
        off_sec = float(cmd.get("pir_off", 45))
        level = int(cmd.get("pir_level", 1000))
        if enable:
            # включаем REMOTE-режим, чтобы ESP не трогал Tuya
            state["mode"] = MODE_REMOTE
            # добавим mode в cmd, чтобы ESP тоже понял
            cmd["mode"] = MODE_REMOTE
            asyncio.create_task(start_pir(on_sec, off_sec, level))
        else:
            asyncio.create_task(stop_pir())

    print(f"[SERVER] /command received: {cmd} -> new state: {state}")

    # --- Tuya управление в REMOTE режиме ---
    if state["mode"] == MODE_REMOTE:
        try:
            if "switch_all" in cmd:
                v = bool(cmd["switch_all"])
                if v:
                    asyncio.create_task(tuya_switch_all(True, max(state["manualLevel"])))
                else:
                    asyncio.create_task(tuya_switch_all(False, 0))

            if "manual" in cmd:
                asyncio.create_task(tuya_set_manual_levels(state["manualLevel"]))

            if "triac" in cmd and "level" in cmd:
                try:
                    idx = int(cmd["triac"])
                    if 0 <= idx < 3:
                        lvl = state["manualLevel"][idx]
                        asyncio.create_task(tuya_set_single_triac(idx, lvl))
                except Exception as e:
                    print("REMOTE triac error:", e)
        except Exception as e:
            print("REMOTE Tuya control error:", e)

    # --- пушим на ESP по WS ---
    try:
        converted = prepare_cmd_for_push(cmd)
        to_push = converted if converted else cmd
        if isinstance(to_push, dict) and "manual" in to_push:
            try:
                to_push["manual"] = [int(x) for x in to_push["manual"]]
            except:
                pass
        asyncio.create_task(push_to_clients(to_push))
    except Exception as e:
        print("prepare/push error:", e)

    return {"status": "sent", "state": state}

@app.post("/stop_record")
async def stop_record():
    """
    Остановка записи:
    - переводим режим в AUTO
    - перестаём писать в текущий CSV
    """
    global current_csv_file, last_log_time, lp_counter, state
    state["mode"] = MODE_AUTO
    current_csv_file = None
    last_log_time = None
    lp_counter = 0
    print("[SERVER] stop_record: mode -> AUTO, logging stopped, lp reset")
    return {"status": "stopped", "state": state}

@app.get("/state")
async def get_state():
    """Возвращает текущее состояние."""
    return JSONResponse(content=state)

@app.post("/log")
async def receive_log(request: Request):
    """
    Логи от ESP32
    data = {"s":[..3], "level":[..3], "mode":0/1/2/3/4, "correction":[..3]?}
    """
    global current_csv_file, last_log_time, lp_counter

    data = await request.json()
    s = data.get("s",[0,0,0])
    level = data.get("level",[0,0,0])
    mode_val = int(data.get("mode", -1))

    correction_from_esp = data.get("correction", None)
    if correction_from_esp and isinstance(correction_from_esp, list) and len(correction_from_esp)>=3:
        correction_to_log = correction_from_esp
    else:
        correction_to_log = state["correction"] if mode_val==2 else [0,0,0]

    # Обновляем глобальный state (сенсоры и level) — для отображения/GUI
    try:
        if isinstance(s, list) and len(s) >= 3:
            state["s"] = [float(s[0]), float(s[1]), float(s[2])]
        if isinstance(level, list) and len(level) >= 3:
            state["level"] = [int(level[0]), int(level[1]), int(level[2])]
        if mode_val >= 0:
            state["mode"] = int(mode_val)
    except Exception as e:
        print("receive_log: failed to store state fields:", e)

    # Запись в CSV:
    # - только если есть текущий файл
    # - только в режиме RECORD
    # - не чаще, чем раз в 1 секунду
    if state.get("mode") == MODE_RECORD and current_csv_file is None:
        create_new_csv_file()

    if current_csv_file is not None and state.get("mode") == MODE_RECORD:
        now_t = time.time()
        if last_log_time is None or (now_t - last_log_time) >= 1.0:
            try:
                with open(current_csv_file, "a", newline="") as f:
                    writer = csv.writer(f)
                    try:
                        writer.writerow([
                            lp_counter,
                            level[0], level[1], level[2],
                            mode_val,
                            correction_to_log[0], correction_to_log[1], correction_to_log[2]
                        ])
                    except Exception as e:
                        try:
                            writer.writerow(
                                [lp_counter]
                                + list(level)[:3]
                                + [mode_val]
                                + list(correction_to_log)[:3]
                            )
                        except Exception as e2:
                            print("Failed to write CSV row:", e2)
                last_log_time = now_t
                lp_counter += 1
            except Exception as e:
                print("Failed to open CSV file for append:", e)

    return {"status":"ok"}

# ================= WebSocket =================
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    async with clients_lock:
        clients.append(websocket)
        print(f"[SERVER] New WS client connected. Total clients: {len(clients)}")
    # Отправим начальное состояние
    try:
        await websocket.send_text(json.dumps({"type":"state_init","state":state}))
    except Exception as e:
        print("Failed to send initial state:", e)

    try:
        while True:
            msg = await websocket.receive_text()
            print("[SERVER] Received from WS client:", msg)
    except Exception as e:
        print("[SERVER] WS client disconnected:", e)
        async with clients_lock:
            try:
                clients.remove(websocket)
            except ValueError:
                pass

# ================= HTML тест =================
@app.get("/")
def index():
    html = """
    <html>
        <body>
            <h1>ESP32 Server</h1>
            <p>POST logs to /log, push commands to /command</p>
            <p>GET current state at /state</p>
        </body>
    </html>
    """
    return HTMLResponse(content=html)

# ================= Helper: локальный IP =================
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

# ================= Run =================
if __name__ == "__main__":
    local_ip = get_local_ip()
    print(f"🌐 Local server IP: http://{local_ip}:5000")
    uvicorn.run(app, host="0.0.0.0", port=5000)
