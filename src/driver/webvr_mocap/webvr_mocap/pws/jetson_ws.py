import asyncio
import json
import threading
import time
from contextlib import asynccontextmanager
from typing import Set

import uvicorn
import websockets
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidURI

try:
    from .common import JETSON_HOST, jetson_ssh_exec
    from .upgrade import upgrade_jetson
except ImportError:
    from upgrade import upgrade_jetson

    from common import JETSON_HOST, jetson_ssh_exec

PND_WEBRTC_SERVICE_NAME = "pnd-webrtc-server"
PND_WEBRTC_WS_PORT = 12079
WEBSOCKET_PORT = 12078  # 使用不同的端口避免冲突
CHECK_INTERVAL = 10  # 检查间隔（秒）
WS_CHECK_TIMEOUT = 3  # WebSocket 连接超时时间（秒）

# WebSocket 服务器相关变量
websocket_clients: Set[WebSocket] = set()
setup_status = {
    "command": "pws-state",
    "is_setup": False,
    "status": "checking",  # checking, success, failed
    "message": "",
    "package_version": "",
}
status_lock = threading.Lock()
app = None
event_loop = None  # 存储 FastAPI 的事件循环


async def check_websocket_service(
    host: str, port: int, timeout: float = WS_CHECK_TIMEOUT
) -> bool:
    """通过 WebSocket 连接检测服务是否启动"""
    ws_url = f"ws://{host}:{port}/ws"
    websocket = None
    try:
        # 使用 asyncio.wait_for 包装连接，设置超时
        websocket = await asyncio.wait_for(
            websockets.connect(ws_url, ping_interval=None), timeout=timeout
        )
        # 尝试接收初始消息
        try:
            message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
            # 如果成功接收到消息，说明服务正常运行
            return True
        except asyncio.TimeoutError:
            # 即使没有收到消息，连接成功也说明服务在运行
            return True
    except (
        ConnectionRefusedError,
        ConnectionClosed,
        InvalidURI,
        InvalidHandshake,
        OSError,
        asyncio.TimeoutError,
    ):
        # 连接失败，说明服务未启动或不可访问
        return False
    except Exception as e:
        # 其他异常，认为服务不可用
        print(f"WebSocket 检测异常: {e}")
        return False
    finally:
        # 确保关闭连接
        if websocket:
            try:
                await websocket.close()
            except Exception:
                pass


async def check_jetson_package_setup():
    """检查 Jetson 包是否 setup 成功"""
    try:
        # 尝试连接 WebSocket 服务
        is_running = await check_websocket_service(JETSON_HOST, PND_WEBRTC_WS_PORT)

        if is_running:
            return {
                "is_setup": True,
                "status": "success",
                "message": f"PND WebRTC服务已启动 (WebSocket: {JETSON_HOST}:{PND_WEBRTC_WS_PORT})",
            }
        else:
            return {
                "is_setup": False,
                "status": "failed",
                "message": f"PND WebRTC服务未启动 (无法连接到 {JETSON_HOST}:{PND_WEBRTC_WS_PORT})",
            }
    except Exception as e:
        print(f"检测服务状态时出错: {e}")
        return {"is_setup": False, "status": "failed", "message": f"检测服务状态失败: {str(e)}"}


def update_setup_status(new_status):
    """更新 setup 状态并广播给所有客户端"""
    global setup_status, event_loop
    with status_lock:
        old_status = setup_status.copy()
        setup_status = new_status

        # 如果状态改变，广播给所有客户端
        if (
            old_status["is_setup"] != new_status["is_setup"]
            or old_status["status"] != new_status["status"]
        ):
            if app is not None and event_loop is not None:
                try:
                    # 使用存储的事件循环来广播状态
                    asyncio.run_coroutine_threadsafe(broadcast_status(), event_loop)
                except Exception as e:
                    print(f"广播状态失败: {e}")


async def broadcast_status():
    """向所有连接的客户端广播 setup 状态"""
    if not websocket_clients:
        return

    with status_lock:
        status_message = json.dumps(setup_status, ensure_ascii=False)

    disconnected = set()

    for client in websocket_clients:
        try:
            await client.send_text(status_message)
        except Exception as e:
            print(f"发送状态到客户端失败: {e}")
            disconnected.add(client)

    # 移除断开的客户端
    websocket_clients.difference_update(disconnected)


async def check_status_periodically():
    """定期检查 setup 状态"""
    while True:
        try:
            new_status = await check_jetson_package_setup()
            update_setup_status(new_status)
        except Exception as e:
            print(f"检查状态时出错: {e}")

        await asyncio.sleep(CHECK_INTERVAL)


def create_fastapi_app():
    """创建 FastAPI 应用"""
    global app

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        """Lifespan 事件：启动与关闭"""
        # Startup
        global event_loop
        event_loop = asyncio.get_running_loop()
        task = asyncio.create_task(check_status_periodically())
        print("开始定期检查 Jetson 包 setup 状态...")

        yield

        # Shutdown
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    app = FastAPI(title="Jetson 包 Setup 状态 WebSocket 服务", lifespan=lifespan)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        """WebSocket 端点"""
        await websocket.accept()
        websocket_clients.add(websocket)
        print(f"新客户端连接，当前连接数: {len(websocket_clients)}")

        try:
            # 新客户端连接时，立即发送当前状态
            with status_lock:
                status_message = json.dumps(setup_status, ensure_ascii=False)
            await websocket.send_text(status_message)

            # 保持连接，等待客户端断开
            while True:
                try:
                    # 等待客户端消息（用于保持连接）
                    await websocket.receive_text()
                except WebSocketDisconnect:
                    break
        except Exception as e:
            print(f"WebSocket 连接错误: {e}")
        finally:
            websocket_clients.discard(websocket)
            print(f"客户端断开，当前连接数: {len(websocket_clients)}")

    @app.get("/status")
    async def get_status():
        """HTTP 接口：获取当前 setup 状态"""
        with status_lock:
            return JSONResponse(content=setup_status)

    return app


def run_fastapi_server():
    """在单独线程中运行 FastAPI 服务器"""
    global app
    if app is None:
        return
    try:
        uvicorn.run(app, host="0.0.0.0", port=WEBSOCKET_PORT, log_level="info")
    except Exception as e:
        print(f"FastAPI 服务器启动失败: {e}")


def pws():
    """主函数"""
    # 创建 FastAPI 应用
    create_fastapi_app()
    upgrade_jetson()

    if app is not None:
        print(f"启动 Jetson Setup 状态 WebSocket 服务...")
        print(f"WebSocket 地址: ws://0.0.0.0:{WEBSOCKET_PORT}/ws")
        print(f"HTTP 状态接口: http://0.0.0.0:{WEBSOCKET_PORT}/status")
        print(f"检查间隔: {CHECK_INTERVAL} 秒")

        # 运行服务器（阻塞）
        run_fastapi_server()
    else:
        print("FastAPI 应用未创建")


if __name__ == "__main__":
    pws()
