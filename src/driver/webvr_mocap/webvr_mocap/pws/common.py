import logging
import os
import shutil
import subprocess
import tarfile
import threading

import requests

logger = logging.getLogger(__name__)

remote_service_process = None

SSH_KEY = "/home/pnd-humanoid/.ssh/jetson_ed25519"
JETSON_HOST = "10.10.20.126"
JETSON_USER = "pnd-humanoid"
NUC_USER = "pnd-humanoid"
HIDE_PATH = "/home/pnd-humanoid/.pnd"
APP_NAME = "pnd-gst-webrtc"

# minio
MINIO_HOST = "prs-pas.pndbotics.com:9000"
PACKAGE_NAME = f"{APP_NAME}.tar.gz"
PACKAGE_VERSION_NAME = ".version"
REMOTE_PACKAGE_VERSION_NAME = "version.txt"


def jetson_ssh_python_exec(
    host=JETSON_HOST, user=JETSON_USER, ssh_key=SSH_KEY, python_code=None, timeout=3
):
    ssh_cmd = ["ssh"]
    if ssh_key:
        ssh_cmd.extend(["-i", ssh_key])
    ssh_cmd.extend(
        [
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "BatchMode=yes",
            f"{user}@{host}",
            "python3",
        ]
    )
    try:
        result = subprocess.run(
            ssh_cmd, input=python_code, capture_output=True, text=True, timeout=timeout
        )
        return {
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except Exception as e:
        print(f"command error: {e} {python_code}")
        return {"returncode": None, "stdout": None, "stderr": None}


def jetson_ssh_exec(
    host=JETSON_HOST, user=JETSON_USER, ssh_key=SSH_KEY, command=None, timeout=3
):
    ssh_cmd = ["ssh"]
    if ssh_key:
        ssh_cmd.extend(["-i", ssh_key])
    ssh_cmd.extend(
        [
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "BatchMode=yes",
            f"{user}@{host}",
            command,
        ]
    )
    try:
        result = subprocess.run(
            ssh_cmd, capture_output=True, text=True, timeout=timeout
        )
        return {
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except Exception as e:
        print(f"command error: {e} {command}")
        return {"returncode": None, "stdout": None, "stderr": None}


def remote_logger(pipe, prefix="[JETSON]"):
    """在后台线程中读取远程服务输出并直接输出到屏幕"""
    import re
    import sys

    try:
        # 使用 readline 按行读取，更简单可靠
        for line_bytes in iter(pipe.readline, b""):
            if not line_bytes:
                break
            try:
                # 解码 bytes 为字符串
                line_str = line_bytes.decode("utf-8", errors="replace")
                # 去掉 ANSI 转义序列
                ansi_escape = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
                line_str = ansi_escape.sub("", line_str)
                # 去掉所有 \r 字符（这是导致阶梯状缩进的主要原因）
                line_str = line_str.replace("\r", "")
                # 去掉末尾的换行符，由 print 统一添加
                clean_line = line_str.rstrip("\n")

                if clean_line:  # 只有非空行才输出
                    # 使用 print 输出，确保格式正确
                    print(f"{prefix} {clean_line}", flush=True)
            except Exception as e:
                logger.warning(f"处理远程输出失败: {e}")
    except Exception as e:
        logger.error(f"读取远程输出时出错: {e}")
    finally:
        pipe.close()


def stop_remote_service():
    """停止远程服务"""
    global remote_service_process
    if remote_service_process is None:
        return

    logger.info("正在停止 Jetson 上的服务...")
    try:
        # 先尝试终止 SSH 进程
        if remote_service_process.poll() is None:  # 进程还在运行
            remote_service_process.terminate()
            try:
                remote_service_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                remote_service_process.kill()

        # 通过 SSH 发送 kill 命令，确保远程进程被停止
        kill_cmd = "pkill -f 'pnd-webrtc-server' || true"
        ssh_kill_cmd = [
            "ssh",
            "-i",
            SSH_KEY,
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "LogLevel=ERROR",
            f"{JETSON_USER}@{JETSON_HOST}",
            kill_cmd,
        ]
        subprocess.run(
            ssh_kill_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        logger.info("Jetson 服务已停止")
    except Exception as e:
        logger.warning(f"停止 Jetson 服务时出错: {e}")
    finally:
        remote_service_process = None


def start_synchronized_service(command):
    """启动远程服务"""
    global remote_service_process
    ssh_cmd = [
        "ssh",
        "-i",
        SSH_KEY,
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "LogLevel=ERROR",
        f"{JETSON_USER}@{JETSON_HOST}",
        command,
    ]

    proc = subprocess.Popen(
        ssh_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=False,  # 保持 bytes 模式，使用默认缓冲
    )

    remote_service_process = proc

    log_thread = threading.Thread(
        target=remote_logger, args=(proc.stdout,), daemon=True
    )
    log_thread.start()

    return proc


def download_package_from_minio():
    if not os.path.exists(f"{HIDE_PATH}"):
        os.makedirs(f"{HIDE_PATH}")
    url = f"http://{MINIO_HOST}/{APP_NAME}/{PACKAGE_NAME}"
    response = requests.get(url)
    with open(f"{HIDE_PATH}/{PACKAGE_NAME}", "wb") as f:
        f.write(response.content)


def package_unpack():
    # 如果目标目录已存在，先清理（避免新旧文件混合）
    target_dir = f"{HIDE_PATH}/{APP_NAME}"
    if os.path.exists(target_dir):
        logger.info(f"清理现有目录: {{target_dir}}")
        shutil.rmtree(target_dir)

    # 解压到目标目录
    with tarfile.open(f"{HIDE_PATH}/{PACKAGE_NAME}", "r:gz") as tar:
        tar.extractall(f"{HIDE_PATH}")
    os.remove(f"{HIDE_PATH}/{PACKAGE_NAME}")
