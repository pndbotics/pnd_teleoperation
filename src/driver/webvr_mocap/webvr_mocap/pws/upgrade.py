import atexit
import logging
import os
import signal
import subprocess
import sys
import threading

try:
    from .check_network import (
        check_jetson_network_connection,
        check_local_network_connection,
        check_nuc_to_jetson_connection,
    )
    from .check_version import get_remote_package_version
    from .common import (
        APP_NAME,
        HIDE_PATH,
        JETSON_HOST,
        JETSON_USER,
        NUC_USER,
        PACKAGE_NAME,
        PACKAGE_VERSION_NAME,
        SSH_KEY,
        download_package_from_minio,
        jetson_ssh_exec,
        jetson_ssh_python_exec,
        start_synchronized_service,
        stop_remote_service,
    )
except ImportError:
    from check_network import (
        check_jetson_network_connection,
        check_local_network_connection,
        check_nuc_to_jetson_connection,
    )
    from check_version import get_remote_package_version

    from common import (
        APP_NAME,
        HIDE_PATH,
        JETSON_HOST,
        JETSON_USER,
        NUC_USER,
        PACKAGE_NAME,
        PACKAGE_VERSION_NAME,
        SSH_KEY,
        download_package_from_minio,
        jetson_ssh_exec,
        jetson_ssh_python_exec,
        start_synchronized_service,
        stop_remote_service,
    )

# 获取脚本所在目录（用于单独运行时读取 common.py）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


def check_jetson_package_version():
    cmd = f"cat {HIDE_PATH}/{APP_NAME}/{PACKAGE_VERSION_NAME}"
    result = jetson_ssh_exec(command=cmd)
    stdout = result.get("stdout")
    return (stdout or "").strip()


def download_package_and_transfer_to_jetson():
    download_package_from_minio()
    subprocess.run(
        [
            "scp",
            "-i",
            SSH_KEY,
            f"{HIDE_PATH}/{PACKAGE_NAME}",
            f"{JETSON_USER}@{JETSON_HOST}:{HIDE_PATH}",
        ]
    )
    os.remove(f"{HIDE_PATH}/{PACKAGE_NAME}")
    return True


def jetson_handle():
    with open(f"{SCRIPT_DIR}/common.py", "r") as f:
        common_code = f.read()
    python_code = f"""
{common_code}
import shutil

download_package_from_minio()

# 如果目标目录已存在，先清理（避免新旧文件混合）
target_dir = f"{HIDE_PATH}/{APP_NAME}"
if os.path.exists(target_dir):
    logger.info(f"清理现有目录: {{target_dir}}")
    shutil.rmtree(target_dir)

# 解压到目标目录
with tarfile.open(f"{HIDE_PATH}/{PACKAGE_NAME}", "r:gz") as tar:
    tar.extractall(f"{HIDE_PATH}")
os.remove(f"{HIDE_PATH}/{PACKAGE_NAME}")
"""
    jetson_ssh_python_exec(python_code=python_code, timeout=30)
    return True


def setup_jetson():
    nuc_to_jetson = check_nuc_to_jetson_connection(host=JETSON_HOST)
    if not nuc_to_jetson:
        logger.error(f"NUC 无法连接 Jetson {JETSON_HOST}，请检查网络或 Jetson 是否在线")
        return False

    local_network_connection = check_local_network_connection()
    jetson_network_connection = check_jetson_network_connection()
    jetson_package_version = check_jetson_package_version()
    remote_package_version = get_remote_package_version()
    logger.info(f"本地网络连接: {local_network_connection}")
    logger.info(f"Jetson网络连接: {jetson_network_connection}")
    logger.info(f"Jetson包版本: {jetson_package_version}")
    logger.info(f"远程包版本: {remote_package_version}")

    def compare_version(version1, version2):
        # 比较版本号大小，version1 < version2 返回 True，否则返回 False
        version1_list = version1.split(".")
        version2_list = version2.split(".")
        for i in range(len(version1_list)):
            if int(version1_list[i]) < int(version2_list[i]):
                return True
            elif int(version1_list[i]) > int(version2_list[i]):
                return False
        return False

    if jetson_package_version != "" and not compare_version(
        jetson_package_version, remote_package_version
    ):
        logger.info("Jetson包版本已是最新，不需要升级")
    else:
        if jetson_network_connection == True:
            logger.info("开始在 Jetson 上部署...")
            jetson_handle()
        elif local_network_connection == True:
            logger.info("开始下载并传输包到 Jetson...")
            download_package_and_transfer_to_jetson()
        else:
            logger.error("网络连接检查失败")
            return False
    logger.info("jetson pws 远程服务启动...")
    start_synchronized_service(
        command=f"cd {HIDE_PATH}/{APP_NAME} && ./bin/pnd-webrtc-server"
    )
    logger.info("jetson pws 远程服务启动成功")
    return True


def signal_handler(signum, frame):
    """信号处理函数"""
    logger.info("收到退出信号，正在清理...")
    stop_remote_service()
    sys.exit(0)


def upgrade_jetson():
    try:
        if setup_jetson():
            logger.info("Jetson包setup成功")
        else:
            logger.error("Jetson包setup失败")
            stop_remote_service()
            # sys.exit(1)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)
    except Exception as e:
        logger.error(f"程序异常: {e}")
        stop_remote_service()
        sys.exit(1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(stop_remote_service)
    upgrade_jetson()
    import time

    while True:
        logger.info("Jetson包运行中")
        time.sleep(1)
