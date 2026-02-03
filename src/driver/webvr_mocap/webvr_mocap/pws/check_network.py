import socket
import subprocess
import urllib.error
import urllib.request

try:
    from .common import JETSON_HOST, jetson_ssh_python_exec
except ImportError:
    from common import JETSON_HOST, jetson_ssh_python_exec

URL_TEST = "https://www.baidu.com/"
JETSON_SSH_PORT = 22


def check_nuc_to_jetson_connection(host=JETSON_HOST, port=JETSON_SSH_PORT, timeout=2):
    """检查 NUC 是否可以连接到 Jetson 网络（TCP 可达性，默认检测 SSH 端口）。"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (socket.timeout, socket.error, OSError):
        return False


def check_local_network_connection(test_url=URL_TEST, timeout=2):
    try:
        urllib.request.urlopen(test_url, timeout=timeout)
        return True
    except (urllib.error.URLError, urllib.error.HTTPError, Exception):
        return False


def check_jetson_network_connection(test_url=URL_TEST, timeout=2):
    python_code = f"""
import urllib.request
import urllib.error
try:
    urllib.request.urlopen('{test_url}', timeout={timeout})
    print('SUCCESS')
except:
    print('FAILED')
"""
    result = jetson_ssh_python_exec(python_code=python_code)
    print(result)
    return result["returncode"] == 0 and "SUCCESS" in result["stdout"]
