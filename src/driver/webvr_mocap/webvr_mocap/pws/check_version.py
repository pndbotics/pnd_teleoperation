import requests

try:
    from .common import (
        APP_NAME,
        MINIO_HOST,
        PACKAGE_VERSION_NAME,
        REMOTE_PACKAGE_VERSION_NAME,
    )
except ImportError:
    from common import (
        APP_NAME,
        MINIO_HOST,
        PACKAGE_VERSION_NAME,
        REMOTE_PACKAGE_VERSION_NAME,
    )


def get_remote_package_version():
    url = f"http://{MINIO_HOST}/{APP_NAME}/{REMOTE_PACKAGE_VERSION_NAME}"
    response = requests.get(url)
    return response.text.strip()
