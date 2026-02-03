#!/bin/bash

IPS=$(hostname -I | awk '{$1=$1; print}' | sed 's/ /, /g')

if [ -z "$IPS" ]; then
    echo "错误: 未找到有效的 IP 地址"
    exit 1
fi

echo "Caddy 正在启动并监听: $IPS"

CADDY_CONFIG=$(cat <<EOF
{
    auto_https disable_redirects
}
$IPS {
    handle_path /vrwebsocket* {
        reverse_proxy 127.0.0.1:8442 {
            header_up Host {host}
            header_up X-Real-IP {remote_host}
        }
    }
    handle_path /webvr* {
        reverse_proxy 127.0.0.1:8443 {
            header_up Host {host}
            header_up X-Real-IP {remote_host}
        }
    }
    handle_path /nuc_ws* {
        reverse_proxy 127.0.0.1:12078 {
            header_up Host {host}
            header_up X-Real-IP {remote_host}
        }  
    }
    handle_path /webrtc* {
        reverse_proxy 10.10.20.126:8443 {
            header_up Host {host}
            header_up X-Real-IP {remote_host}
        }
    }
    handle_path /jetson_ws* {
        reverse_proxy 10.10.20.126:12079 {
            header_up Host {host}
            header_up X-Real-IP {remote_host}
        }
    }
    @static {
        path /assets/* /index.css /telegrip_instructions.jpg *.js *.css *.ico *.jpg *.jpeg *.png *.gif *.svg *.woff *.woff2 *.ttf *.eot *.json
        not path /vrwebsocket* /webrtc* /webvr* /nuc_ws* /jetson_ws*
    }
    handle @static {
        reverse_proxy 127.0.0.1:8443 {
            header_up Host {host}
            header_up X-Real-IP {remote_host}
        }
    }
    tls internal
}
EOF
)

echo "$CADDY_CONFIG" | caddy run --adapter caddyfile --config -
