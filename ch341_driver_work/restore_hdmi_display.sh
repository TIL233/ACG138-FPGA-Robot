#!/bin/sh
# This robot uses an external HDMI screen. Do not alter outputs until HDMI
# is detected; the RK image also advertises two unpopulated DSI connectors.
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/ubuntu/.Xauthority}"
attempt=0
while [ "$attempt" -lt 30 ]; do
    state=$(xrandr --current 2>/dev/null) || state=
    output=$(printf '%s\n' "$state" | awk '$1 ~ /^HDMI-/ && $2 == "connected" {print $1; exit}')
    if [ -n "$output" ]; then
        # Only use connector names actually enumerated by this X server.
        set -- --output "$output" --auto --primary --pos 0x0
        for dsi in $(printf '%s\n' "$state" | awk '$1 ~ /^DSI-[0-9]+$/ && ($2 == "connected" || $2 == "disconnected") {print $1}'); do
            set -- "$@" --output "$dsi" --off
        done
        if xrandr "$@"; then
            xset s off
            xset s noblank
            xset -dpms
            exit 0
        fi
    fi
    attempt=$((attempt + 1))
    sleep 1
done
exit 1
