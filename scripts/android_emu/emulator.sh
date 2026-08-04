#!/usr/bin/env bash

set -euo pipefail

function color_msg() {
    printf '%s \033[1;35m%s\033[0m\n' "---" "$1"
}

function run_cmd() {
    echo "+++ $*"
    "$@" || exit 1
}

export PATH="${ANDROID_SDK_ROOT}/tools/bin:${PATH}"
export PATH="${ANDROID_SDK_ROOT}/cmdline-tools/latest/bin:${PATH}"

action="${1:-}"

if [[ "${action}" == enable_kvm ]]
then
    color_msg "enabling kvm"

    echo 'KERNEL=="kvm", GROUP="kvm", MODE="0666", OPTIONS+="static_node=kvm"' \
        | sudo tee /etc/udev/rules.d/99-kvm4all.rules

    sudo udevadm control --reload-rules
    sudo udevadm trigger --name-match=kvm
fi

if [[ "${action}" == create_routes ]]
then
    color_msg "creating routes"

    # Poll for a while to verify that the emulator has network interfaces available
    ip_output=""
    for _ in $(seq 1 10)
    do
        ip_output="$(adb shell "ip a" 2>/dev/null || true)"
        if echo "${ip_output}" | grep -q 'state UP'
        then
            break
        fi
        sleep 2
    done

    echo "+++ adb shell ip a"
    echo "${ip_output}"

    # Collect interfaces in state UP. grep returns non-zero when it
    # find nothing, so guard the pipeline against pipefail to avoid aborting
    # the whole script when no interface is up.
    ifaces="$(echo "${ip_output}" \
        | grep 'state UP' \
        | cut -d':' -f2 \
        | awk '{print $1}' \
        | cut -d'@' -f1 || true)"

    if [[ -z "${ifaces}" ]]
    then
        color_msg "warning: no network interface is up, skipping route setup"
    fi

    # Iterate over a plain variable (not a pipe) so that the "adb" calls inside
    # the loop do not consume the loop's stdin and skip interfaces.
    for iface in ${ifaces}
    do
        if ! adb shell ip route show table all | \
                grep -qF "224.0.0.0/4 dev ${iface} table local"
        then
            run_cmd adb shell "su 0 ip route add 224.0.0.0/4 dev ${iface} table local"
        fi
    done
fi

if [[ "${action}" == start_avd ]]
then
    # create avd if it doesn't exist
    if ! avdmanager list avd -c | grep -qF roc_device
    then
        color_msg "creating avd"
        run_cmd device \
            --name roc_device --image "default" --arch "${ABI}" --api "${API}" \
            create
    fi

    # show avd list
    color_msg "checking avd"
    run_cmd avdmanager list avd

    # start emulator if it's not started
    if ! adb devices | grep -qF emulator
    then
        color_msg "starting device"
        run_cmd device --name roc_device start
    fi

    # show device list
   color_msg "checking device"
   run_cmd adb devices
fi
