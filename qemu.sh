#!/bin/bash

PORT=1248

killall qemu-system-arm

function bg() {
    $@&
}

DEBUG=""
DEBUG_E=""
if [ $# -ge 1 ] && [ "$1" == "--debug" ]
then
    DEBUG="-gdb tcp::$PORT -S -singlestep"
    DEBUG_E="bg "
fi

set -e
$DEBUG_E qemu-system-arm -cpu cortex-m3 -machine mps2-an385 -nographic -semihosting -monitor none -serial stdio $DEBUG -kernel ./jt_arm

if [ ! -z "$DEBUG" ]
then
    sleep 0.5
    gdb-multiarch \
        -iex "file ./jt_arm" \
        -iex "target remote localhost:$PORT" \
        -iex "set arm force-mode thumb" \
        -iex "display/i \$pc" \
        -iex "b main" \
        -iex "c"
fi
