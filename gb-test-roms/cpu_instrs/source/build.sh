#!/bin/bash

i=0

for SRC in *.s; do
    ((i++))

    if [[ $SRC == 02* ]]; then
        continue
    fi
    
    echo "building $SRC"

    # build test.gb rom from src file (already implemented):
    wla-gb -D BUILD_JIT_TEST -o test.o "$SRC"
    if [ $? -ne 0 ]; then
        echo "Error: wla-gb failed on $SRC"
        exit 1
    fi
    
    echo "linking $SRC"

    wlalink -s linkfile test.gb
    if [ $? -ne 0 ]; then
        echo "Error: wlalink failed on test.o"
        exit 1
    fi
    
    echo "built $SRC"
    
    cp test.sym "test_$i.sym"
    
    # find the line with _ROM_END_ and extract the length of the rom
    ROM_LEN=$(grep '_ROM_END_' test.sym | awk -F':' '{print $2}' | awk '{print $1}')
    if [ -z "$ROM_LEN" ]; then
        echo "Error: _ROM_END_ not found in test.sym"
        exit 1
    fi
    
    # confirm that ROM_LEN is a hexadecimal number
    if [[ ! $ROM_LEN =~ ^[0-9A-Fa-f]+$ ]]; then
        echo "Error: ROM_LEN is not a hexadecimal number"
        exit 1
    fi
    
    # convert hexadecimal ROM length to decimal
    ROM_LEN_DEC=$((16#$ROM_LEN))
    
    # crop test.gb (a binary file) to the length $ROM_LEN
    dd if=test.gb of="test_$i.gb" bs=1 count=$ROM_LEN_DEC
    if [ $? -ne 0 ]; then
        echo "Error: dd failed on test.gb"
        exit 1
    fi
    
    # convert the resulting binary file to a .c source file containing
    # an array of values.
    xxd -i "test_$i.gb" > "test_$i.c"
    if [ $? -ne 0 ]; then
        echo "Error: xxd failed on test_$i.gb"
        exit 1
    fi
done
