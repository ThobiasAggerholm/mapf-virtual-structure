#!/bin/bash

for e in {0..1}
do
    for i in {0..2}
    do
        for j in {0..4}
        do
            echo "i = $i, j = $j, e = $e"
            ./build/mapf-virtual-structure $i $j $e
        done
    done
done