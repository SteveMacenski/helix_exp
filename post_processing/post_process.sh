#!/bin/sh

for i in `ls -v *.bmp`; do
    ./segm $i
done
