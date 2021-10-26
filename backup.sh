#!/bin/bash

set -e

if [[ -d robotcar_ws ]]; then
	rm -r robotcar_ws
	echo "[INFO] robotcar_ws removed"
else 
	echo "[INFO] No need to removed"
fi


# 1
cp -r ~/robotcar_ws/ ./
echo "[INFO] robotcar_ws backup success!"
cd ./robotcar_ws
rm -r build/ devel/


exit 0
