#!/bin/bash

for file in /opt/Plugins/Linux/x86_64/*
do
	# echo "Setting RPTH to ORIGIN for: $file"
	patchelf --remove-rpath "$file"
	patchelf --force-rpath --set-rpath '$ORIGIN' "$file"
done
	
