#!/bin/bash

for file in ./Plugins/Linux/x86_64/*
do
	echo "            --- Showing missing libs for: $file" ---
	patchelf --print-needed $file
done
	
