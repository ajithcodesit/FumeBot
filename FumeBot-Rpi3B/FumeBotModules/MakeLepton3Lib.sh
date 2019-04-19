#!/bin/bash

echo "Creating the object file"
gcc -Wall -fPIC -c Lepton3capture.c -o Lepton3capture.o

echo "Creating Shared Object from Object file"
gcc Lepton3capture.o -shared -o libLep3cap.so -lpthread

echo "Done"

