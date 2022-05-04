#!/bin/bash




if [ $# != 0 ];
then
	if [ $1 = "clean" ]
	then
		rm -rf build
	fi
else
    mkdir build
    cd build
    cmake ..
    make
    cd ..
fi
