#!/bin/bash

if [ -d "test_csf/build_test" ];
then
        rm -rf test_csf/build_test/*
else
        mkdir -p test_csf/build_test
fi

cd test_csf/build_test

qmake ..

make -j $(($(nproc) - 1))

shopt -s extglob
rm -rf !("test_csf")

./test_csf
