#!/bin/bash

if [ -d "test_csf/build_test" ];
then
        rm -rf tests/build/*
else
        mkdir -p tests/build
fi

cd tests/build

qmake ..

make -j $(($(nproc) - 1))

shopt -s extglob
rm -rf !("tests")

./tests
