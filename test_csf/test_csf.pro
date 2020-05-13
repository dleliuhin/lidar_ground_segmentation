#========================================================================================

TARGET = test_livox_cluster

TEMPLATE = app

#========================================================================================

QT += core
QT -= gui

#========================================================================================

CONFIG *= link_pkgconfig
CONFIG += c++11

#========================================================================================

DEFINES += QT_DEPRECATED_WARNINGS

#========================================================================================

MAIN_DIR = $$PWD

ZCM_DIR = $$PWD/../zcm_types
include( $$PWD/../zcm_types.pri )
LIBS += -L$$ZCM_DIR

INCLUDEPATH += /usr/src/gtest/include/gtest/
LIBS += -lgtest

VLIBS_DIR = $$PWD/../vlibs
include( $$PWD/../vlibs.pri )

SOURCES += main.cpp

include( $$PWD/../clusterization/clusterization.pri )
include( $$PWD/../config_reader/config_reader.pri )
include( $$PWD/../defs/defs.pri )

#========================================================================================
