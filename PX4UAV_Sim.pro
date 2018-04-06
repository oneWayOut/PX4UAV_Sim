TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/main.cpp \
    src/FGColumnVector3.cpp \
    src/FGJSBBase.cpp \
    src/FGLocation.cpp \
    src/FGMatrix33.cpp \
    src/FGPropagate.cpp \
    src/FGQuaternion.cpp \
    src/FGAdaptor.cpp \
    src/ForceMoments.cpp

HEADERS += \
    src/FGColumnVector3.h \
    src/FGJSBBase.h \
    src/FGLocation.h \
    src/FGMatrix33.h \
    src/FGPropagate.h \
    src/FGQuaternion.h \
    src/FGAdaptor.h \
    src/ForceMoments.h

