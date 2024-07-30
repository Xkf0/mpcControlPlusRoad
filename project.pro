#QT -= gui core

#CONFIG += C++11 console
#CONFIG -= app_bundle

##\\库名称
#TARGET = nav_mpc
#TEMPLATE = lib

#DEFINES += NAVMPC_LIBRARY

#DEFINES += QT_DEFRECATED_WARNINGS

HEADERS += \
    getControl/min_j.hpp \
    getControl/mpc_control.h \
    getRoad/mpc_road.h \
    math_formula.hpp \
    params/parameter_reader.h \
    params/params_option.hpp \
    robos/duration.h \
    robos/geometry_msgs.h \
    robos/navi_msgs.h \
    robos/rate.h \
    robos/sensor_msgs.h \
    robos/srv_msgs.h \
    robos/std_msgs.h \
    robos/time.h \
    bezier_smoothing/bezier_smoothing.h \
    nav_control.h \
    inf.h

SOURCES += \
    getControl/mpc_control.cpp \
    getRoad/mpc_road.cpp \
    main.cpp \
    bezier_smoothing/bezier_smoothing.cpp \
    nav_control.cpp \
    params/parameter_reader.cpp

LIBS += -L/usr/local/lib/ -lnlopt


