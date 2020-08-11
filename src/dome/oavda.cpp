/*
    CupolaOadva class.
    Copyright (C) 2019 Stefano Sartor <sartor@oavda.it>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "oavda/driver_serial.h"
#include "cupola.h"

#include <iostream>

using namespace oavda;

#define OPT_TEENSYD_ADDR (OPT_LOCAL + 300)
#define OPT_TEENSYD_PORT (OPT_LOCAL + 301)

// Cupola window aperture
#define SPLIT_WIDTH_DEGS 5.00

// Target reached confidence
#define TARGET_REACHED_DELTA 3.00

#define STEPS_360 42900
#define STEPS_0 (STEPS_360 / 2)

class CupolaOavda : public Cupola
{
public:
    CupolaOavda(int argc, char **argv, bool inhibit_auto_close = false);

    virtual int processOption(int in_opt);
    virtual int init();

    // update here status using modbus
    virtual int info();

    virtual int moveStop();

    // calculate split width in arcdeg for given altititude; when copula don't have split at given altitude, returns -1
    virtual double getSlitWidth(double alt) { return SPLIT_WIDTH_DEGS; }

    /* TODO START */
    /**
     * Open dome. Called either for open command, or when system
     * transitioned to on state.
     *
     * @return 0 on success, -1 on error.
     */
    virtual int startOpen()
    {
        std::cout << "CALL: startOpen()" << std::endl;
        return 0;
    }

    /**
     * Check if dome is opened.
     *
     * @return -2 if dome is opened, -1 if there was an error, >=0 is timeout in miliseconds till
     * next isOpened call.
     */
    virtual long isOpened()
    {
        std::cout << "CALL: isOpened()" << std::endl;
        return -2;
    }

    /**
     * Called when dome is fully opened. Can be used to turn off compressors used
     * to open dome etc..
     *
     * @return -1 on error, 0 on success.
     */
    virtual int endOpen()
    {
        std::cout << "CALL: endOpen()" << std::endl;
        return 0;
    }

    /**
     * Called when dome needs to be closed. Should start dome
     * closing sequence.
     *
     * @return -1 on error, 0 on success, 1 if dome will close (trigger for close) after equipment is stowed
     */
    virtual int startClose()
    {
        std::cout << "CALL: startClose()" << std::endl;
        return 0;
    }

    /**
     * Called to check if dome is closed. It is called also outside
     * of the closing sequence, to check if dome is closed when bad
     * weather arrives. When implemented correctly, it should check
     * state of dome end switches, and return proper values.
     *
     * @return -2 if dome is closed, -1 if there was an error, >=0 is timeout in miliseconds till
     * next isClosed call (when dome is closing).
     */
    virtual long isClosed()
    {
        std::cout << "CALL: isClosed()" << std::endl;
        return -2;
    }

    /**
     * Called after dome is closed. Can turn of various devices
     * used to close dome,..
     *
     * @return -1 on error, 0 on sucess.
     */
    virtual int endClose()
    {
        std::cout << "CALL: endClose()" << std::endl;
        return 0;
    }
    /* TODO END */

    virtual int ready();

protected:
    // called to bring copula in sync with target az
    virtual int moveStart();

    /**
     * Check if Cupola is moving. Called during
     * movement to detect if the target destination was reached.
     *
     * @return -2 when destination was reached, -1 on failure, >= 0
     * return value is number of milliseconds for next isMoving
     * call.
     */
    virtual long isMoving();
    virtual int moveEnd();

    // called when dome passed some states..
    virtual int observing()
    {
        std::cout << "CALL: observing()" << std::endl;
        return Cupola::observing();
    }
    virtual int standby()
    {
        std::cout << "CALL: standby()" << std::endl;
        return Cupola::standby();
    }
    virtual int off()
    {
        std::cout << "CALL: off()" << std::endl;
        return Cupola::off();
    }

private:
    oavda::AxisPWM _motor;

    rts2core::ValueInteger *_encoder_steps;
    rts2core::ValueFloat *_speed;

    std::string _teensyd_ip;
    uint16_t _teensyd_port;
};

inline double steps2az(const int32_t &steps)
{
    int32_t pos = steps - STEPS_0;
    //modulo implementation wich works also with negative numbers
    pos = (STEPS_360 + (pos % STEPS_360)) % STEPS_360;
    double d_pos = double(pos * 360) / double(STEPS_360);

    // libnova...
    d_pos += 180.0;
    if (d_pos > 360.0)
        d_pos -= 360;
    return d_pos;
}
inline int32_t az2steps(const double &az)
{
    // libnova...
    double d_pos = az + 180.0;
    if (d_pos > 360.0)
        d_pos -= 360;

    return d_pos * STEPS_360 + STEPS_0;
}

CupolaOavda::CupolaOavda(int argc, char **argv, bool inhibit_auto_close)
    : Cupola(argc, argv, inhibit_auto_close),
      _motor(oavda::AxisPWM::axis::DOME),
      _teensyd_ip("localhost"),
      _teensyd_port(1500)
{

    createValue(_encoder_steps, "ENCODER_RAW", "raw encoder value", false);
    _encoder_steps->setValueInteger(0);

    createValue(_speed, "SPEED", "speed in steps/s, positive CW", false);
    _speed->setValueInteger(0);

    addOption(OPT_TEENSYD_ADDR, "teensyd-ip", 1, "ip address of the teensy daemon");
    addOption(OPT_TEENSYD_PORT, "teensyd-port", 1, "port of the the teensy daemon");
}

int CupolaOavda::processOption(int in_opt)
{
    switch (in_opt)
    {
    case OPT_TEENSYD_ADDR:
        _teensyd_ip = std::string(optarg);
        break;
    case OPT_TEENSYD_PORT:
        _teensyd_port = atoi(optarg);
        break;
    default:
        return Cupola::processOption(in_opt);
    }
    return 0;
}

int CupolaOavda::init()
{

    std::cout << "CALL: init()" << std::endl;
    int ret;
    ret = Cupola::init();
    if (ret)
        return ret;

    if (_teensyd_ip.empty())
        return -1;

    ret = _motor.init(_teensyd_ip, _teensyd_port);
    if (ret)
        return ret;

    return 0;
}

int CupolaOavda::info()
{

    oavda::error_t err = 0;
    int32_t pos = _motor.get_position(err);
    if (err)
        return -1;
    float speed = _motor.get_speed(err);
    if (err)
        return -1;

    _encoder_steps->setValueInteger(pos);
    _speed->setValueFloat(speed);

    double az = steps2az(pos);
    setCurrentAz(az);

    sendValueAll(_encoder_steps);
    sendValueAll(_speed);

    return Cupola::info();
}

int CupolaOavda::moveStop()
{

    std::cout << "CALL: moveStop()" << std::endl;
    oavda::error_t err = _motor.stop();
    if (err)
        return -1;
    return Cupola::moveStop();
}

int CupolaOavda::moveStart()
{

    std::cout << "CALL: moveStart()";

    int32_t target = az2steps(getTargetAz());
    oavda::error_t err = 0;
    _motor.go_to(target,err);
    if(err) return -1;

    return Cupola::moveStart();
}

/**
     * Check if Cupola is moving. Called during
     * movement to detect if the target destination was reached.
     *
     * @return -2 when destination was reached, -1 on failure, >= 0
     * return value is number of milliseconds for next isMoving
     * call.
     */
long CupolaOavda::isMoving()
{

    //    std::cout << "CALL: isMoving()" << std::endl;
    if (fabs(getTargetDistance()) <= TARGET_REACHED_DELTA)
    {
        return -2;
    }
    else
    {
        //TODO tune this relative to speed and distance
        return 300; // check if reached di no less then 1000 ms
    }
}

int CupolaOavda::moveEnd()
{

    std::cout << "CALL: moveEnd()" << std::endl;

    oavda::error_t err = _motor.stop();
    if(err) return -1;
    return Cupola::moveEnd();
}

int CupolaOavda::ready()
{
    std::cout << "CALL: ready()" << std::endl;
    return 0;
}


//#define OAVDA_TEST 1
#ifndef OAVDA_TEST

int main(int argc, char **argv)
{
    CupolaOavda device(argc, argv);
    return device.run();
}

#else

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
int main()
{
    auto reg = cupola_registers_r;
    Modbus m;
    if (m.init("169.254.181.2"))
    {
        std::cout << "error connection" << std::endl;
        return -1;
    }
    if (m.set_slave(1))
    {
        std::cout << "error connection" << std::endl;
        return -1;
    }

    while (true)
    {
        m.read_input_registers(reg);
        std::cout << reg.at("Position_Encoder") << " " << steps2az(reg.at("Position_Encoder")) << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

#endif
