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

#include "oavda/modbus.h"
#include "oavda/modbus_address.h"
#include "cupola.h"

#include <iostream>

using namespace oavda;

#define OPT_CRIO_ADDR (OPT_LOCAL + 300)
#define OPT_CRIO_PORT (OPT_LOCAL + 301)

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
    Modbus _m;
    Modbus::bulk_coils _r_coils_status;
    Modbus::bulk_coils _rw_coils_move;
    Modbus::bulk_coils _rw_coils_encoder;
    Modbus::bulk_registers _r_regs;

    rts2core::ValueBool *_status_park;
    rts2core::ValueBool *_status_CCW;
    rts2core::ValueBool *_status_CW;
    rts2core::ValueBool *_status_reset_Encoder;

    rts2core::ValueBool *_move_CCW;
    rts2core::ValueBool *_move_CW;
    rts2core::ValueBool *_move_stop;

    rts2core::ValueBool *_stop_reset_encoder;
    rts2core::ValueBool *_reset_encoder;

    rts2core::ValueInteger *_encoder_steps;

    std::string _crio_ip;
    uint16_t _crio_port;
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
CupolaOavda::CupolaOavda(int argc, char **argv, bool inhibit_auto_close)
    : Cupola(argc, argv, inhibit_auto_close),
      _r_coils_status(cupola_coils_r),
      _rw_coils_move(cupola_coils_move_rw),
      _rw_coils_encoder(cupola_coils_encoder_rw),
      _r_regs(cupola_registers_r),
      _crio_ip(""),
      _crio_port(502)
{
    createValue(_status_park, "PARK_status", "status of the park sensor", false);
    _status_park->setValueBool(false);

    createValue(_status_CCW, "moving_CCW", "true while moving in counterclockwise direction", false);
    _status_CCW->setValueBool(false);

    createValue(_status_CW, "moving_CW", "true while moving in clockwise direction", false);
    _status_CW->setValueBool(false);

    createValue(_move_CCW, "MOVE_CCW", "move the cupola counterclockwise", false, RTS2_VALUE_WRITABLE);
    _move_CCW->setValueBool(false);

    createValue(_move_CW, "MOVE_CW", "move the cupola clockwise", false, RTS2_VALUE_WRITABLE);
    _move_CW->setValueBool(false);

    createValue(_move_stop, "STOP_CUPOLA", "stop the coupola", false, RTS2_VALUE_WRITABLE);
    _move_stop->setValueBool(false);

    createValue(_status_reset_Encoder, "RTS_Encoder", "true if encoder reset flag is active", false);
    _status_reset_Encoder->setValueBool(false);

    createValue(_reset_encoder, "RESET_Encoder", "reset the encoder position", false, RTS2_VALUE_WRITABLE);
    _reset_encoder->setValueBool(false);

    createValue(_stop_reset_encoder, "Disable_RESET_Encoder", "disable reset the encoder position", false, RTS2_VALUE_WRITABLE);
    _stop_reset_encoder->setValueBool(false);

    createValue(_encoder_steps, "ENCODER_RAW", "raw encoder value", false);
    _encoder_steps->setValueInteger(0);

    addOption(OPT_CRIO_ADDR, "crio-ip", 1, "ip address of the C-RIO controller");
    addOption(OPT_CRIO_PORT, "crio-port", 1, "port of the C-RIO controller");
}

int CupolaOavda::processOption(int in_opt)
{
    switch (in_opt)
    {
    case OPT_CRIO_ADDR:
        _crio_ip = std::string(optarg);
        break;
    case OPT_CRIO_PORT:
        _crio_port = atoi(optarg);
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

    if (_crio_ip.empty())
        return -1;

    ret = _m.init(_crio_ip, _crio_port);
    if (ret)
        return ret;

    ret = _m.set_slave(1);
    if (ret)
        return ret;

    return 0;
}

int CupolaOavda::info()
{

    //   std::cout << "CALL: info()" << std::endl;
    int n = _m.read_bits(_r_coils_status);
    if (n != _r_coils_status.len)
    {
        //FIXME log error
        return -1;
    }

    n = _m.read_bits(_rw_coils_move);
    if (n != _rw_coils_move.len)
    {
        //FIXME log error
        return -1;
    }

    n = _m.read_bits(_rw_coils_encoder);
    if (n != _rw_coils_encoder.len)
    {
        //FIXME log error
        return -1;
    }

    n = _m.read_input_registers(_r_regs);
    if (n != _r_regs.len)
    {
        //FIXME log error
        return -1;
    }

    _status_park->setValueBool(_r_coils_status.at("Status_Park"));
    _status_CCW->setValueBool(_r_coils_status.at("Status_CCW"));
    _status_CW->setValueBool(_r_coils_status.at("Status_CW"));
    _status_reset_Encoder->setValueBool(_r_coils_status.at("Status_Reset_Encoder"));

    _move_CCW->setValueBool(_rw_coils_move.at("Move_CCW"));
    _move_CW->setValueBool(_rw_coils_move.at("Move_CW"));
    _move_stop->setValueBool(_rw_coils_move.at("Stop"));

    _stop_reset_encoder->setValueBool(_rw_coils_encoder.at("StopReset_Encoder"));
    _reset_encoder->setValueBool(_rw_coils_encoder.at("Reset_Encoder"));

    _encoder_steps->setValueInteger(_r_regs.at("Position_Encoder"));

    double az = steps2az(_r_regs.at("Position_Encoder"));
    setCurrentAz(az);

    sendValueAll(_status_park);
    sendValueAll(_status_CCW);
    sendValueAll(_status_CW);
    sendValueAll(_status_reset_Encoder);

    sendValueAll(_move_CCW);
    sendValueAll(_move_CW);
    sendValueAll(_move_stop);

    sendValueAll(_stop_reset_encoder);
    sendValueAll(_reset_encoder);

    sendValueAll(_encoder_steps);

    return Cupola::info();
}

int CupolaOavda::moveStop()
{

    std::cout << "CALL: moveStop()" << std::endl;
    _rw_coils_move.at("Stop") = 1;
    _rw_coils_move.at("Move_CCW") = 0;
    _rw_coils_move.at("Move_CW") = 0;
    int ret = _m.write_bits(_rw_coils_move);
    if (ret != _rw_coils_move.len)
    {
        return -1;
    }
    return Cupola::moveStop();
}

int CupolaOavda::moveStart()
{

    std::cout << "CALL: moveStart()";

    double distance = getTargetDistance();

    std::cout << " distance: " << distance << std::endl;
    _rw_coils_move.at("Stop") = 0;
    _rw_coils_move.at("Move_CCW") = 0;
    _rw_coils_move.at("Move_CW") = 0;

    if (distance < 0)
    {
        _rw_coils_move.at("Move_CW") = 1;
    }
    else
    {
        _rw_coils_move.at("Move_CCW") = 1;
    }

    int ret = _m.write_bits(_rw_coils_move);
    if (ret != _rw_coils_move.len)
    {
        return -1;
    }
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
    _rw_coils_move.at("Stop") = 1;
    _rw_coils_move.at("Move_CCW") = 0;
    _rw_coils_move.at("Move_CW") = 0;
    int ret = _m.write_bits(_rw_coils_move);
    if (ret != _rw_coils_move.len)
    {
        return -1;
    }
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
