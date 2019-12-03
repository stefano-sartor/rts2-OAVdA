#ifndef MODBUS_ADDRESS
#define MODBUS_ADDRESS
#include "modbus.h"

namespace oavda
{

const  Modbus::bulk_coils cupola_status_coil_r =  {
    {
        {"Status_CCW"              , {0,0}},
        {"Status_CW"               , {1,0}},
        {"Status_Reset_Encoder"    , {3,0}},
        {"Status_Park"             , {4,0}},
    },
    201,
    5
};

const  Modbus::bulk_coils cupola_move_coil_rw =  {
    {
        {"Stop"                    , {0,0}},
        {"Move_CCW"                , {1,0}},
        {"Move_CW"                 , {2,0}},
    },
    100,
    3
};

const  Modbus::bulk_coils cupola_reset_coil_rw =  {
    {
        {"StopReset_Encoder"       , {0,0}},
        {"Reset_Encoder"           , {1,0}},
    },
    103,
    2
};

const  Modbus::bulk_coils cupola_init_coil_rw =  {
    {
        {"Init"                    , {0,0}},
    },
    115,
    1
};

const  Modbus::bulk_registers cupola_status_reg_r =  {
    {
        {"Position_Encoder"        , {0,0}},
    },
    300,
    2
};

const  Modbus::bulk_coils focus_status_coil_r =  {
    {
        {"Status_FW"               , {0,0}},
        {"Status_BW"               , {1,0}},
        {"Status_Slow"             , {3,0}},
        {"Status_Reset_Encoder"    , {5,0}},
    },
    206,
    6
};

const  Modbus::bulk_coils focus_move_coil_rw =  {
    {
        {"Stop"                    , {0,0}},
        {"Move_FW"                 , {1,0}},
        {"Move_BW"                 , {2,0}},
        {"Stop_Slow"               , {3,0}},
        {"Move_Slow"               , {4,0}},
    },
    105,
    5
};

const  Modbus::bulk_coils focus_reset_coil_rw =  {
    {
        {"StopReset_Encoder"       , {0,0}},
        {"Reset_Encoder"           , {1,0}},
    },
    110,
    2
};

const  Modbus::bulk_coils focus_init_coil_rw =  {
    {
        {"Init"                    , {0,0}},
    },
    116,
    1
};

const  Modbus::bulk_registers focus_status_reg_r =  {
    {
        {"Position_Encoder"        , {0,0}},
    },
    302,
    2
};

const  Modbus::bulk_coils ra_status_coil_r =  {
    {
        {"Status_Park"             , {0,0}},
        {"Status_STOP"             , {1,0}},
    },
    212,
    2
};

const  Modbus::bulk_coils ra_init_coil_rw =  {
    {
        {"Init"                    , {0,0}},
    },
    117,
    1
};

const  Modbus::bulk_coils ra_reset_position_coil_rw =  {
    {
        {"Reset_Position_Motor"    , {0,0}},
    },
    120,
    1
};

const  Modbus::bulk_coils ra_goto_coil_rw =  {
    {
        {"GOTO_Remote"             , {0,0}},
    },
    121,
    1
};

const  Modbus::bulk_coils ra_reset_steps_360_coil_rw =  {
    {
        {"Reset_Steps_360"         , {0,0}},
    },
    124,
    1
};

const  Modbus::bulk_coils ra_stop_coil_rw =  {
    {
        {"Enable_STOP"             , {0,0}},
        {"STOP"                    , {1,0}},
    },
    127,
    2
};

const  Modbus::bulk_registers ra_status_reg_r =  {
    {
        {"Position_Motor"          , {0,0}},
        {"Speed_microSec"          , {2,0}},
    },
    304,
    4
};

const  Modbus::bulk_registers ra_steps_360_reg_r =  {
    {
        {"Steps_360"               , {0,0}},
    },
    312,
    2
};

const  Modbus::bulk_registers ra_encoder_reg_r =  {
    {
        {"Position_Encoder"        , {0,0}},
    },
    316,
    2
};

const  Modbus::bulk_registers ra_set_position_motor_reg_rw =  {
    {
        {"Set_Position_Motor"      , {0,0}},
    },
    404,
    2
};

const  Modbus::bulk_registers ra_move_reg_rw =  {
    {
        {"Speed_init_microSec"     , {0,0}},
        {"Speed_final_microSec"    , {2,0}},
        {"Steps_accel"             , {4,0}},
        {"Steps_decel"             , {6,0}},
        {"Target_Position_Motor"   , {8,0}},
    },
    406,
    10
};

const  Modbus::bulk_registers ra_set_steps_360_reg_rw =  {
    {
        {"Set_Steps_360"           , {0,0}},
    },
    428,
    2
};

const  Modbus::bulk_coils dec_status_coil_r =  {
    {
        {"Status_Park"             , {0,0}},
        {"Status_Limit_H"          , {1,0}},
        {"Status_Limit_L"          , {2,0}},
        {"Status_STOP"             , {3,0}},
    },
    214,
    4
};

const  Modbus::bulk_coils dec_init_coil_rw =  {
    {
        {"Init"                    , {0,0}},
    },
    118,
    1
};

const  Modbus::bulk_coils dec_reset_position_coil_rw =  {
    {
        {"Reset_Position_Motor"    , {0,0}},
    },
    122,
    1
};

const  Modbus::bulk_coils dec_goto_coil_rw =  {
    {
        {"GOTO_Remote"             , {0,0}},
    },
    123,
    1
};

const  Modbus::bulk_coils dec_reset_steps_360_coil_rw =  {
    {
        {"Reset_Steps_360"         , {0,0}},
    },
    125,
    1
};

const  Modbus::bulk_coils dec_stop_coil_rw =  {
    {
        {"Enable_STOP"             , {0,0}},
        {"STOP"                    , {1,0}},
    },
    129,
    2
};

const  Modbus::bulk_registers dec_status_reg_r =  {
    {
        {"Position_Motor"          , {0,0}},
        {"Speed_microSec"          , {2,0}},
    },
    308,
    4
};

const  Modbus::bulk_registers dec_steps_360_reg_r =  {
    {
        {"Steps_360"               , {0,0}},
    },
    314,
    2
};

const  Modbus::bulk_registers dec_encoder_reg_r =  {
    {
        {"Position_Encoder"        , {0,0}},
    },
    318,
    2
};

const  Modbus::bulk_registers dec_set_position_motor_reg_rw =  {
    {
        {"Set_Position_Motor"      , {0,0}},
    },
    416,
    2
};

const  Modbus::bulk_registers dec_move_reg_rw =  {
    {
        {"Speed_init_microSec"     , {0,0}},
        {"Speed_final_microSec"    , {2,0}},
        {"Steps_accel"             , {4,0}},
        {"Steps_decel"             , {6,0}},
        {"Target_Position_Motor"   , {8,0}},
    },
    418,
    10
};

const  Modbus::bulk_registers dec_set_steps_360_reg_rw =  {
    {
        {"Set_Steps_360"           , {0,0}},
    },
    430,
    2
};

} //namespace oavda

#endif
