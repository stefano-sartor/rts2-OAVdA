#ifndef MODBUS_ADDRESS
#define MODBUS_ADDRESS
#include "modbus.h"

namespace oavda
{

const  Modbus::bulk_coils cupola_coils_r =  {
    {
        {"Status_CCW"              , {0,0}},
        {"Status_CW"               , {1,0}},
        {"Status_Reset_Encoder"    , {3,0}},
        {"Status_Park"             , {4,0}},
    },
    201,
    5
};

const  Modbus::bulk_coils cupola_coils_move_rw =  {
    {
        {"Stop"                    , {0,0}},
        {"Move_CCW"                , {1,0}},
        {"Move_CW"                 , {2,0}},
    },
    100,
    3
};

const  Modbus::bulk_coils cupola_coils_encoder_rw =  {
    {
        {"StopReset_Encoder"       , {0,0}},
        {"Reset_Encoder"           , {1,0}},
    },
    103,
    2
};

const Modbus::bulk_registers cupola_registers_r =  {
    {
        {"Position_Encoder"        , {0,0}},
    },
    300,
    2
};

const  Modbus::bulk_coils focus_coils_r =  {
    {
        {"Status_FW"               , {0,0}},
        {"Status_BW"               , {1,0}},
        {"Status_Slow"             , {3,0}},
        {"Status_Reset_Encoder"    , {5,0}},
    },
    206,
    6
};

const  Modbus::bulk_coils focus_coils_rw =  {
    {
        {"Stop"                    , {0,0}},
        {"Move_FW"                 , {1,0}},
        {"Move_BW"                 , {2,0}},
        {"Stop_Slow"               , {3,0}},
        {"Move_Slow"               , {4,0}},
        {"StopReset_Encoder"       , {5,0}},
        {"Reset_Encoder"           , {6,0}},
    },
    105,
    7
};

const Modbus::bulk_registers focus_registers_r =  {
    {
        {"Position_Encoder"        , {0,0}},
    },
    302,
    2
};

const  Modbus::bulk_coils ra_coils_r =  {
    {
        {"Status_Park"             , {0,0}},
    },
    212,
    1
};

const  Modbus::bulk_coils ra_coils_rw =  {
    {
        {"Reset_Position_Motor"    , {0,0}},
        {"GOTO_Remote"             , {1,0}},
        {"Reset_Steps_360"         , {4,0}},
    },
    120,
    5
};

const Modbus::bulk_registers ra_registers_r =  {
    {
        {"Position_Motor"          , {0,0}},
        {"Speed_microSec"          , {2,0}},
        {"Steps_360"               , {8,0}},
        {"Position_Encoder"        , {12,0}},
    },
    304,
    14
};

const Modbus::bulk_registers ra_registers_rw =  {
    {
        {"Set_Position_Motor"      , {0,0}},
        {"Speed_init_microSec"     , {2,0}},
        {"Speed_final_microSec"    , {4,0}},
        {"Steps_accel"             , {6,0}},
        {"Steps_decel"             , {8,0}},
        {"Target_Position_Motor"   , {10,0}},
        {"Set_Steps_360"           , {24,0}},
    },
    404,
    26
};

const  Modbus::bulk_coils dec_coils_r =  {
    {
        {"Status_Park"             , {0,0}},
        {"Status_Limit_H"          , {1,0}},
        {"Status_Limit_L"          , {2,0}},
    },
    213,
    3
};

const  Modbus::bulk_coils dec_coils_rw =  {
    {
        {"Reset_Position_Motor"    , {0,0}},
        {"GOTO_Remote"             , {1,0}},
        {"Reset_Steps_360"         , {3,0}},
    },
    122,
    4
};

const Modbus::bulk_registers dec_registers_r =  {
    {
        {"Position_Motor"          , {0,0}},
        {"Speed_microSec"          , {2,0}},
        {"Steps_360"               , {6,0}},
        {"Position_Encoder"        , {10,0}},
    },
    308,
    12
};

const Modbus::bulk_registers dec_registers_rw =  {
    {
        {"Set_Position_Motor"      , {0,0}},
        {"Speed_init_microSec"     , {2,0}},
        {"Speed_final_microSec"    , {4,0}},
        {"Steps_accel"             , {6,0}},
        {"Steps_decel"             , {8,0}},
        {"Target_Position_Motor"   , {10,0}},
        {"Set_Steps_360"           , {14,0}},
    },
    416,
    16
};

} //namespace oavda

#endif
