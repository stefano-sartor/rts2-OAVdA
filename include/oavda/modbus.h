#ifndef OAVDA_MODBUS_H
#define OAVDA_MODBUS_H
#include <modbus/modbus.h>
#include <mutex>
#include <string>
#include <map>

namespace oavda
{

class Modbus
{
public:
    template <typename T>
    struct bulk_map_t
    {
        std::map<std::string, std::pair<int, T>> value_map;
        int start;
        int len;

        T& at(const std::string &key) { return value_map.at(key).second; }
    };

    typedef bulk_map_t<uint8_t> bulk_coils;
    typedef bulk_map_t<int32_t> bulk_registers;

    Modbus();
    ~Modbus();
    int init(const std::string &ip_addr, uint16_t port = 502);
    int set_slave(int slave);

    int read_bits(int addr_start, int num, uint8_t *dest);
    int read_bits(bulk_coils &bulk);

    int read_input_bits(int addr_start, int num, uint8_t *dest);
    int read_input_bits(bulk_coils &bulk);

    int write_bit(int addr, int val);
    int write_bit(const std::string &key, const bulk_coils &bulk);

    int write_bits(int addr_start, int num, const uint8_t *src);
    int write_bits(const bulk_coils &bulk);

    int read_registers(int addr_start, int num, uint16_t *dest);
    int read_registers(bulk_registers &bulk);

    int read_input_registers(int addr_start, int num, uint16_t *dest);
    int read_input_registers(bulk_registers &bulk);

    int write_registers(int addr_start, int num, uint16_t *src);
    int write_registers(const bulk_registers &bulk);

private:
    static int32_t mb_to_i32(uint16_t *buff);
    static void i32_to_mb(int32_t n, uint16_t *buff);
    std::mutex _m;
    modbus_t *_mb;
};

} // namespace oavda

#endif