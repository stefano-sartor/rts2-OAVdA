#include "oavda/modbus.h"
#include <boost/endian/conversion.hpp>

namespace oavda
{

Modbus::Modbus()
    : _mb(nullptr)
{
}

Modbus::~Modbus()
{
    if (_mb)
    {
        modbus_close(_mb);
        modbus_free(_mb);
    }
}

int Modbus::init(const std::string &ip_addr, uint16_t port)
{
    _mb = modbus_new_tcp(ip_addr.c_str(), port);
    if (!_mb)
    {
        return -1;
    }
    if(modbus_connect(_mb)){
        return -1;
    }
    return 0;
}

int Modbus::set_slave(int slave)
{
    return modbus_set_slave(_mb, slave);
}

int32_t Modbus::mb_to_i32(uint16_t *buff)
{
    boost::endian::native_to_little(buff[0]);
    boost::endian::native_to_little_inplace(buff[1]);
    int32_t *i = reinterpret_cast<int32_t *>(buff);
    boost::endian::little_to_native(i);
    return *i;
}

void Modbus::i32_to_mb(int32_t n, uint16_t *buff)
{
    int32_t *i = new int32_t(n);
    boost::endian::native_to_little_inplace(*i);
    uint16_t *u = reinterpret_cast<uint16_t *>(i);
    buff[0] = boost::endian::little_to_native(u[0]);
    buff[1] = boost::endian::little_to_native(u[1]);
}

int Modbus::read_bits(int addr_start, int num, uint8_t *dest)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_read_bits(_mb, addr_start, num, dest);
}

int Modbus::read_bits(Modbus::bulk_coils &bulk)
{
    uint8_t buff[bulk.len];
    int ret = read_bits(bulk.start, bulk.len, buff);
    for (auto &i : bulk.value_map)
    {
        i.second.second = buff[i.second.first];
    }
    return ret;
}

int Modbus::read_input_bits(int addr_start, int num, uint8_t *dest)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_read_input_bits(_mb, addr_start, num, dest);
}

int Modbus::read_input_bits(Modbus::bulk_coils &bulk)
{
    uint8_t buff[bulk.len];
    int ret = read_bits(bulk.start, bulk.len, buff);
    for (auto &i : bulk.value_map)
    {
        i.second.second = buff[i.second.first];
    }
    return ret;
}

int Modbus::write_bit(int addr, int val)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_write_bit(_mb, addr, val);
}

int Modbus::write_bit(const std::string &key, const Modbus::bulk_coils &bulk)
{
    if (bulk.value_map.find(key) == bulk.value_map.end())
        return -1;

    int addr = bulk.start + bulk.value_map.at(key).first;
    return write_bit(addr, bulk.value_map.at(key).second);
}

int Modbus::write_bits(int addr_start, int num, const uint8_t *src)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_write_bits(_mb, addr_start, num, src);
}

int Modbus::write_bits(const Modbus::bulk_coils &bulk)
{
    uint8_t buff[bulk.len];
    for (auto &i : bulk.value_map)
    {
        buff[i.second.first] = i.second.second;
    }
    int ret = write_bits(bulk.start, bulk.len, buff);
    return ret;
}

int Modbus::read_registers(int addr_start, int num, uint16_t *dest)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_read_registers(_mb, addr_start, num, dest);
}

int Modbus::read_registers(Modbus::bulk_registers &bulk)
{
    uint16_t buff[bulk.len];
    int ret = read_registers(bulk.start, bulk.len, buff);
    for (auto &i : bulk.value_map)
    {
        i.second.second = mb_to_i32(&buff[i.second.first]);
    }
    return ret;
}

int Modbus::read_input_registers(int addr_start, int num, uint16_t *dest)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_read_input_registers(_mb, addr_start, num, dest);
}

int Modbus::read_input_registers(Modbus::bulk_registers &bulk)
{
    uint16_t buff[bulk.len];
    int ret = read_input_registers(bulk.start, bulk.len, buff);
    for (auto &i : bulk.value_map)
    {
        i.second.second = mb_to_i32(&buff[i.second.first]);
    }
    return ret;
}

int Modbus::write_registers(int addr_start, int num, uint16_t *src)
{
    std::unique_lock<std::mutex> lock(_m);
    return modbus_write_registers(_mb, addr_start, num, src);
}

int Modbus::write_registers(const Modbus::bulk_registers &bulk)
{
    uint16_t buff[bulk.len];
    for (auto &i : bulk.value_map)
    {
        i32_to_mb(i.second.second, &buff[i.second.first]);
    }
    int ret = write_registers(bulk.start, bulk.len, buff);
    return ret;
}

} // namespace oavda