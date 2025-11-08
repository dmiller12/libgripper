#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/executor_work_guard.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <thread>
#include <future>

class SerialCommunicator {
public:
    SerialCommunicator();
    ~SerialCommunicator();

    // Disable copy and move semantics
    SerialCommunicator(const SerialCommunicator&) = delete;
    SerialCommunicator& operator=(const SerialCommunicator&) = delete;
    SerialCommunicator(SerialCommunicator&&) = delete;
    SerialCommunicator& operator=(SerialCommunicator&&) = delete;

    bool connect(const std::string& port, unsigned int baud_rate);
    void disconnect();
    bool isOpen() const;
    std::future<bool> write(const std::vector<uint8_t>& data);
    std::future<std::vector<uint8_t>> read(size_t num_bytes, int timeout_ms);
    std::future<std::vector<uint8_t>> read_until(std::string delim, int timeout_ms);

private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port port_;
    boost::asio::steady_timer timer_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;
};
