#include "serial_communicator.h"

// --- Boost.Asio Headers for implementation ---
#include <boost/asio/post.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

// --- Standard Library Headers ---
#include <iostream>

SerialCommunicator::SerialCommunicator()
    : io_context_()
    , port_(io_context_)
    , timer_(io_context_)
    , work_guard_(boost::asio::make_work_guard(io_context_)) {
    // Start the I/O context on a background thread. All async operations
    // will be dispatched to this thread.
    io_thread_ = std::thread([this]() { io_context_.run(); });
}

SerialCommunicator::~SerialCommunicator() {
    disconnect();
    work_guard_.reset(); // Allow io_context::run() to exit
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

bool SerialCommunicator::connect(const std::string& port_name, unsigned int baud_rate) {
    std::promise<bool> promise;
    auto future = promise.get_future();

    boost::asio::post(io_context_, [&, port_name, baud_rate]() {
        boost::system::error_code ec;
        port_.open(port_name, ec);
        if (ec) {
            std::cerr << "[Serial] Error opening port " << port_name << ": " << ec.message() << std::endl;
            promise.set_value(false);
            return;
        }

        // Configure serial port options
        port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate), ec);
        port_.set_option(boost::asio::serial_port_base::character_size(8), ec);
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
        port_.set_option(
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec
        );

        if (ec) {
            std::cerr << "[Serial] Error configuring port: " << ec.message() << std::endl;
            port_.close();
            promise.set_value(false);
            return;
        }
        promise.set_value(true);
    });
    is_open_ = future.get();
    return is_open_;
}

void SerialCommunicator::disconnect() {
    if (!isOpen())
        return;
    boost::asio::post(io_context_, [&]() {
        if (port_.is_open()) {
            port_.close();
        }
    });
    is_open_ = false;
}

bool SerialCommunicator::isOpen() const {
    return is_open_ && port_.is_open();
}

std::future<bool> SerialCommunicator::write(const std::vector<uint8_t>& data) {
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();

    if (!isOpen()) {
        promise->set_value(false);
        return future;
    }

    boost::asio::post(io_context_, [&, data, promise]() {
        boost::system::error_code ec;
        boost::asio::write(port_, boost::asio::buffer(data), ec);
        try {
            promise->set_value(!ec);
            if (ec) {
                std::cerr << "[Serial] Write error: " << ec.message() << std::endl;
            }
        } catch (const std::future_error&) {
            // catch "broken promise" error if user discards future.
        }
    });
    return future;
}

std::future<std::vector<uint8_t>> SerialCommunicator::read(size_t num_bytes, int timeout_ms) {

    auto promise = std::make_shared<std::promise<std::vector<uint8_t>>>();
    auto future = promise->get_future();
    if (!isOpen() || num_bytes == 0) {
        promise->set_value({});
        return future;
    };

    boost::asio::post(io_context_, [&, num_bytes, timeout_ms, promise]() {
        auto read_buffer = std::make_shared<std::vector<uint8_t>>(num_bytes);

        boost::asio::async_read(
            port_,
            boost::asio::buffer(*read_buffer),
            [&, read_buffer, promise](const boost::system::error_code& ec, size_t /*bytes_transferred*/) {
                timer_.cancel(); // Read finished, cancel the timer.
                try {
                    if (!ec) {
                        promise->set_value(*read_buffer);
                    } else if (ec != boost::asio::error::operation_aborted) {
                        promise->set_value({});
                    }

                } catch (const std::future_error&) {
                    // catch "broken promise" error if user discards future.
                }
            }
        );

        // Start a timer to handle timeouts
        timer_.expires_after(std::chrono::milliseconds(timeout_ms));
        timer_.async_wait([&, promise](const boost::system::error_code& ec) {
            if (ec)
                return;     // Timer was cancelled, not expired
            port_.cancel(); // Timer expired, cancel the read operation.
            try {
                promise->set_value({}); // Fulfill promise with empty vector.
            } catch (const std::future_error&) {
                // catch "broken promise" error if user discards future.
            }
        });
    });

    return future;
}

std::future<std::vector<uint8_t>> SerialCommunicator::read_until(std::string delim, int timeout_ms) {
    auto promise = std::make_shared<std::promise<std::vector<uint8_t>>>();
    auto future = promise->get_future();

    if (!isOpen() || delim.length() == 0) {
        promise->set_value({});
        return future;
    }

    boost::asio::post(io_context_, [&, delim, timeout_ms, promise]() {
        auto streambuf = std::make_shared<boost::asio::streambuf>();

        boost::asio::async_read_until(
            port_,
            *streambuf,
            delim,
            [&, streambuf, promise](const boost::system::error_code& ec, size_t bytes_transferred) {
                timer_.cancel(); // Read finished, cancel the timer.
                try {
                    if (!ec) {
                        std::istream is(streambuf.get());
                        std::vector<uint8_t> data(bytes_transferred);
                        is.read(reinterpret_cast<char*>(data.data()), bytes_transferred);

                        promise->set_value(data);
                    } else if (ec != boost::asio::error::operation_aborted) {
                        promise->set_value({});
                    }
                } catch (const std::future_error&) {
                    // catch "broken promise" error if user discards future.
                }
            }
        );

        // Start a timer to handle timeouts
        timer_.expires_after(std::chrono::milliseconds(timeout_ms));
        timer_.async_wait([&, promise](const boost::system::error_code& ec) {
            if (ec)
                return;            // Timer was cancelled, not expired
            port_.cancel();        // Timer expired, cancel the read operation.
            try {
                promise->set_value({}); // Fulfill promise with empty vector.
                      //
            } catch (const std::future_error&) {
                // catch "broken promise" error if user discards future.
            }
        });
    });

    return future;
}
