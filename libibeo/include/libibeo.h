#pragma once

/*

libibeo: client library to interface with ibeo LUX laser scanners


Copyright 2014, Aaron Klingaman, Limitedslip Engineering LLC

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3.0 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library.
*/


#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>


#include "message_types.h"


namespace ibeo
{


typedef boost::function<void (const ErrorWarning& )> ErrorWarningHandler;
typedef boost::function<void (const ScanDataPoints& )> ScanPointsHandler;
typedef boost::function<void (const ScanDataObjects& )> ScanObjectsHandler;


class LUX : public boost::enable_shared_from_this<LUX>
{
public:
    LUX();
    ~LUX();

    bool connect( const std::string& device_addr, unsigned short port );
    void disconnect();
    bool reset();

    void register_error_warning_handler( ErrorWarningHandler ew_handler );
    void register_scan_points_handler( ScanPointsHandler points_handler );
    void register_scan_objects_handler( ScanObjectsHandler objects_handler );

    const std::string& last_error();


protected:
    // this should be at least two, depending on how
    // many async call backs are to be supported.
    // leave at least one thread dedicated to async
    // read/write to the device.
    const unsigned int IOSERVICE_THREAD_POOL_SIZE = 2;



    void _io_service_run();
    void _start_io_service_threads();

    void _start_read_header_magic();
    void _handle_read_header_magic( const boost::system::error_code& err,
                                   std::size_t bytes_transferred );
    void _start_read_header();
    void _handle_read_header( const boost::system::error_code& err,
                                std::size_t bytes_transferred );
    void _start_write_cmd();
    void _handle_write_cmd( const boost::system::error_code& err );

    void _start_read_scan_data_header();
    void _handle_read_scan_data_header( const boost::system::error_code& err, std::size_t bytes_transferred );

    void _start_read_scan_points();
    void _handle_read_scan_points( const boost::system::error_code& err, std::size_t bytes_transferred );

    void _start_read_error_warning_data();
    void _handle_read_error_warning_data( const boost::system::error_code& err, std::size_t bytes_transferred );

    void _start_read_object_data_header();
    void _handle_read_object_data_header( const boost::system::error_code& err, std::size_t bytes_transferred );

    void _start_read_single_object_data();
    void _handle_read_single_object_data( const boost::system::error_code& err, std::size_t bytes_transferred );

    void _start_read_object_data_contours();
    void _handle_read_object_data_contours( const boost::system::error_code& err, std::size_t bytes_transferred );

    static void _make_cmd_header( std::vector<char>& buf, uint32_t message_data_size );
    static void _make_cmd_reset( std::vector<char>& buf );
    static void _make_cmd_get_status( std::vector<char>& buf );
    static void _make_cmd_start_measure( std::vector<char>& buf );
    static void _make_cmd_stop_measure( std::vector<char>& buf );
    static void _make_cmd_save_config( std::vector<char>& buf );
    static void _make_cmd_set_parameter( std::vector<char>& buf, uint16_t parameter_index, uint32_t parameter );
    static void _make_cmd_get_parameter( std::vector<char>& buf, uint16_t parameter_index );
    static void _make_cmd_reset_default_parameters( std::vector<char>& buf );
    static void _make_cmd_set_ntp_timestamp_sync( std::vector<char>& buf, uint32_t seconds, uint32_t fractional_seconds );

    static void _ntoh_MessageHeader( MessageHeader& h );

    // handlers for getting data back
    boost::mutex m_ew_handler_mutex;
    ErrorWarningHandler m_ew_handler;

    boost::mutex m_points_handler_mutex;
    ScanPointsHandler m_points_handler;

    boost::mutex m_objects_handler_mutex;
    ScanObjectsHandler m_objects_handler;

    boost::asio::io_service m_io_service;
    boost::asio::io_service::work m_io_service_work;

    // io_service thread pool
    std::vector<boost::shared_ptr<boost::thread> > m_io_service_thread_pool;

    boost::asio::ip::tcp::resolver m_resolver;
    boost::asio::ip::tcp::socket m_socket;

    std::vector<char> m_send_buf;
    std::string m_magic_word_buf;

    MessageHeader m_current_header;

    ScanDataPoints m_current_scan_data;

    ScanDataObjects m_current_objects_data;

    ScanDataObject m_current_object;
    uint16_t m_remaining_objects;

    boost::asio::streambuf m_response_buf;

    std::string m_last_error;
};


typedef boost::shared_ptr<LUX> LUXptr;


// lifted from the asio method of avoiding the ugly _1 in the
// boost bind calls. see ibeoscan/main.cpp for an example.
namespace placeholders
{

template <int Number>
struct placeholder
{
    static boost::arg<Number>& get()
    {
        static boost::arg<Number> result;
        return result;
    }
};


// argument placeholder, for use with boost::bind(), that corresponds to
// the ErrorWarning argument of a handler for any of the asynchronous functions.
boost::arg<1>& ew = ibeo::placeholders::placeholder<1>::get();

// argument placeholder, for use with boost::bind(), that corresponds to
// the points argument of a handler for any of the asynchronous functions.
boost::arg<1>& points = ibeo::placeholders::placeholder<1>::get();

// argument placeholder, for use with boost::bind(), that corresponds to
// the objects argument of a handler for any of the asynchronous functions.
boost::arg<1>& objects = ibeo::placeholders::placeholder<1>::get();

}


}

