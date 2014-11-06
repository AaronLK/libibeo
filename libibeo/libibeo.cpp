#include "libibeo.h"

/*

libibeo: client library to interface with ibeo LUX laser scanners


Copyright 2014, Aaron Klingaman, Limitedslip Engineering, LLC

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


#include <boost/lexical_cast.hpp>


#include "message_types.h"



// lets keep these namespaces under control
using boost::asio::ip::tcp;


namespace ibeo
{

LUX::LUX() :
  m_resolver(m_io_service), m_socket(m_io_service), m_io_service_work(m_io_service)
{
    // used for async_read_until to frame read operations
    m_magic_word_buf.resize( 4 );
    *reinterpret_cast<uint32_t*>( &(m_magic_word_buf[0]) ) = htonl( MagicWordValue );
}


LUX::~LUX()
{
}


bool LUX::connect( const std::string& device_addr, unsigned short port )
{
    _start_io_service_threads();

    std::string port_str;
    try
    {
       port_str = boost::lexical_cast<std::string>( port );
    }
    catch( boost::bad_lexical_cast& )
    {
        m_last_error= "Unable to convert port to string.";
        return false;
    }

    tcp::resolver::query query( device_addr, port_str );

    boost::system::error_code ec;
    tcp::resolver::iterator endpoint_it= m_resolver.resolve( query, ec );

    if( ec )
    {
        m_last_error= ec.message();
        return false;
    }

    boost::asio::connect( m_socket, endpoint_it, ec );

    if( ec )
    {
        m_last_error= ec.message();
        return false;
    }

    // always have an outstanding read ready
    _start_read_header_magic();

    // now that we're connected, resync with the device
    // to make sure all the messages are being framed correctly.
    // do this by sending a reset then asking for a device status message
    // if all that works, then we're good to go and and begin
    // normal async operations
    if( !reset() )
    {
        return false;
    }

    return true;
}


void LUX::disconnect()
{
    if( m_socket.is_open() )
    {
        m_socket.close();
    }
}


bool LUX::reset()
{
    _make_cmd_reset( m_send_buf );
    _start_write_cmd();

    return true;
}


const std::string& LUX::last_error()
{
    return m_last_error;
}


void LUX::register_error_warning_handler( ErrorWarningHandler ew_handler )
{
    boost::mutex::scoped_lock lock(m_ew_handler_mutex);

    m_ew_handler= ew_handler;
}

void LUX::register_scan_points_handler( ScanPointsHandler points_handler )
{
    boost::mutex::scoped_lock lock(m_points_handler_mutex);

    m_points_handler= points_handler;
}

void LUX::register_scan_objects_handler( ScanObjectsHandler objects_handler )
{
    boost::mutex::scoped_lock lock(m_objects_handler_mutex);

    m_objects_handler= objects_handler;
}

// -----------------------------

void LUX::_io_service_run()
{
    for (;;)
    {
        try
        {
            m_io_service.run();
            break; // run() exited normally, don't continue
        }
        catch( std::exception& e )
        {
        }
    }
}


void LUX::_start_io_service_threads()
{
    if( m_io_service_thread_pool.size() > 0 )
        return;

    m_io_service_thread_pool.resize( IOSERVICE_THREAD_POOL_SIZE );

    for( unsigned int i= 0; i < IOSERVICE_THREAD_POOL_SIZE; ++i )
    {
        m_io_service_thread_pool[i].reset( new boost::thread(
                    boost::bind( &LUX::_io_service_run, shared_from_this() ) ) );
    }
}


void LUX::_start_read_header_magic()
{
    boost::asio::async_read_until( m_socket, m_response_buf, m_magic_word_buf,
        boost::bind( &LUX::_handle_read_header_magic, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ));

}


void LUX::_handle_read_header_magic( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_header_magic\n" );
        return;
    }

    printf( "_handle_read_header_magic complete with %zu bytes\n", bytes_transferred );

    _start_read_header();
}

void LUX::_start_read_header()
{
    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(MessageHeader)),
        boost::bind( &LUX::_handle_read_header, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );

}


void LUX::_handle_read_header( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_header_magic\n" );
        return;
    }

    printf( "_handle_read_header complete with %zu bytes\n", bytes_transferred );

    const MessageHeader* hdr_buf= boost::asio::buffer_cast<const MessageHeader*>( m_response_buf.data() );
    m_current_header= *hdr_buf;
    m_response_buf.consume( sizeof(MessageHeader) );

    _ntoh_MessageHeader( m_current_header );

    printf( "incoming message data type %x size %d\n", m_current_header.DataType, m_current_header.MessageSize );

    if( m_current_header.MessageSize == 0 )
    {
        // no more data for this one, look for the next header
        _start_read_header_magic();
    }

    // start a data read depending on the header
    switch( m_current_header.DataType )
    {
    case ScanData:
        _start_read_scan_data_header();
        break;

    case ObjectData:
        _start_read_object_data_header();
        break;

    case ErrorWarningData:
        _start_read_error_warning_data();
        break;

    default:
        // unknown data type. consume the message data,
        // report error and go back to reading headers
        break;
    }
}


void LUX::_start_write_cmd()
{
    boost::asio::async_write( m_socket, boost::asio::buffer(m_send_buf),
                            boost::bind( &LUX::_handle_write_cmd, shared_from_this(),
                                        boost::asio::placeholders::error ));
}


void LUX::_handle_write_cmd( const boost::system::error_code& err )
{
    if( err )
    {
    }
}


void LUX::_start_read_scan_data_header()
{
    printf( "starting scan data header read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ScanDataHeader)),
        boost::bind( &LUX::_handle_read_scan_data_header, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_scan_data_header( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_scan_data\n" );
        return;
    }

    printf( "_handle_read_scan_data_header complete with %zu bytes\n", bytes_transferred );

    const ScanDataHeader* hdr_buf= boost::asio::buffer_cast<const ScanDataHeader*>( m_response_buf.data() );
    m_current_scan_data.Header= *hdr_buf;
    m_response_buf.consume( sizeof(ScanDataHeader) );

    printf( "ScanNumber: %d\n", m_current_scan_data.Header.ScanNumber );
    printf( "ScanPointCount: %d\n", m_current_scan_data.Header.ScanPointCount );

    // sanity check
    if( m_current_scan_data.Header.ScanPointCount * sizeof(ScanPoint) + \
       sizeof(ScanDataHeader) != m_current_header.MessageSize )
    {
        printf( "unexpected length in scan data.\n" );

        // consume the rest, go on
        return;
    }

    // start to read scan points
    _start_read_scan_points();
}


void LUX::_start_read_scan_points()
{
    printf( "starting scan data header read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ScanPoint)*m_current_scan_data.Header.ScanPointCount),
        boost::bind( &LUX::_handle_read_scan_points, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_scan_points( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_scan_points\n" );
        return;
    }

    printf( "_handle_read_scan_points complete with %zu bytes (%d points)\n",
           bytes_transferred, m_current_scan_data.Header.ScanPointCount );

    const ScanPoint* point_buf= boost::asio::buffer_cast<const ScanPoint*>( m_response_buf.data() );
    m_current_scan_data.Points.resize( m_current_scan_data.Header.ScanPointCount );
    memcpy( &m_current_scan_data.Points[0], point_buf, sizeof(ScanPoint)*m_current_scan_data.Header.ScanPointCount );

    m_response_buf.consume( sizeof(ScanPoint)*m_current_scan_data.Header.ScanPointCount );

    // make a copy for the callback
    ScanDataPoints pts_copy= m_current_scan_data;

    // go back to the beginning
    _start_read_header_magic();

    {
        boost::mutex::scoped_lock lock(m_points_handler_mutex);

        if( m_points_handler )
        {
            m_points_handler( pts_copy );
        }
    }
}


void LUX::_start_read_error_warning_data( )
{
    printf( "starting error warning data read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ErrorWarning)),
        boost::bind( &LUX::_handle_read_error_warning_data, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_error_warning_data( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_error_warning_data\n" );
        return;
    }

    printf( "_handle_read_error_warning_data complete with %zu bytes\n", bytes_transferred );

    const ErrorWarning* err_warn_buf= boost::asio::buffer_cast<const ErrorWarning*>( m_response_buf.data() );

    ErrorWarning err_warn;
    memcpy( &err_warn, err_warn_buf, sizeof(ErrorWarning) );

    m_response_buf.consume( sizeof(ErrorWarning) );

    // go back to the beginning
    _start_read_header_magic();

    {
        boost::mutex::scoped_lock lock(m_ew_handler_mutex);

        if( m_ew_handler )
        {
            m_ew_handler( err_warn );
        }
    }
}


void LUX::_start_read_object_data_header()
{
    printf( "starting object data header read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ObjectDataHeader)),
        boost::bind( &LUX::_handle_read_object_data_header, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_object_data_header( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_object_data_header\n" );
        return;
    }


    printf( "_handle_read_object_data_header complete with %zu bytes\n", bytes_transferred );

    const ObjectDataHeader* hdr_buf= boost::asio::buffer_cast<const ObjectDataHeader*>( m_response_buf.data() );
    m_current_objects_data.Header= *hdr_buf;

    m_response_buf.consume( sizeof(ObjectDataHeader) );

    m_remaining_objects= m_current_objects_data.Header.ObjectCount;
    printf( "Object Count: %d\n", m_current_objects_data.Header.ObjectCount );

    // start to read object data
    _start_read_single_object_data();
}


void LUX::_start_read_single_object_data()
{
    // read just one object, we don't know how many contour points
    // it has until we read it. so do this over for all objects one at a time

    printf( "starting single object data read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ObjectData)),
        boost::bind( &LUX::_handle_read_single_object_data, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_single_object_data( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_single_object_data\n" );
        return;
    }

    printf( "_handle_read_single_object_data complete with %zu bytes\n", bytes_transferred );

    const Object* data_buf= boost::asio::buffer_cast<const Object*>( m_response_buf.data() );
    m_current_object.Obj= *data_buf;

    m_response_buf.consume( sizeof(Object) );

    printf( "Object Contour Count: %d\n", m_current_object.Obj.ContourPointCount );

    _start_read_object_data_contours();
}


void LUX::_start_read_object_data_contours()
{
    printf( "starting object data contour point read.\n" );

    boost::asio::async_read( m_socket, m_response_buf,
                            boost::asio::transfer_at_least(sizeof(Point2D)*m_current_object.Obj.ContourPointCount),
                    boost::bind( &LUX::_handle_read_object_data_contours, shared_from_this(),
                        boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_object_data_contours( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        printf( "error in _handle_read_object_data_contours\n" );
        return;
    }

    // this is the last bit of the current object
    m_remaining_objects--;

    printf( "_handle_read_object_data_contours complete with %zu bytes\n", bytes_transferred );

    const Point2D* point_buf= boost::asio::buffer_cast<const Point2D*>( m_response_buf.data() );
    m_current_object.Contour.resize( m_current_object.Obj.ContourPointCount );
    memcpy( &m_current_object.Contour[0], point_buf, sizeof(Point2D)*m_current_object.Obj.ContourPointCount );

    m_current_objects_data.Objects.push_back( m_current_object );

    m_response_buf.consume( sizeof(Point2D)*m_current_object.Obj.ContourPointCount );

    // if this was the contours for the last object, go back to reading the next header
    if( m_remaining_objects > 0 )
    {
        // get the next object
        _start_read_single_object_data();
        return;
    }

    // make a copy for the callback
    ScanDataObjects objects_cpy= m_current_objects_data;


    _start_read_header_magic();

    {
        boost::mutex::scoped_lock lock(m_objects_handler_mutex);

        if( m_objects_handler )
        {
            m_objects_handler( objects_cpy );
        }
    }


}


// build as network endian ready for transport
void LUX::_make_cmd_header( std::vector<char>& buf, uint32_t message_data_size )
{
    buf.resize( 0 );

    buf.push_back( sizeof(MessageHeader) );
    MessageHeader& h= *reinterpret_cast<MessageHeader*>( &buf[0] );

    h.MagicWord = htonl(MagicWordValue);
    h.PreviousMessagesSize = 0;
    h.MessageSize = htonl(message_data_size);
    h.Reserved = 0;
    h.DeviceID = 0;
    h.DataType = htons(CommandData);
    h.NTPTime = 0;
}

// no args
void LUX::_make_cmd_reset( std::vector<char>& buf )
{
    const int cmd_size= 4;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::Reset;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
}

// no args
void LUX::_make_cmd_get_status( std::vector<char>& buf )
{
    const int cmd_size= 4;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::GetStatus;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
}

// no args
void LUX::_make_cmd_start_measure( std::vector<char>& buf )
{
    const int cmd_size= 4;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::StartMeasure;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
}

// no args
void LUX::_make_cmd_stop_measure( std::vector<char>& buf )
{
    const int cmd_size= 4;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::StopMeasure;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
}


// no args
void LUX::_make_cmd_save_config( std::vector<char>& buf )
{
    const int cmd_size= 4;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::SaveConfig;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
}


void LUX::_make_cmd_set_parameter( std::vector<char>& buf, uint16_t parameter_index, uint32_t parameter )
{
    const int cmd_size= 10;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::SetParameter;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
    *reinterpret_cast<uint16_t*>(&buf[4])= parameter_index;
    *reinterpret_cast<uint32_t*>(&buf[6])= parameter;
}


void LUX::_make_cmd_get_parameter( std::vector<char>& buf, uint16_t parameter_index )
{
    const int cmd_size= 6;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::GetParameter;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
    *reinterpret_cast<uint16_t*>(&buf[4])= parameter_index;
}

// no args
void LUX::_make_cmd_reset_default_parameters( std::vector<char>& buf )
{
    const int cmd_size= 4;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::ResetDefaultParameters;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
}

void LUX::_make_cmd_set_ntp_timestamp_sync( std::vector<char>& buf, uint32_t seconds, uint32_t fractional_seconds )
{
    const int cmd_size= 14;

    _make_cmd_header( buf, cmd_size );
    buf.insert( buf.end(), cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[0])= Command::SetNTPTimestampSync;
    *reinterpret_cast<uint16_t*>(&buf[2])= 0;                 // reserved
    *reinterpret_cast<uint16_t*>(&buf[4])= 0;                 // reserved
    *reinterpret_cast<uint32_t*>(&buf[6])= seconds;
    *reinterpret_cast<uint32_t*>(&buf[10])= fractional_seconds;
}


// convert a network byte order header to host byte order
void LUX::_ntoh_MessageHeader( MessageHeader& h )
{
    h.PreviousMessagesSize= ntohl( h.PreviousMessagesSize );
    h.MessageSize= ntohl( h.MessageSize );
    h.DataType= ntohs( h.DataType );
    h.NTPTime= htobe64( h.NTPTime );
}

}
