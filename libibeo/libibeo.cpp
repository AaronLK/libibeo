#include "libibeo.h"

/*

libibeo: client library to interface with ibeo LUX laser scanners


Copyright 2014, Aaron Klingaman, Limitedslip Engineering

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
    m_current_scan_data_header= *hdr_buf;
    m_response_buf.consume( sizeof(ScanDataHeader) );

    printf( "ScanNumber: %d\n", m_current_scan_data_header.ScanNumber );
    printf( "ScanPointCount: %d\n", m_current_scan_data_header.ScanPointCount );

    // sanity check
    if( m_current_scan_data_header.ScanPointCount * sizeof(ScanPoint) + \
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

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ScanPoint)*m_current_scan_data_header.ScanPointCount),
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

    printf( "_handle_read_scan_points complete with %zu bytes (%d points)\n", bytes_transferred, m_current_scan_data_header.ScanPointCount );

    const ScanPoint* point_buf= boost::asio::buffer_cast<const ScanPoint*>( m_response_buf.data() );
    m_current_scan_points.resize( m_current_scan_data_header.ScanPointCount );
    memcpy( &m_current_scan_points[0], point_buf, sizeof(ScanPoint)*m_current_scan_data_header.ScanPointCount );

    m_response_buf.consume( sizeof(ScanPoint)*m_current_scan_data_header.ScanPointCount );

    for( size_t i= 0; i < 10; ++i )
        printf( "got scan point angle/distance: %d %d\n", m_current_scan_points[i].HorizontalAngle,
               m_current_scan_points[i].RadialDistance );

    // go back to the beginning
    _start_read_header_magic();
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


// convert a network byte order header to host byte order
void LUX::_ntoh_MessageHeader( MessageHeader& h )
{
    h.PreviousMessagesSize= ntohl( h.PreviousMessagesSize );
    h.MessageSize= ntohl( h.MessageSize );
    h.DataType= ntohs( h.DataType );
    h.NTPTime= htobe64( h.NTPTime );
}

}
