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

    return true;
}


void LUX::disconnect()
{
    if( m_socket.is_open() )
    {
        m_socket.close();
    }
}


CommandResultPtr LUX::reset()
{
    CommandResultPtr cmd( new CommandResult(Reset) );

    _make_cmd_reset( cmd->cmd_buf() );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::get_status()
{
    CommandResultPtr cmd( new CommandResult(GetStatus) );

    _make_cmd_get_status( cmd->cmd_buf() );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::save_config()
{
    CommandResultPtr cmd( new CommandResult(SaveConfig) );

    _make_cmd_save_config( cmd->cmd_buf() );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::set_parameter( uint16_t index, uint32_t value )
{
    CommandResultPtr cmd( new CommandResult(SetParameter) );

    _make_cmd_set_parameter( cmd->cmd_buf(), index, value );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::get_parameter( uint16_t index )
{
    CommandResultPtr cmd( new CommandResult(GetParameter) );

    _make_cmd_get_parameter( cmd->cmd_buf(), index );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::reset_default_parameters()
{
    CommandResultPtr cmd( new CommandResult(ResetDefaultParameters) );

    _make_cmd_reset_default_parameters( cmd->cmd_buf() );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::start_measure()
{
    CommandResultPtr cmd( new CommandResult(StartMeasure) );

    _make_cmd_start_measure( cmd->cmd_buf() );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::stop_measure()
{
    CommandResultPtr cmd( new CommandResult(StopMeasure) );

    _make_cmd_stop_measure( cmd->cmd_buf() );
    _enqueue_cmd( cmd );

    return cmd;
}


CommandResultPtr LUX::set_ntp_timestamp_sync( uint32_t seconds, uint32_t fractional_seconds )
{
    CommandResultPtr cmd( new CommandResult(SetNTPTimestampSync) );

    _make_cmd_set_ntp_timestamp_sync( cmd->cmd_buf(), seconds, fractional_seconds );
    _enqueue_cmd( cmd );

    return cmd;
}


const std::string& LUX::last_error()
{
    return m_last_error;
}


float LUX::status_temp_to_C( uint16_t temp )
{
    // from the datasheet
    return - ( (float)temp - 579.2364 ) / 3.63;
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
        //printf( "error in _handle_read_header_magic\n" );
        return;
    }

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
        //printf( "error in _handle_read_header_magic\n" );
        return;
    }

    const MessageHeader* hdr_buf= boost::asio::buffer_cast<const MessageHeader*>( m_response_buf.data() );
    m_current_header= *hdr_buf;
    m_response_buf.consume( sizeof(MessageHeader) );

    _ntoh_MessageHeader( m_current_header );

    //printf( "incoming message data type %x size %d\n", m_current_header.DataType, m_current_header.MessageSize );

    if( m_current_header.MessageSize == 0 )
    {
        // no more data for this one, look for the next header
        _start_read_header_magic();
        return;
    }

    // start a data read depending on the header
    switch( m_current_header.DataType )
    {
    case ReplyData:
        _start_read_cmd_reply();
        break;

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
        printf( "error: unknown data type: 0x%04x.\n", m_current_header.DataType );
        break;
    }
}


bool LUX::_enqueue_cmd( CommandResultPtr cmd )
{
    if( !cmd )
        return false;

    {
        boost::mutex::scoped_lock lock( m_commands_mutex );
        m_outgoing_commands.push( cmd );
    }

    _start_write_next_cmd();

    return true;
}

void LUX::_start_write_next_cmd()
{
    {
        boost::mutex::scoped_lock lock( m_commands_mutex );

        // if one is is the works already, or none, don't do anything
        if( m_current_command || m_outgoing_commands.size() == 0 )
        {
            return;
        }

        m_current_command= m_outgoing_commands.front();
        m_outgoing_commands.pop();
    }

    if( !m_current_command )
    {
        printf( "internal error: m_current_command invalid.\n" );
        return;
    }

    boost::asio::async_write( m_socket, boost::asio::buffer(m_current_command->cmd_buf()),
                            boost::bind( &LUX::_handle_write_cmd, shared_from_this(),
                                        boost::asio::placeholders::error ));
}


void LUX::_handle_write_cmd( const boost::system::error_code& err )
{
    if( err )
    {
        return;
    }

    {
        boost::mutex::scoped_lock lock( m_commands_mutex );

        // the reset command never gets data back, so short cut and notify the caller
        // it is done
        if( m_current_command->command() == Reset )
        {
            m_current_command->m_error= false;
            m_current_command->notify_complete();
        }
        else
        {
            // there is a response of some sort, deal with it after we get the
            // response back
            m_sent_commands.push( m_current_command );
        }

        m_current_command.reset();
    }

    _start_write_next_cmd();
}


void LUX::_start_read_cmd_reply()
{
    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(uint16_t)),
        boost::bind( &LUX::_handle_read_cmd_reply, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_cmd_reply( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_cmd_reply\n" );
        return;
    }

    //printf( "_handle_read_cmd_reply complete with %zu bytes\n", bytes_transferred );

    const uint16_t* buf= boost::asio::buffer_cast<const uint16_t*>( m_response_buf.data() );
    uint16_t reply_id= *buf;
    m_response_buf.consume( sizeof(uint16_t) );

    Command reply_cmd= static_cast<Command>( reply_id & 0x7FFF );
    uint16_t cmd_success= (reply_id & 0x8000) == 0;

    //printf( "cmd id %d returned %d success (0x%04x response data)\n", reply_cmd, cmd_success, reply_id );

    CommandResultPtr cmd_result;

    {
        boost::mutex::scoped_lock lock();

        // since the device itself is synchronous, this should
        // correspond with what we just received
        cmd_result= m_sent_commands.front();

        if( cmd_result->command() != reply_cmd )
        {
            printf( "got out of sync reply.\n" );
        }

        cmd_result->m_error= !cmd_success;

        if( cmd_result )
        {
            switch( reply_cmd )
            {
                case SaveConfig:
                case SetParameter:
                case ResetDefaultParameters:
                case StartMeasure:
                case StopMeasure:
                case SetNTPTimestampSync:
                    // no expected response or reply data
                    m_sent_commands.pop();
                    break;

                // have more data to read before this command
                // can be done.

                case GetStatus:
                    _start_read_status();
                    return;

                case GetParameter:
                    _start_read_parameter();
                    return;

                default:
                    break;
            }
        }
    }

    // go back to the beginning
    _start_read_header_magic();

    // notify the caller
    cmd_result->notify_complete();
}


void LUX::_start_read_scan_data_header()
{
    //printf( "starting scan data header read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ScanDataHeader)),
        boost::bind( &LUX::_handle_read_scan_data_header, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_scan_data_header( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_scan_data\n" );
        return;
    }

    //printf( "_handle_read_scan_data_header complete with %zu bytes\n", bytes_transferred );

    const ScanDataHeader* hdr_buf= boost::asio::buffer_cast<const ScanDataHeader*>( m_response_buf.data() );
    m_current_scan_data.Header= *hdr_buf;
    m_response_buf.consume( sizeof(ScanDataHeader) );

    //printf( "ScanNumber: %d\n", m_current_scan_data.Header.ScanNumber );
    //printf( "ScanPointCount: %d\n", m_current_scan_data.Header.ScanPointCount );

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


void LUX::_start_read_status()
{
    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(StatusMessage)),
        boost::bind( &LUX::_handle_read_status, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_status( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_status\n" );
        return;
    }

    //printf( "_handle_read_status complete with %zu bytes \n", bytes_transferred );

    StatusMessage status_buf= *boost::asio::buffer_cast<const StatusMessage*>( m_response_buf.data() );
    m_response_buf.consume( sizeof(StatusMessage) );

    CommandResultPtr cmd_result;

    {
        boost::mutex::scoped_lock lock();

        // since the device itself is synchronous, this should
        // correspond with what we just received
        cmd_result= m_sent_commands.front();
        m_sent_commands.pop();
    }

    // go back to the beginning
    _start_read_header_magic();

    // and notify the caller
    cmd_result->set_response_status( status_buf );
    cmd_result->notify_complete();
}

void LUX::_start_read_parameter()
{
    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(Parameter)),
        boost::bind( &LUX::_handle_read_parameter, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_parameter( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_parameter\n" );
        return;
    }

    // printf( "_handle_read_status complete with %zu bytes \n", bytes_transferred );

    Parameter param_buf= *boost::asio::buffer_cast<const Parameter*>( m_response_buf.data() );
    m_response_buf.consume( sizeof(Parameter) );

    CommandResultPtr cmd_result;

    {
        boost::mutex::scoped_lock lock();

        // since the device itself is synchronous, this should
        // correspond with what we just received
        cmd_result= m_sent_commands.front();
        m_sent_commands.pop();
    }

    // go back to the beginning
    _start_read_header_magic();

    // and notify the caller
    cmd_result->set_response_parameter( param_buf );
    cmd_result->notify_complete();
}

void LUX::_start_read_scan_points()
{
    //printf( "starting scan data header read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ScanPoint)*m_current_scan_data.Header.ScanPointCount),
        boost::bind( &LUX::_handle_read_scan_points, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_scan_points( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_scan_points\n" );
        return;
    }

    //printf( "_handle_read_scan_points complete with %zu bytes (%d points)\n",
    //       bytes_transferred, m_current_scan_data.Header.ScanPointCount );

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
    //printf( "starting error warning data read.\n" );

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ErrorWarning)),
        boost::bind( &LUX::_handle_read_error_warning_data, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_error_warning_data( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_error_warning_data\n" );
        return;
    }

    //printf( "_handle_read_error_warning_data complete with %zu bytes\n", bytes_transferred );

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
    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ObjectDataHeader)),
        boost::bind( &LUX::_handle_read_object_data_header, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_object_data_header( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_object_data_header\n" );
        return;
    }


    //printf( "_handle_read_object_data_header complete with %zu bytes\n", bytes_transferred );

    const ObjectDataHeader* hdr_buf= boost::asio::buffer_cast<const ObjectDataHeader*>( m_response_buf.data() );
    m_current_objects_data.Header= *hdr_buf;

    m_response_buf.consume( sizeof(ObjectDataHeader) );

    m_remaining_objects= m_current_objects_data.Header.ObjectCount;
    //printf( "Object Count: %d\n", m_current_objects_data.Header.ObjectCount );

    // start to read object data
    _start_read_single_object_data();
}


void LUX::_start_read_single_object_data()
{
    // read just one object, we don't know how many contour points
    // it has until we read it. so do this over for all objects one at a time

    boost::asio::async_read( m_socket, m_response_buf, boost::asio::transfer_at_least(sizeof(ObjectData)),
        boost::bind( &LUX::_handle_read_single_object_data, shared_from_this(),
                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_single_object_data( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_single_object_data\n" );
        return;
    }

    //printf( "_handle_read_single_object_data complete with %zu bytes\n", bytes_transferred );

    const Object* data_buf= boost::asio::buffer_cast<const Object*>( m_response_buf.data() );
    m_current_object.Obj= *data_buf;

    m_response_buf.consume( sizeof(Object) );

    //printf( "Object Contour Count: %d\n", m_current_object.Obj.ContourPointCount );

    _start_read_object_data_contours();
}


void LUX::_start_read_object_data_contours()
{
    boost::asio::async_read( m_socket, m_response_buf,
                            boost::asio::transfer_at_least(sizeof(Point2D)*m_current_object.Obj.ContourPointCount),
                    boost::bind( &LUX::_handle_read_object_data_contours, shared_from_this(),
                        boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
}


void LUX::_handle_read_object_data_contours( const boost::system::error_code& err, std::size_t bytes_transferred )
{
    if( err )
    {
        //printf( "error in _handle_read_object_data_contours\n" );
        return;
    }

    // this is the last bit of the current object
    m_remaining_objects--;

    //printf( "_handle_read_object_data_contours complete with %zu bytes\n", bytes_transferred );

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
size_t LUX::_make_cmd_header( std::vector<char>& buf, uint32_t message_data_size )
{
    buf.resize( sizeof(MessageHeader) + message_data_size );
    MessageHeader& h= *reinterpret_cast<MessageHeader*>( &buf[0] );

    h.MagicWord = htonl(MagicWordValue);
    h.PreviousMessagesSize = 0;
    h.MessageSize = htonl( message_data_size );
    h.Reserved = 0;
    h.DeviceID = 0;
    h.DataType = htons(CommandData);
    h.NTPTime = 0;

    return sizeof(MessageHeader);
}

// no args
void LUX::_make_cmd_reset( std::vector<char>& buf )
{
    const int cmd_size= 4;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::Reset;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
}

// no args
void LUX::_make_cmd_get_status( std::vector<char>& buf )
{
    const int cmd_size= 4;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::GetStatus;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
}

// no args
void LUX::_make_cmd_start_measure( std::vector<char>& buf )
{
    const int cmd_size= 4;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::StartMeasure;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
}

// no args
void LUX::_make_cmd_stop_measure( std::vector<char>& buf )
{
    const int cmd_size= 4;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::StopMeasure;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
}


// no args
void LUX::_make_cmd_save_config( std::vector<char>& buf )
{
    const int cmd_size= 4;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::SaveConfig;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
}


void LUX::_make_cmd_set_parameter( std::vector<char>& buf, uint16_t parameter_index, uint32_t parameter )
{
    const int cmd_size= 10;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::SetParameter;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
    *reinterpret_cast<uint16_t*>(&buf[end_pos+4])= parameter_index;
    *reinterpret_cast<uint32_t*>(&buf[end_pos+6])= parameter;
}


void LUX::_make_cmd_get_parameter( std::vector<char>& buf, uint16_t parameter_index )
{
    const int cmd_size= 6;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::GetParameter;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
    *reinterpret_cast<uint16_t*>(&buf[end_pos+4])= parameter_index;
}

// no args
void LUX::_make_cmd_reset_default_parameters( std::vector<char>& buf )
{
    const int cmd_size= 4;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::ResetDefaultParameters;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
}

void LUX::_make_cmd_set_ntp_timestamp_sync( std::vector<char>& buf, uint32_t seconds, uint32_t fractional_seconds )
{
    const int cmd_size= 14;

    size_t end_pos= _make_cmd_header( buf, cmd_size );

    *reinterpret_cast<uint16_t*>(&buf[end_pos+0])= Command::SetNTPTimestampSync;
    *reinterpret_cast<uint16_t*>(&buf[end_pos+2])= 0;                 // reserved
    *reinterpret_cast<uint16_t*>(&buf[end_pos+4])= 0;                 // reserved
    *reinterpret_cast<uint32_t*>(&buf[end_pos+6])= seconds;
    *reinterpret_cast<uint32_t*>(&buf[end_pos+10])= fractional_seconds;
}


// convert a network byte order header to host byte order
void LUX::_ntoh_MessageHeader( MessageHeader& h )
{
    h.PreviousMessagesSize= ntohl( h.PreviousMessagesSize );
    h.MessageSize= ntohl( h.MessageSize );
    h.DataType= ntohs( h.DataType );
    h.NTPTime= htobe64( h.NTPTime );
}


// --------------

CommandResult::CommandResult(  Command cmd  ) : m_command(cmd)
{
    m_complete= false;
    m_error= true;

    memset( &m_response_status, 0, sizeof(m_response_status) );
    memset( &m_response_parameter, 0, sizeof(Parameter) );
}

std::vector<char>& CommandResult::cmd_buf()
{
    return m_command_data;
}


void CommandResult::notify_complete()
{
    boost::mutex::scoped_lock lock( m_mutex );

    m_complete= true;

    m_changed_cond.notify_all();
}


void CommandResult::set_response_status( const StatusMessage& status )
{
    boost::mutex::scoped_lock lock( m_mutex );

    m_response_status= status;
}

void CommandResult::set_response_parameter( const Parameter& param )
{
    boost::mutex::scoped_lock lock( m_mutex );

    m_response_parameter= param;
}


void CommandResult::wait()
{
    boost::mutex::scoped_lock lock( m_mutex );

    if( m_complete )
        return;

    m_changed_cond.wait( lock );
    return;
}


void CommandResult::wait( boost::posix_time::time_duration timeout )
{
    boost::mutex::scoped_lock lock( m_mutex );

    if( m_complete )
        return;

    m_changed_cond.timed_wait( lock, timeout );
}


bool CommandResult::is_complete()
{
    boost::mutex::scoped_lock lock( m_mutex );

    return m_complete;
}


bool CommandResult::is_error()
{
    boost::mutex::scoped_lock lock( m_mutex );

    return m_error;
}

// response data

Command CommandResult::command()
{
    return m_command;
}

const StatusMessage& CommandResult::response_status()
{
    return m_response_status;
}

const Parameter& CommandResult::response_parameter()
{
    return m_response_parameter;
}

}
