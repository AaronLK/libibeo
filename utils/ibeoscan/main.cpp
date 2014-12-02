/*

ibeoscan: utility to interface with ibeo LUX scanners using libibeo


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


#include <libibeo.h>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


void handle_error_warning( const ibeo::ErrorWarning& ew );
void handle_scan_points( const ibeo::ScanDataPoints& points );
void handle_scan_objects( const ibeo::ScanDataObjects& objects );


int main( int argc, char** argv )
{
    ibeo::LUXptr dev( new ibeo::LUX() );

    dev->register_error_warning_handler( boost::bind( &handle_error_warning, ibeo::placeholders::ew ) );

    printf( "Connecting to scanner.\n" );

    std::string ip_str= "172.16.28.75";
    unsigned int port= 12002;
    if( !dev->connect( ip_str, port ) )
    {
        fprintf( stderr, "Unable to connect to %s:%d: %s\n", ip_str.c_str(), port, dev->last_error().c_str() );
        return 1;
    }

    printf( "Connected.\n" );

    ibeo::CommandResultPtr cmd;

    // get status doesn't seem to work when it isn't scanning
    printf( "Starting measuring (can take a few seconds)\n" );

    cmd= dev->start_measure();
    cmd->wait( boost::posix_time::seconds(10) );

    if( !cmd->is_complete() || cmd->is_error() )
    {
        fprintf( stderr, "Unable to start scan.\n" );
        dev->disconnect();
        return 1;
    }

    // sanity check, get the IP and TCP port out of it.

    cmd= dev->get_parameter( ibeo::TCPPort );
    cmd->wait( boost::posix_time::seconds(10) );

    if( !cmd->is_complete() || cmd->is_error() )
    {
        fprintf( stderr, "Unable to get parameter.\n" );
        dev->disconnect();
        return 1;
    }

    uint16_t tcp_port= cmd->response_parameter().Value;

    cmd= dev->get_parameter( ibeo::IPAddress );
    cmd->wait( boost::posix_time::seconds(10) );

    if( !cmd->is_complete() || cmd->is_error() )
    {
        fprintf( stderr, "Unable to get parameter.\n" );
        dev->disconnect();
        return 1;
    }

    in_addr ip;
    ip.s_addr= htonl(cmd->response_parameter().Value);
    printf( "Scanner is configured to output data on TCP addr: %s port %d, \n", inet_ntoa(ip), tcp_port );


    // enable all ethernet data output. see the datasheet for info.
    cmd= dev->set_parameter( ibeo::DataOutputFlag, 0 );
    cmd->wait( boost::posix_time::seconds(10) );

    if( !cmd->is_complete() || cmd->is_error() )
    {
        fprintf( stderr, "Unable to set parameter.\n" );
        dev->disconnect();
        return 1;
    }

    // figure out what data is being output
    cmd= dev->get_parameter( ibeo::DataOutputFlag );
    cmd->wait( boost::posix_time::seconds(10) );

    if( !cmd->is_complete() || cmd->is_error() )
    {
        fprintf( stderr, "Unable to get parameter.\n" );
        dev->disconnect();
        return 1;
    }

    printf( "Scanner is configured to output data for (0x%04x): ", cmd->response_parameter().Value );

    if( !(cmd->response_parameter().Value & (1<<0)) )
        printf( "scan " );
    if( !(cmd->response_parameter().Value & (1<<2)) )
        printf( "object " );
    if( !(cmd->response_parameter().Value & (1<<3)) )
        printf( "vehicle " );
    if( !(cmd->response_parameter().Value & (1<<4)) )
        printf( "errwarn " );
    printf( "\n" );

    // get the current scanner status
    cmd= dev->get_status();
    cmd->wait( boost::posix_time::seconds(5) );

    if( !cmd->is_complete() || cmd->is_error() )
    {
        fprintf( stderr, "Timeout while waiting for scanner status.\n" );
        dev->disconnect();
        return 1;
    }

    printf( "ibeo LUX scanner details:\n" );
    printf( "Firmware version: 0x%04x\n", cmd->response_status().FirmwareVersion );
    printf( "FPGA version: 0x%04x\n", cmd->response_status().FPGAVersion );
    printf( "Temperature: %3.2f\n", ibeo::LUX::status_temp_to_C(cmd->response_status().Temperature) );
    printf( "Serial: %04x %04x\n", cmd->response_status().SerialNumber0, cmd->response_status().SerialNumber1 );
    printf( "Scanner status: %04x\n", cmd->response_status().ScannerStatus );

    // register handlers for incoming data
    dev->register_scan_points_handler( boost::bind( &handle_scan_points, ibeo::placeholders::points ) );
    dev->register_scan_objects_handler( boost::bind( &handle_scan_objects, ibeo::placeholders::objects ) );


    uint16_t run_time_s= 30;
    printf( "Running for %d seconds.\n", run_time_s );
    boost::this_thread::sleep( boost::posix_time::seconds(run_time_s) );

    printf( "Stopping scanner.\n" );
    cmd= dev->stop_measure();
    cmd->wait( boost::posix_time::seconds(10) );

    if( cmd->is_error() )
    {
        fprintf( stderr, "Unable to stop scan.\n" );
        dev->disconnect();
        return 1;
    }

    printf( "Completed test, disconnecting.\n" );
    dev->disconnect();

    return 0;
}


void handle_error_warning( const ibeo::ErrorWarning& ew )
{
    printf( "got error warning (see datasheet):\n" );
    printf( "ErrorRegister1: 0x%04x\n", ew.ErrorRegister1 );
    printf( "ErrorRegister1: 0x%04x\n", ew.ErrorRegister2 );
    printf( "WarningRegister1: 0x%04x\n", ew.WarningRegister1 );
    printf( "WarningRegister2: 0x%04x\n", ew.WarningRegister2 );

}

void handle_scan_points( const ibeo::ScanDataPoints& points )
{
    static uint64_t count= 0;
    static uint64_t point_count= 0;

    count++;
    point_count += points.Points.size();

    if( count % 100 == 0 )
    {
        printf( "Total point count received: %zu\n", point_count );

        printf( "Last 2 points:\n" );

        for( size_t i= points.Points.size() - 2; i < points.Points.size(); ++i )
            printf( "angle, distance: %d, %d\n", points.Points[i].HorizontalAngle, points.Points[i].RadialDistance );
    }
}

void handle_scan_objects( const ibeo::ScanDataObjects& objects )
{
    printf( "got objects\n" );
}



