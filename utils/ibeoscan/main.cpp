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


void handle_error_warning( const ibeo::ErrorWarning& ew );
void handle_scan_points( const ibeo::ScanDataPoints& points );
void handle_scan_objects( const ibeo::ScanDataObjects& objects );


int main( int argc, char** argv )
{
    ibeo::LUXptr dev( new ibeo::LUX() );

    dev->register_error_warning_handler( boost::bind( &handle_error_warning, ibeo::placeholders::ew ) );
    dev->register_scan_points_handler( boost::bind( &handle_scan_points, ibeo::placeholders::points ) );
    dev->register_scan_objects_handler( boost::bind( &handle_scan_objects, ibeo::placeholders::objects ) );

    std::string ip= "172.16.28.75";
    unsigned int port= 12002;
    if( !dev->connect( ip, port ) )
    {
        fprintf( stderr, "Unable to connect to %s:%d: %s\n", ip.c_str(), port, dev->last_error().c_str() );
        return 1;
    }

    printf( "Connected.\n" );

    boost::this_thread::sleep( boost::posix_time::seconds(30) );


    printf( "Disconnecting.\n" );
    dev->disconnect();

    return 0;
}


void handle_error_warning( const ibeo::ErrorWarning& ew )
{
    printf( "got error\n" );
}

void handle_scan_points( const ibeo::ScanDataPoints& points )
{
    for( size_t i= 0; i < 10; ++i )
        printf( "got scan point angle/distance: %d %d\n", points.Points[i].HorizontalAngle,
            points.Points[i].RadialDistance );
}

void handle_scan_objects( const ibeo::ScanDataObjects& objects )
{
    printf( "got objects\n" );
}


