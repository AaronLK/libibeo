/*

ibeoscan: utility to interface with ibeo LUX scanners using libibeo


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


#include <libibeo.h>


int main( int argc, char** argv )
{
    ibeo::LUXptr dev( new ibeo::LUX() );

    std::string ip= "172.16.28.75";
    unsigned int port= 12002;
    if( !dev->connect( ip, port ) )
    {
        fprintf( stderr, "Unable to connect to %s:%d: %s\n", ip.c_str(), port, dev->last_error().c_str() );
        return 1;
    }

    printf( "Connected.\n" );

    boost::this_thread::sleep( boost::posix_time::seconds(5) );


    printf( "Disconnecting.\n" );
    dev->disconnect();

    return 0;
}


