/** THE FOLLOWING CODE IS FOR THE NEW VERSION OF THE ROBOTS **/
//  Built on December 9, 2013
//  Author: Thommen George, Yashwanth Tumuluri
//This code is used to establish RF communication between a mother node and up to 6 child nodes. For the mother node, just run this code as it is.
//Each of the child nodes is on a unique pipe. Depending on to which pipe the robot is on, comment out the appropriate sections


#include "mbed.h"
#include "nRF24L01P.h"
#include <ctype.h>
//Robot RX addresses in pipes 2-5 need to be only 1 byte. The MSBs will automatically be taken as C2C2C2C2
//THIS ONE WORKS
//#define ROBOT1_ADDRESS      ((unsigned long long) 0xE7D3F03577 )//17E7E7E7E7 )//
//#define ROBOT2_ADDRESS      ((unsigned long long) 0xC2C2C2C2C2 )//0x17E7E7E7E7 )
//#define ROBOT3_ADDRESS      ((unsigned long long) 0xC2C2C2C2C3)//
/*#define RX_ADDR_P0          ((unsigned long long) 0x7878787878)
#define RX_ADDR_P1          ((unsigned long long) 0xB3B4B5B6F1)
#define RX_ADDR_P2          ((unsigned long long) 0xB3B4B5B6CD)
#define RX_ADDR_P3          ((unsigned long long) 0xB3B4B5B6A3)
#define RX_ADDR_P4          ((unsigned long long) 0xB3B4B5B60F)
#define RX_ADDR_P5          ((unsigned long long) 0xB3B4B5B605)
*/
/////////////////////////THIS WORKED////////////////////////////////
#define RX_ADDR_P0          ((unsigned long long) 0xE7D3F03577)
#define RX_ADDR_P1          ((unsigned long long) 0xC2C2C2C2C2)
#define RX_ADDR_P2          ((unsigned long long) 0xC2C2C2C2C3)
#define RX_ADDR_P3          ((unsigned long long) 0xC2C2C2C2C4)
#define RX_ADDR_P4          ((unsigned long long) 0xC2C2C2C2C5)
#define RX_ADDR_P5          ((unsigned long long) 0xC2C2C2C2C6)

//////////////////////////////////////////////////////////////

/*#define RX_ADDR_P5          ((unsigned long long) 0xE7D3F03577)
#define RX_ADDR_P1          ((unsigned long long) 0xC2C2C2C2C2)
#define RX_ADDR_P3          ((unsigned long long) 0xC2C2C2C2C3)
#define RX_ADDR_P2          ((unsigned long long) 0xC2C2C2C2C4)
#define RX_ADDR_P4          ((unsigned long long) 0xC2C2C2C2C5)
#define RX_ADDR_P0          ((unsigned long long) 0xC2C2C2C2C6)*/
//#define ALLROBOT_ADDRESS    ((unsigned long long) 0xE7E7E7E7E7 )
//////////////////Alternate addresses tried out/////////////////
//#define ROBOT3_ADDRESS      ((unsigned long long) 0xC2C2C2C2C2 )
//#define ROBOT3_ADDRESS      ((unsigned long long) 0x37E7E7E7E7 )
//#define ROBOT3_ADDRESS      ((unsigned long long) 0xC4 )
#define ADDRESS_WIDTH       5

DigitalOut myled(PTE3); //MCU_STATUS LED//
DigitalOut btSwitch(PTE25);//Bluetooth Switch
Serial bt(PTA2,PTA1);   //Bluetooth as Serial port//

nRF24L01P my_nrf24l01p(PTD2, PTD3, PTD1, PTD0, PTC17, PTD4);    // mosi, miso, sck, csn, ce, irq //CSN has been changed from PTD5 to PTC17


int main()
{

#define TRANSFER_SIZE   1

    char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE];
    int txDataCnt = 0;
    int rxDataCnt = 0;
    char TXDATA[TRANSFER_SIZE];
    int TXDATACnt = 0;
    btSwitch = 1;
    my_nrf24l01p.powerUp();
    my_nrf24l01p.setRfFrequency(2400);

    ////////////JUST CHANGE THIS////////////////////////
    /** unsigned long long RX_ROBOT=ROBOT3_ADDRESS;//Choose the RX robot and pipe
     int RX_PIPE=0;
     unsigned long long TX_ROBOT=ROBOT2_ADDRESS; //Choose the TX robot and pipe
     int TX_PIPE=1;**/
    ////////////////////////////////////////

//bt.baud(115200);    //set the baud rate for the Bluetooth (new board)
    bt.baud(9600);//baud rate for old board
    while(!bt.readable()) bt.printf("Type something\n\r");    //wait until user has responded

    // Display the (default) setup of the nRF24L01+ chip
    bt.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );
    //bt.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    //bt.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    //bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );//gets address of pipe 0 (default)


//Set transfer rate for each of the pipes individually
    my_nrf24l01p.setTransferSize( TRANSFER_SIZE,0 );//default-sets the transfer size of pipe 0. (Refer nRF24L01P class documentation)
    my_nrf24l01p.setTransferSize( TRANSFER_SIZE,1 );
    my_nrf24l01p.setTransferSize( TRANSFER_SIZE,2 );
    my_nrf24l01p.setTransferSize( TRANSFER_SIZE,3 );
    my_nrf24l01p.setTransferSize( TRANSFER_SIZE,4 );
    my_nrf24l01p.setTransferSize( TRANSFER_SIZE,5 );

    my_nrf24l01p.setReceiveMode();//into receive mode
    my_nrf24l01p.enable();//enable to receive or transmit

     
    //Set the RECEIVE ADDRESS
    //my_nrf24l01p.setRxAddress(RX_ADDR_P0,ADDRESS_WIDTH,0);  //CHANGE-Keep same for mother node. For child node, keep only one of these according to which pipe it is on.
    //my_nrf24l01p.setRxAddress(RX_ADDR_P1,ADDRESS_WIDTH,1);  //sets the pipe to associate this address with
    //my_nrf24l01p.setRxAddress(RX_ADDR_P2,ADDRESS_WIDTH,2);  //sets the pipe to associate this address with
    //my_nrf24l01p.setRxAddress(RX_ADDR_P3,ADDRESS_WIDTH,3);  //sets the pipe to associate this address with
    //my_nrf24l01p.setRxAddress(RX_ADDR_P4,ADDRESS_WIDTH,4);  //sets the pipe to associate this address with
    my_nrf24l01p.setRxAddress(RX_ADDR_P5,ADDRESS_WIDTH,5);  //sets the pipe to associate this address with*/

    //IGNORE 'ADDRESS WIDTH' FOR pipes 2to5 (Refer nRF24L01P class documentation)
    //bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress(0) );//CHANGE - accordingly
   // bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress(1) );//address of pipe 0
    //bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress(2) );//address of pipe 0
    //bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress(3) );//address of pipe 0
    //bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress(4) );//address of pipe 0
    bt.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress(5) );//address of pipe 0

    while (1) {

        // If we've received anything over the host serial link...
        if ( bt.readable() ) {

            // ...add it to the transmit buffer
            txData[txDataCnt] = bt.getc();
            TXDATA[txDataCnt]= toupper(txData[txDataCnt]);
            txDataCnt ++;
            // If the transmit buffer is full
            if ( txDataCnt >= sizeof( txData ) ) {


                 //my_nrf24l01p.setTxAddress(RX_ADDR_P0,ADDRESS_WIDTH);    ////CHANGE-Keep same for mother node. For child node, keep only one of these according to which pipe it is on.
                //Send the transmitbuffer via the nRF24L01+
                 //my_nrf24l01p.write( 0, txData, txDataCnt );  //send through pipe 1

                 //my_nrf24l01p.setTxAddress(RX_ADDR_P1,ADDRESS_WIDTH);    //Set which address to send to
                //Send the transmitbuffer via the nRF24L01+
                //my_nrf24l01p.write( 1, txData, txDataCnt );  //send through pipe 1

                //my_nrf24l01p.setTxAddress(RX_ADDR_P2,ADDRESS_WIDTH);    //Set which address to send to
                //Send the transmitbuffer via the nRF24L01+
               // my_nrf24l01p.write( 2, txData, txDataCnt );  //send through pipe 1


                //my_nrf24l01p.setTxAddress(RX_ADDR_P3,ADDRESS_WIDTH);    //Set which address to send to
                //Send the transmitbuffer via the nRF24L01+
                //my_nrf24l01p.write( 3, txData, txDataCnt );  //send through pipe 1

                //my_nrf24l01p.setTxAddress(RX_ADDR_P4,ADDRESS_WIDTH);    //Set which address to send to
                //my_nrf24l01p.write( 4, TXDATA, txDataCnt);//Send through pipe 2

                my_nrf24l01p.setTxAddress(RX_ADDR_P5,ADDRESS_WIDTH);    //Set which address to send to
                //Send the transmitbuffer via the nRF24L01+
                my_nrf24l01p.write( 5, txData, txDataCnt );  //send through pipe 1
                 //bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );*/


                txDataCnt = 0;
            }

            // Toggle LED1 (to help debug Host -> nRF24L01+ communication)
            myled = !myled;
        }

        // If we've received anything in the nRF24L01+...
        /*if ( my_nrf24l01p.readable(0) ) {    //CHANGE- to readable(1) if you want to read from pipe 1 and so on for the other pipes
            
            my_nrf24l01p.enableAutoAcknowledge(0);
            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( 0, rxData, sizeof( rxData ) );//read from pipe 0

            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                bt.putc( rxData[i] );
            }
            
             my_nrf24l01p.disableAutoAcknowledge(); 
             bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower());
            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled = !myled;
        }


        if ( my_nrf24l01p.readable(1) ) {    //change this to .readable(1) to check if pipe 1 is readable
            
            my_nrf24l01p.enableAutoAcknowledge(1);
            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( 1, rxData, sizeof( rxData ) );//read from pipe 0

            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                bt.putc( rxData[i] );
            }
            
             my_nrf24l01p.disableAutoAcknowledge();
             bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower()); 
            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled = !myled;
        }



        if ( my_nrf24l01p.readable(2) ) {    //change this to .readable(1) to check if pipe 1 is readable
            
            
            my_nrf24l01p.enableAutoAcknowledge(2);
            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( 2, rxData, sizeof( rxData ) );//read from pipe 0

            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                bt.putc( rxData[i] );
            }
           
             my_nrf24l01p.disableAutoAcknowledge(); 
             bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower());
            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled = !myled;
        }

        if ( my_nrf24l01p.readable(3) ) {    //change this to .readable(1) to check if pipe 1 is readable
            
            my_nrf24l01p.enableAutoAcknowledge(3);
            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( 3, rxData, sizeof( rxData ) );//read from pipe 0

            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                bt.putc( rxData[i] );
            }
           
             my_nrf24l01p.disableAutoAcknowledge(); 
             bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower());
            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled = !myled;
        }


        if ( my_nrf24l01p.readable(4) ) {    //change this to .readable(1) to check if pipe 1 is readable
            
            my_nrf24l01p.enableAutoAcknowledge(4);
            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( 4, rxData, sizeof( rxData ) );//read from pipe 0

            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                bt.putc( rxData[i] );
            }
            
            my_nrf24l01p.disableAutoAcknowledge();
            bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower()); 
            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled = !myled;
        }*/



        if ( my_nrf24l01p.readable(5) ) {    //change this to .readable(1) to check if pipe 1 is readable
            
            my_nrf24l01p.enableAutoAcknowledge(5);
            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( 5, rxData, sizeof( rxData ) );//read from pipe 0

            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                bt.putc( rxData[i] );
            }
            
              my_nrf24l01p.disableAutoAcknowledge(); 
              bt.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower());
            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled = !myled;
        }
        else {
                bt.printf( "Pipe not readable");            
            }
    }
}