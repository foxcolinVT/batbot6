/*
 * Author: Ben Westcott
 * Date created: 9/5/22
 */

#include <header/ML_M4_PlasmaDriver_NI.hpp>
#include <header/utils.hpp>

void SERCOM_SPI_init(SercomSpi *SPIx, const enum ML_SPI_mode mode){

  /*
   Enable protected registers:
      CTRLA, except ENABLE, and SWRST bits
      CTRLB, except RXEN bit
      BAUD
      ADDR
  */

  //Synchronized bits: CTRLA.SWRST, CTRLA.ENABLE, CTRLB.RXEN

 /*
  initialization:
      1) master/slave mode: CTRLA.MODE = 0x2, 0x3
      2) transfer mode for clock polarity: CTRLA.CPOL or CTRLA.CPHA
      3) frame format value CTRLA.FORM
      4) data in pinout bit group set: CTRLA.DIPO
      5) data out pinout bit group set: CTRLA.DOPO
      6) character size value CTRLA.CHSIZE
      7) data order bit: CTRLA.DORD
      8) If master mode: 
         8.1) BAUD rate in BAUD reg
         8.2) Slave select control required, CTRLB.MSSEN
      9) enable RX: CTRLB.RXEN
 */

  /*
    Clock generation: CTRLA.MODE = 0x3 (master) -> SCK generated internally by baud rate generator
                                   0x2 (slave) -> SCK pin
  */

 // RxDATA and TxData share same IO line. Read operation returns RxData, Write operation writes to TxData

 /*
  Master mode:
    If CTRLB.MSSEN = 1, SS' controled by harware. Else SS' must be configured as output through GPIO,
    and pulled low when ready to transmit data

    When writing a char to data, char will be transferred to shift reg. When finished, INTFLAG.DRE is set
    and new char can be written to data

    Ea. time char shifted out from master, char shifted in from slave if CTRLB.RXEN, contents of shift reg
    will be transferred to recieve buffer. INTFLAG.RXC will be set

    When last char has been transmitted and no data left in DATA and INTFLAG.TXC is set. If CTRLB.MSSEN = 0,
    software must pull SS' high to notify slave of complete transaction


  Slave mode:
    When SS' pulled low and SCK running, slave will sample and shift out data according to transaction mode.
    When content of TxDATA loaded into shift reg, INTFLAG.DRE set and new data can be written to DATA

    Like master mode, slave will recieve one char for every char transmitted. Recieved char can be retrieved
    once INTFLAG.RXC is set

    When master pulls SS' high, transaction finished and INTFLAG.TXC is set
 */

SERCOM_SPI_disable(SPIx);

SERCOM_SPI_swrst(SPIx);

}