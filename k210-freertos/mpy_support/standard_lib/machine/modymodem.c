/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * MicroPython-K210 YModem driver/Module
 *
 * Copyright (C) 2019 Boris Lovosevic (https://github.com/loboris)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/stream.h"
#include "extmod/vfs.h"

#include "uarths.h"
#include "py/ringbuf.h"
#include "mphalport.h"


// ==== Y-MODEM defines ====
#define PACKET_SEQNO_INDEX      (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER           (3)
#define PACKET_TRAILER          (2)
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE             (128)
#define PACKET_1K_SIZE          (1024)

#define FILE_SIZE_LENGTH        (16)

#define SOH                     (0x01)  /* start of 128-byte data packet */
#define STX                     (0x02)  /* start of 1024-byte data packet */
#define EOT                     (0x04)  /* end of transmission */
#define ACK                     (0x06)  /* acknowledge */
#define NAK                     (0x15)  /* negative acknowledge */
#define CA                      (0x18)  /* two of these in succession aborts transfer */
#define CRC16                   (0x43)  /* 'C' == 0x43, request 16-bit CRC */

#define ABORT1                  (0x41)  /* 'A' == 0x41, abort by user */
#define ABORT2                  (0x61)  /* 'a' == 0x61, abort by user */

#define NAK_TIMEOUT             (1000)
#define MAX_ERRORS              (200)

#define YM_MAX_FILESIZE         (10*1024*1024)

static volatile uarths_t *const uarths = (volatile uarths_t *)UARTHS_BASE_ADDR;

//------------------------------------------------------------------------
static unsigned short crc16(const unsigned char *buf, unsigned long count)
{
  unsigned short crc = 0;
  int i;

  while(count--) {
    crc = crc ^ *buf++ << 8;

    for (i=0; i<8; i++) {
      if (crc & 0x8000) crc = crc << 1 ^ 0x1021;
      else crc = crc << 1;
    }
  }
  return crc;
}

//--------------------------------------------------------------
static int32_t Receive_Byte (unsigned char *c, uint32_t timeout)
{
    int cc = -1;
    uarths_rxdata_t recv = uarths->rxdata;
    if (!recv.empty) {
        cc = recv.data;
    }
    if (cc >= 0) {
        *c = (uint8_t)cc;
        return 0;
    }
    while ( (cc < 0) && (timeout--)) {
        mp_hal_delay_us(100);
        recv = uarths->rxdata;
        if (!recv.empty) {
            cc = recv.data;
        }
        else cc = -1;
    }
    if (cc < 0) return -1;
	*c = (uint8_t)cc;
    return 0;
}

//------------------------
static void uart_consume()
{
    uarths_rxdata_t recv = uarths->rxdata;
    //uint8_t c;
    while (!recv.empty) {
        //c = recv.data;
        recv = uarths->rxdata;
    }
}

//----------------------------------------
static void send_Bytes(char *buf, int len)
{
    while (len--) {
        uarths_write_byte(*buf++);
    }
}
//--------------------------------
static uint32_t Send_Byte (char c)
{
	send_Bytes(&c,1);
	return 0;
}

//----------------------------
static void send_CA ( void ) {
  Send_Byte(CA);
  Send_Byte(CA);
}

//-----------------------------
static void send_ACK ( void ) {
  Send_Byte(ACK);
}

//----------------------------------
static void send_ACKCRC16 ( void ) {
  Send_Byte(ACK);
  Send_Byte(CRC16);
}

//-----------------------------
static void send_NAK ( void ) {
  Send_Byte(NAK);
}

//-------------------------------
static void send_CRC16 ( void ) {
  Send_Byte(CRC16);
}


/**
  * @brief  Receive a packet from sender
  * @param  data
  * @param  timeout
  * @param  length
  *    >0: packet length
  *     0: end of transmission
  *    -1: abort by sender
  *    -2: error or crc error
  * @retval 0: normally return
  *        -1: timeout
  *        -2: abort by user
  */
//--------------------------------------------------------------------------
static int32_t Receive_Packet (uint8_t *data, int *length, uint32_t timeout)
{
  int count, packet_size, i;
  unsigned char ch;
  *length = 0;
  
  // receive 1st byte
  if (Receive_Byte(&ch, timeout) < 0) {
	  return -1;
  }

  switch (ch) {
    case SOH:
		packet_size = PACKET_SIZE;
		break;
    case STX:
		packet_size = PACKET_1K_SIZE;
		break;
    case EOT:
        *length = 0;
        return 0;
    case CA:
    	if (Receive_Byte(&ch, timeout) < 0) {
    		return -2;
    	}
    	if (ch == CA) {
    		*length = -1;
    		return 0;
    	}
    	else return -1;
    case ABORT1:
    case ABORT2:
    	return -2;
    default:
        mp_hal_delay_ms(100);
    	uart_consume();
    	return -1;
  }

  *data = (uint8_t)ch;
  uint8_t *dptr = data+1;
  count = packet_size + PACKET_OVERHEAD-1;

  for (i=0; i<count; i++) {
	  if (Receive_Byte(&ch, timeout) < 0) {
		  return -1;
	  }
	  *dptr++ = (uint8_t)ch;;
  }

  if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff)) {
      *length = -2;
      return 0;
  }
  if (crc16(&data[PACKET_HEADER], packet_size + PACKET_TRAILER) != 0) {
      *length = -2;
      return 0;
  }

  *length = packet_size;
  return 0;
}

// Receive a file using the ymodem protocol.
//----------------------------------------------------------------------------------
int Ymodem_Receive (mp_obj_t ffd, unsigned int maxsize, char* getname, char *errmsg)
{
  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  uint8_t *file_ptr;
  char file_size[128];
  unsigned int i, file_len, write_len, session_done, file_done, packets_received, errors, size = 0;
  int packet_length = 0;
  file_len = 0;
  int eof_cnt = 0;
  
  for (session_done = 0, errors = 0; ;) {
    for (packets_received = 0, file_done = 0; ;) {
      switch (Receive_Packet(packet_data, &packet_length, NAK_TIMEOUT)) {
        case 0:  // normal return
          switch (packet_length) {
            case -1:
                // Abort by sender
                send_ACK();
                size = -1;
                sprintf(errmsg, "Abort by sender");
                goto exit;
            case -2:
                // error
                errors ++;
                if (errors > 5) {
                  send_CA();
                  size = -2;
                  sprintf(errmsg, "Error limit exceeded");
                  goto exit;
                }
                send_NAK();
                break;
            case 0:
                // End of transmission
            	eof_cnt++;
            	if (eof_cnt == 1) {
            		send_NAK();
            	}
            	else {
            		send_ACKCRC16();
            	}
                break;
            default:
              // ** Normal packet **
              if (eof_cnt > 1) {
          		send_ACK();
              }
              else if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0x000000ff)) {
                errors ++;
                if (errors > 5) {
                  send_CA();
                  size = -3;
                  sprintf(errmsg, "Wrong packet type received");
                  goto exit;
                }
                send_NAK();
              }
              else {
                if (packets_received == 0) {
                  // ** First packet, Filename packet **
                  if (packet_data[PACKET_HEADER] != 0) {
                    errors = 0;
                    // ** Filename packet has valid data
                    if (getname) {
                      for (i = 0, file_ptr = packet_data + PACKET_HEADER; ((*file_ptr != 0) && (i < 64));) {
                        *getname = *file_ptr++;
                        getname++;
                      }
                      *getname = '\0';
                    }
                    for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < packet_length);) {
                      file_ptr++;
                    }
                    for (i = 0, file_ptr ++; (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH);) {
                      file_size[i++] = *file_ptr++;
                    }
                    file_size[i++] = '\0';
                    if (strlen(file_size) > 0) size = strtol(file_size, NULL, 10);
                    else size = 0;

                    // Test the size of the file
                    if ((size < 1) || (size > maxsize)) {
                      // End session
                      send_CA();
                      if (size > maxsize) size = -9;
                      else size = -4;
                      sprintf(errmsg, "Wrong file size");
                      goto exit;
                    }

                    file_len = 0;
                    send_ACKCRC16();
                  }
                  // Filename packet is empty, end session
                  else {
                      errors ++;
                      if (errors > 5) {
                        send_CA();
                        sprintf(errmsg, "Filename packet is empty, end session");
                        size = -5;
                        goto exit;
                      }
                      send_NAK();
                  }
                }
                else {
                  // ** Data packet **
                  // Write received data to file
                  if (file_len < size) {
                    file_len += packet_length;  // total bytes received
                    if (file_len > size) {
                    	write_len = packet_length - (file_len - size);
                    	file_len = size;
                    }
                    else write_len = packet_length;

                    int written_bytes = mp_stream_posix_write(ffd, (char*)(packet_data + PACKET_HEADER), write_len);
                    //int written_bytes = fwrite((char*)(packet_data + PACKET_HEADER), 1, write_len, ffd);
                    if (written_bytes != write_len) { //failed
                      /* End session */
                      send_CA();
                      size = -6;
                      sprintf(errmsg, "fwrite() error [%d <> %d]", written_bytes, write_len);
                      goto exit;
                    }
                  }
                  //success
                  errors = 0;
                  send_ACK();
                }
                packets_received++;
              }
          }
          break;
        case -2:  // user abort
          send_CA();
          size = -7;
          sprintf(errmsg, "User abort");
          goto exit;
        default: // timeout
          if (eof_cnt > 1) {
        	file_done = 1;
          }
          else {
			  errors ++;
			  if (errors > MAX_ERRORS) {
				send_CA();
				size = -8;
                sprintf(errmsg, "Max errors");
				goto exit;
			  }
			  send_CRC16();
          }
      }
      if (file_done != 0) {
    	  session_done = 1;
    	  break;
      }
    }
    if (session_done != 0) break;
  }

exit:
  return size;
}

//------------------------------------------------------------------------------------
static void Ymodem_PrepareIntialPacket(uint8_t *data, char *fileName, uint32_t length)
{
  uint16_t tempCRC;

  memset(data, 0, PACKET_SIZE + PACKET_HEADER);
  // Make first three packet
  data[0] = SOH;
  data[1] = 0x00;
  data[2] = 0xff;
  
  // add filename
  sprintf((char *)(data+PACKET_HEADER), "%s", fileName);

  //add file site
  sprintf((char *)(data + PACKET_HEADER + strlen((char *)(data+PACKET_HEADER)) + 1), "%d", length);
  data[PACKET_HEADER + strlen((char *)(data+PACKET_HEADER)) +
	   1 + strlen((char *)(data + PACKET_HEADER + strlen((char *)(data+PACKET_HEADER)) + 1))] = ' ';
  
  // add crc
  tempCRC = crc16(&data[PACKET_HEADER], PACKET_SIZE);
  data[PACKET_SIZE + PACKET_HEADER] = tempCRC >> 8;
  data[PACKET_SIZE + PACKET_HEADER + 1] = tempCRC & 0xFF;
}

//-------------------------------------------------
static void Ymodem_PrepareLastPacket(uint8_t *data)
{
  uint16_t tempCRC;
  
  memset(data, 0, PACKET_SIZE + PACKET_HEADER);
  data[0] = SOH;
  data[1] = 0x00;
  data[2] = 0xff;
  tempCRC = crc16(&data[PACKET_HEADER], PACKET_SIZE);
  //tempCRC = crc16_le(0, &data[PACKET_HEADER], PACKET_SIZE);
  data[PACKET_SIZE + PACKET_HEADER] = tempCRC >> 8;
  data[PACKET_SIZE + PACKET_HEADER + 1] = tempCRC & 0xFF;
}

//--------------------------------------------------------------------------------------------
static void Ymodem_PreparePacket(uint8_t *data, uint8_t pktNo, uint32_t sizeBlk, mp_obj_t ffd)
{
  uint16_t i, size;
  uint16_t tempCRC;
  
  data[0] = STX;
  data[1] = (pktNo & 0x000000ff);
  data[2] = (~(pktNo & 0x000000ff));

  size = sizeBlk < PACKET_1K_SIZE ? sizeBlk :PACKET_1K_SIZE;
  // Read block from file
  if (size > 0) {
      size = mp_stream_posix_read(ffd, data + PACKET_HEADER, size);
	  //size = fread(data + PACKET_HEADER, 1, size, ffd);
  }

  if ( size  < PACKET_1K_SIZE) {
    for (i = size + PACKET_HEADER; i < PACKET_1K_SIZE + PACKET_HEADER; i++) {
      data[i] = 0x00; // EOF (0x1A) or 0x00
    }
  }
  tempCRC = crc16(&data[PACKET_HEADER], PACKET_1K_SIZE);
  //tempCRC = crc16_le(0, &data[PACKET_HEADER], PACKET_1K_SIZE);
  data[PACKET_1K_SIZE + PACKET_HEADER] = tempCRC >> 8;
  data[PACKET_1K_SIZE + PACKET_HEADER + 1] = tempCRC & 0xFF;
}

//-------------------------------------------------------------
static uint8_t Ymodem_WaitResponse(uint8_t ackchr, uint8_t tmo)
{
  unsigned char receivedC;
  uint32_t errors = 0;

  do {
    if (Receive_Byte(&receivedC, NAK_TIMEOUT) == 0) {
      if (receivedC == ackchr) {
        return 1;
      }
      else if (receivedC == CA) {
        send_CA();
        return 2; // CA received, Sender abort
      }
      else if (receivedC == NAK) {
        return 3;
      }
      else {
        return 4;
      }
    }
    else {
      errors++;
    }
  }while (errors < tmo);
  return 0;
}


//------------------------------------------------------------------------------------------
int Ymodem_Transmit (char* sendFileName, unsigned int sizeFile, mp_obj_t ffd, char *err_msg)
{
  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  uint16_t blkNumber;
  unsigned char receivedC;
  int err;
  uint32_t size = 0;

  // Wait for response from receiver
  err = 0;
  do {
    Send_Byte(CRC16);
  } while ((Receive_Byte(&receivedC, NAK_TIMEOUT) < 0) && (err++ < MAX_ERRORS));

  if (err >= MAX_ERRORS || receivedC != CRC16) {
    send_CA();
    sprintf(err_msg, "No response from host");
    return -1;
  }
  
  // === Prepare first block and send it =======================================
  /* When the receiving program receives this block and successfully
   * opened the output file, it shall acknowledge this block with an ACK
   * character and then proceed with a normal YMODEM file transfer
   * beginning with a "C" or NAK tranmsitted by the receiver.
   */
  Ymodem_PrepareIntialPacket(packet_data, sendFileName, sizeFile);
  do 
  {
    // Send Packet
	  send_Bytes((char *)packet_data, PACKET_SIZE + PACKET_OVERHEAD);

	// Wait for Ack
    err = Ymodem_WaitResponse(ACK, 10);
    if (err == 0 || err == 4) {
      send_CA();
      sprintf(err_msg, "No ACK from host");
      return -2;                  // timeout or wrong response
    }
    else if (err == 2) {
        sprintf(err_msg, "Host abort");
    	return 98; // abort
    }
  } while (err != 1);

  // After initial block the receiver sends 'C' after ACK
  if (Ymodem_WaitResponse(CRC16, 10) != 1) {
    send_CA();
    sprintf(err_msg, "No CRC after ACK");
    return -3;
  }
  
  // === Send file blocks ======================================================
  size = sizeFile;
  blkNumber = 0x01;
  
  // Resend packet if NAK  for a count of 10 else end of communication
  while (size)
  {
    // Prepare and send next packet
    Ymodem_PreparePacket(packet_data, blkNumber, size, ffd);
    do
    {
    	send_Bytes((char *)packet_data, PACKET_1K_SIZE + PACKET_OVERHEAD);

      // Wait for Ack
      err = Ymodem_WaitResponse(ACK, 10);
      if (err == 1) {
        blkNumber++;
        if (size > PACKET_1K_SIZE) size -= PACKET_1K_SIZE; // Next packet
        else size = 0; // Last packet sent
      }
      else if (err == 0 || err == 4) {
        send_CA();
        sprintf(err_msg, "Timeout or wrong response");
        return -4;                  // timeout or wrong response
      }
      else if (err == 2) {
          sprintf(err_msg, "Host abort");
    	  return -5; // abort
      }
    }while(err != 1);
  }
  
  // === Send EOT ==============================================================
  Send_Byte(EOT); // Send (EOT)
  // Wait for Ack
  do 
  {
    // Wait for Ack
    err = Ymodem_WaitResponse(ACK, 10);
    if (err == 3) {   // NAK
      Send_Byte(EOT); // Send (EOT)
    }
    else if (err == 0 || err == 4) {
      send_CA();
      sprintf(err_msg, "Timeout or wrong response on EOF");
      return -6;                  // timeout or wrong response
    }
    else if (err == 2) {
        sprintf(err_msg, "Host abort on EOT");
    	return -7; // abort
    }
  }while (err != 1);
  
  // === Receiver requests next file, prepare and send last packet =============
  if (Ymodem_WaitResponse(CRC16, 10) != 1) {
	sprintf(err_msg, "No CRC after EOF");
    send_CA();
    return -8;
  }

  Ymodem_PrepareLastPacket(packet_data);
  do 
  {
	// Send Packet
	  send_Bytes((char *)packet_data, PACKET_SIZE + PACKET_OVERHEAD);

	// Wait for Ack
    err = Ymodem_WaitResponse(ACK, 10);
    if (err == 0 || err == 4) {
      send_CA();
      sprintf(err_msg, "Timeout or wrong response on last packet");
      return -9;                  // timeout or wrong response
    }
    else if (err == 2) {
        sprintf(err_msg, "Host abort on last packet");
    	return -10; // abort
    }
  }while (err != 1);
  
  return 0; // file transmitted successfully
}


// ===== Module methods ===============================================================================

//--------------------------------------------
STATIC mp_obj_t ymodem_recv(mp_obj_t fname_in)
{
	const char *fname = mp_obj_str_get_str(fname_in);
    char err_msg[128] = {'\0'};
    int err = 1;
    mp_obj_t args[2];
    args[0] = fname_in;
    args[1] = mp_obj_new_str("wb", 2);

    // Open the file
    mp_obj_t ffd = mp_vfs_open(2, args, (mp_map_t*)&mp_const_empty_map);
	if (ffd) {
	    char orig_name[128] = {'\0'};
		mp_printf(&mp_plat_print, "\nReceiving file, please start YModem transfer on host ...\n");
		mp_printf(&mp_plat_print, "(Press \"a\" to abort)\n");

	    mp_hal_uarths_setirqhandle(NULL);
		int rec_res = Ymodem_Receive(ffd, 1000000, orig_name, err_msg);
	    mp_hal_uarths_setirq_default();

	    mp_stream_close(ffd);
		mp_printf(&mp_plat_print, "\r\n");

		if (rec_res > 0) {
			err = 0;
			mp_printf(&mp_plat_print, "File received, size=%d, original name: \"%s\"\n", rec_res, orig_name);
		}
	}
	else {
		sprintf(err_msg, "Opening file \"%s\" for writing.", fname);
	}

	mp_printf(&mp_plat_print, "\n%s%s\n", ((err == 0) ? "" : "Error: "), err_msg);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ymodem_recv_obj, ymodem_recv);

//--------------------------------------------
STATIC mp_obj_t ymodem_send(mp_obj_t fname_in)
{
    const char *fname = mp_obj_str_get_str(fname_in);
    int err = 0;
    char err_msg[128] = {'\0'};
    mp_obj_t args[2];
    args[0] = fname_in;
    args[1] = mp_obj_new_str("rb", 2);

	// Open the file
    mp_obj_t ffd = mp_vfs_open(2, args, (mp_map_t*)&mp_const_empty_map);
	if (ffd) {
	    // Get file size
        int fsize = mp_stream_posix_lseek(ffd, 0, SEEK_END);
        int at_start = mp_stream_posix_lseek(ffd, 0, SEEK_SET);
        if ((fsize <= 0) || (at_start != 0)) {
            sprintf(err_msg, "Error getting file size");
        }
        else {
            mp_printf(&mp_plat_print, "\nSending file, please start YModem receive on host ...\n");
            mp_printf(&mp_plat_print, "(Press \"a\" to abort)\n");

            mp_hal_uarths_setirqhandle(NULL);
            int trans_res = Ymodem_Transmit((char *)fname, fsize, ffd, err_msg);
            mp_hal_uarths_setirq_default();

            mp_stream_close(ffd);

            mp_printf(&mp_plat_print, "\r\n");
            if (trans_res == 0) {
                err = 0;
                sprintf(err_msg, "Transfer complete, %d bytes sent", fsize);
            }
        }
	}
	else sprintf(err_msg, "Opening file \"%s\" for reading.", fname);

    mp_printf(&mp_plat_print, "\n%s%s\n", ((err == 0) ? "" : "Error: "), err_msg);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ymodem_send_obj, ymodem_send);



//--------------------------------------------------------------
STATIC const mp_rom_map_elem_t ymodem_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ymodem) },

    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&ymodem_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&ymodem_recv_obj) }
};

STATIC MP_DEFINE_CONST_DICT(ymodem_module_globals, ymodem_module_globals_table);

//----------------------------------------
const mp_obj_module_t mp_module_ymodem = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ymodem_module_globals,
};

