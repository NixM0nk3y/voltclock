#!/usr/bin/python3
#
#
#
#

import time
import pigpio
import pytz
import math
import logging
import traceback

from datetime import datetime

logger = logging.getLogger(__name__)

class MSFDecoderError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MSFDecoder(object):

    """
    """
    def __init__(self,gpio, msf_pin, tick_callback=None, sync_callback=None):

        self.pi = gpio

        self.gpio_pin = msf_pin

        self.tick_cb = tick_callback
        self.timesync_cb = sync_callback

        self.ignore_pulse_length = 50000
        self.pulse_margin = 35000

        self.off_millis = 0
        self.off_width = 0
        self.on_millis = 0
        self.on_width = 0

        self.bad_packets = 0
        self.skip_transition = 0

        self.msf_data = {}
        self.msf_data['a'] = []
        self.msf_data['b'] = []

        self.msf_callback = None

        return

    def run(self):
        '''
        '''

        self.msf_callback = self.pi.callback(self.gpio_pin, pigpio.EITHER_EDGE, self.process_callback)

        self.pi.set_watchdog(self.gpio_pin, 500)

        return

    def stop(self):
        '''
        '''
        logger.info("stopping msf decoder")

        if self.msf_callback:
            self.msf_callback.cancel()

        return
        
    def decode_bcd(self, bitlist):
        '''
        '''
        packed_var = 0
        
        nibble_size = int( math.ceil( len(bitlist) / 4.0 ) )
        
        # pack the bit array into a proper bin type
        for bit in bitlist:
            packed_var = (packed_var << 1) | bit
        
        bcd_val = 0
        
        for i in range(nibble_size):
        
            bcd_nibble = (packed_var & 0b1111) * ( 10 ** i)
            
            bcd_val = bcd_val + bcd_nibble
            
            packed_var = packed_var >> 4
        
        return bcd_val
        
    def validate_data(self):
        '''
        '''
        if len(self.msf_data['a']) != 59:
            raise MSFDecoderError("MSF Data incorrect length %d - leap second?" % len(self.msf_data['a']),self.msf_data)

        if self.msf_data['a'][-8:] != [0, 1, 1, 1, 1, 1, 1, 0]:
            raise MSFDecoderError("unable to find second marker")

        if (sum(self.msf_data['a'][16:24]) % 2 ) == self.msf_data['b'][53]:
            raise MSFDecoderError("Bits 16:24 Parity Failed")

        if (sum(self.msf_data['a'][24:35]) % 2 ) == self.msf_data['b'][54]:
            raise MSFDecoderError("Bits 24:35 Parity Failed")

        if (sum(self.msf_data['a'][35:38]) % 2 ) == self.msf_data['b'][55]:
            raise MSFDecoderError("Bits 35:38 Parity Failed")

        if (sum(self.msf_data['a'][38:51]) % 2 ) == self.msf_data['b'][56]:
            raise MSFDecoderError("Bits 38:51 Parity Failed")

        return

    def process_msfdata(self,century=21):
        '''
            Take a bitstream of msf data - validate and decode
        '''
        
        self.validate_data()

        year    = self.decode_bcd(self.msf_data['a'][16:24])
        month   = self.decode_bcd(self.msf_data['a'][24:29])
        day     = self.decode_bcd(self.msf_data['a'][29:35])
        weekday = self.decode_bcd(self.msf_data['a'][35:38])
        hour    = self.decode_bcd(self.msf_data['a'][38:44])
        minute  = self.decode_bcd(self.msf_data['a'][44:51])

        # http://maia.usno.navy.mil/ser7/ser7.dat
        dut     = 100 * sum(self.msf_data['b'][0:8]) + sum(self.msf_data['b'][8:16]) * -100

        bst_change_soon = self.msf_data['b'][52] == 1
        is_bst = self.msf_data['b'][57] == 1

        dt = datetime( (century - 1) * 100 + year, month, day, hour, minute, 0, 0 )

        logger.debug("timestamp:%s , bst:%s , bst_soon:%s, dut:%d" % (dt, is_bst, bst_change_soon, dut))
        
        if bst_change_soon:
            logger.warning("Daylight savings transition due in next 60 mins")

        try:
            if self.timesync_cb:
                # attempt to do something with the data
                self.timesync_cb(dt, bst_change_soon, is_bst, dut)
        except Exception as e:
            logging.error(traceback.format_exc())
        return


    def process_tick(self, level, tick):
        '''
            Take data from from the last GPIO transistion - figure out how long since
            the last transistion and normalise the pulse to with 100ms.
        '''

        normalised_pulse = 0

        # init pulse length
        if level == pigpio.FALLING_EDGE and self.on_millis == 0:
            self.on_millis = tick

        if level == pigpio.RISING_EDGE and self.off_millis == 0:
            self.off_millis = tick

        # our internal ms clock wrapped
        if tick < self.on_millis or tick < self.off_millis:
            self.off_millis = 0
            self.on_millis = 0

        # we haven't see both a on and off transition yes
        if self.off_millis == 0 or self.on_millis == 0:
            return

        if level == pigpio.RISING_EDGE:
            # carrier off
            pulse_length = tick - self.on_millis

            self.off_millis = tick

        elif level == pigpio.FALLING_EDGE:
            # carrier on
            pulse_length = tick - self.off_millis

            self.on_millis = tick
        else:
            return

        if pulse_length < self.ignore_pulse_length:
            logger.debug("IGNORE - %d , length = %d" % ( level, pulse_length))
            self.skip_transition = 1
        else:

            if abs(pulse_length - 900000) < self.pulse_margin:
                normalised_pulse = 900
            elif abs(pulse_length - 800000) < self.pulse_margin:
                normalised_pulse = 800
            elif abs(pulse_length - 700000) < self.pulse_margin:
                normalised_pulse = 700
            elif abs(pulse_length - 500000) < self.pulse_margin:
                normalised_pulse = 500
            elif abs(pulse_length - 400000) < self.pulse_margin:
                normalised_pulse = 400
            elif abs(pulse_length - 300000) < self.pulse_margin:
                normalised_pulse = 300
            elif abs(pulse_length - 200000) < self.pulse_margin:
                normalised_pulse = 200
            elif abs(pulse_length - 100000) < self.pulse_margin:
                normalised_pulse = 100
            else:
                self.bad_packets += 1
                logger.warning("unexpected pulse - length = %d" % ( pulse_length ))

        return normalised_pulse

    def process_callback(self, gpio, level, tick):
        '''
            http://www.npl.co.uk/science-technology/time-frequency/products-and-services/time/msf-radio-time-signal
            GPIO transition callback - extract ook style data and decode the data
            Cases:
                A - 500mS carrier-off marks the start of a minute
                B - 300mS carrier-off means bits 1 1
                C - 200mS carrier-off means bits 1 0
                D - 100mS carrier-off followed by a 900mS carrier-on means bits 0 0
                E - 100mS carrier-off followed by a 100mS carrier-on followed by a 100mS carrier-off means bits 0 1              
        '''

        # ignoring both sides of a blip
        if self.skip_transition:
            self.skip_transition = 0
            return
    
        #
        normalised_pulse = self.process_tick(level, tick)
        
        #  carrier on ( inverted logic )
        if level == 0 and normalised_pulse:
       
            if normalised_pulse == 500:

                if self.bad_packets == 0 and 58 <= len(self.msf_data['a']) <= 60:
                                    
                    try:
                        self.process_msfdata()
                        
                    except MSFDecoderError as e:

                        logger.error(e.value)
                else:
                    logger.warning("Bad packets seen %d, Collected %d/60 Packets - Skipping" % ( self.bad_packets, len(self.msf_data['a'])))

                # reset our data for next min
                self.bad_packets = 0
                self.msf_data['a'] = []
                self.msf_data['b'] = []

            elif normalised_pulse == 100:

                #  process state E
                if self.off_width == 100 and self.on_width == 100:

                        self.msf_data['a'].append(0)
                        self.msf_data['b'].append(1)

                else:
                        # state D - need to check carrier off period
                        pass

            elif normalised_pulse == 200:

                self.msf_data['a'].append(1)
                self.msf_data['b'].append(0)

            elif normalised_pulse == 300:

                self.msf_data['a'].append(1)
                self.msf_data['b'].append(1)

            else:
                 logger.warning("Unknown pulse", normalised_pulse)
                 self.bad_packets += 1

            self.on_width = normalised_pulse

        elif level == 1 and normalised_pulse:

            if normalised_pulse == 500 and self.on_width == 500:        
            
                pass # end of the second marker
                
            elif normalised_pulse == 900 and self.on_width == 100:

                self.msf_data['a'].append(0)
                self.msf_data['b'].append(0)

            elif (normalised_pulse == 100 and self.on_width == 100):
             
                pass # can't really judge yet - looking like a tri-transition second

            elif (normalised_pulse == 700 and self.on_width == 100):

                pass

            elif normalised_pulse + self.on_width != 1000:

                logger.warning("unexpected data packet/carrier period = %d" % ( normalised_pulse + self.on_width ))
                self.bad_packets += 1

            self.off_width = normalised_pulse

            #
            try:

                logger.debug("Tick: %d" % ( normalised_pulse + self.on_width) )

                if self.tick_cb:
                    self.tick_cb()
            except Exception as e:
                logging.error(traceback.format_exc())

        return

if __name__ == "__main__":

    FORMAT = "%(asctime)-15s:%(levelname)s:%(message)s"
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)
   
    gpio = pigpio.pi()

    msf = MSFDecoder(gpio,24)
        
    msf.run()

    try:
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass

    msf.stop()
        
