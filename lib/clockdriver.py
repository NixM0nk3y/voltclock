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

from datetime import datetime, timedelta

from msfdecode import MSFDecoder
from sdlds3231 import SDL_DS3231

logger = logging.getLogger(__name__)

class ClockDriverError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class ClockDriver(object):
    '''
    '''
    def __init__(self, config):
        '''
        '''

        self.config = config

        self.hours_pin = int(config.get('clockdriver', 'hour_pin'))
        self.mins_pin = int(config.get('clockdriver', 'min_pin'))
        self.seconds_pin = int(config.get('clockdriver', 'sec_pin'))

        # 
        self.gpio = pigpio.pi()

        #
        self.ds3231 = SDL_DS3231(0, int(config.get('clockdriver', 'ds3231addr'),0) )

        #
        self.msfdecoder = MSFDecoder( self.gpio, 
                                      int(config.get('clockdriver', 'msf_pin')),
                                      self.tick_callback,
                                      self.timesync_callback,
                                    )

        self.dial_update_scheduled = False

        self.running = True

        return

    def run(self):
        '''
        '''
        logger.info("Starting Clock Driver")

        self.initdials()

        self.msfdecoder.run()

        while self.running:

            time.sleep(0.1)

        return

    def stop(self):
        '''
        '''
        logger.info("Stop Clock Driver")

        self.gpio.set_PWM_dutycycle(self.hours_pin, 0 )
        self.gpio.set_PWM_dutycycle(self.mins_pin, 0 )
        self.gpio.set_PWM_dutycycle(self.seconds_pin, 0 )

        self.msfdecoder.stop()

        self.gpio.stop()

        return

    def timesync_callback(self, dt, bst_change_soon, is_bst, dut):
        '''
        '''
        logger.debug("Syncing Clock Driver")

        rtc_dt = self.ds3231.read_datetime() 

        difference = rtc_dt - dt

        if difference > timedelta(seconds=0):
            logger.warning("Difference in clocks - rtc: %s msf: %s - correcting" % (rtc_dt,dt))
            
            self.ds3231.write_datetime(dt)

        return


    def tick_callback(self):
        '''
        '''
        logger.debug("Tick Callback")

        self.update_dials( self.ds3231.read_datetime() )

        return

    def update_dials(self, dt):
        '''
        '''
        logger.debug("Updating Clock Dials")

        t = dt.timetuple()

        # twelve hour clock
        self.gpio.set_PWM_dutycycle(self.hours_pin, int( ( t.tm_hour % 12 ) / 12.0 * 255 ) )

        self.gpio.set_PWM_dutycycle(self.mins_pin, int( t.tm_min / 60.0 * 255 ) )

        self.gpio.set_PWM_dutycycle(self.seconds_pin, int( t.tm_sec / 60.0 * 255 ) )

        return


 
    def initdials(self):
        '''
        '''
        logger.debug("Initialising the dial driver")

        self.gpio.set_mode(self.hours_pin, pigpio.OUTPUT)
        self.gpio.set_mode(self.mins_pin, pigpio.OUTPUT)
        self.gpio.set_mode(self.seconds_pin, pigpio.OUTPUT)

        self.gpio.set_PWM_dutycycle(self.hours_pin, 0)
        self.gpio.set_PWM_dutycycle(self.mins_pin, 0)
        self.gpio.set_PWM_dutycycle(self.seconds_pin, 0)

        return
