#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: 3.10.12.0

from gnuradio import blocks
from gnuradio import blocks, gr
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import satellites.components.datasinks
import satellites.core
import threading




class SatelliteSystemPractice(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 48000

        ##################################################
        # Blocks
        ##################################################

        self.satellites_satellite_decoder_0 = satellites.core.gr_satellites_flowgraph(norad = 39444, samp_rate = samp_rate, grc_block = True, iq = False, options = " --f_offset 12000 --syncword_threshold 16")
        self.satellites_kiss_file_sink_0 = satellites.components.datasinks.kiss_file_sink('/Users/hoony/inha/ase4080/satellite_commu/decoded/funcube.kiss', append = False, options="")
        self.blocks_wavfile_source_0 = blocks.wavfile_source('/Users/hoony/inha/ase4080/satellite_commu/audio/satnogs_14130070_2026-05-22T03-43-49.ogg', False)
        self.blocks_message_debug_0 = blocks.message_debug(True, gr.log_levels.info)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.satellites_satellite_decoder_0, 'out'), (self.blocks_message_debug_0, 'print_pdu'))
        self.msg_connect((self.satellites_satellite_decoder_0, 'out'), (self.satellites_kiss_file_sink_0, 'in'))
        self.connect((self.blocks_wavfile_source_0, 0), (self.satellites_satellite_decoder_0, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate




def main(top_block_cls=SatelliteSystemPractice, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
