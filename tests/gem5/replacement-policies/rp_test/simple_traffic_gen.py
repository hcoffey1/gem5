# -*- coding: utf-8 -*-
# Copyright (c) 2019 Mingyuan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Mingyuan Xiang

from __future__ import print_function
from __future__ import absolute_import

# import the m5 (gem5) library created when gem5 is built
import m5
import argparse
import glob
from m5.objects import *
from MI_example_caches import MyCacheSystem
from common.ObjectList import ObjectList
dir_path = os.path.dirname(os.path.realpath(__file__))
m5.util.addToPath(dir_path + '/../../../../configs/')

print('''Remember to add --debug-flags=RubyHitMiss so that the hits and misses
information can be printed.
Use LRU replacement policy as the default. Set the default configuration file
to testLRU1.cfg
''')

# Make a list of all replcacement policies
rp_list = ObjectList(m5.objects.BaseReplacementPolicy)

# Make a list of all traffic generator configuration files
config_list = glob.glob(dir_path + '/../traces/*.cfg')
# Only keep the basename of the configuration files
for i, c in enumerate(config_list):
    config_list[i] = os.path.basename(c)

# Add argparse
parser = argparse.ArgumentParser(description='Test replacement policies',
                                 formatter_class=argparse.RawTextHelpFormatter)

# Help messages for replacement policies
rp_help = '''Replacement policies for ruby/classic system. Optional
replacement policies are:
'''
rp_help += '\n'.join(rp_list.get_names())

# Help messages for configuration files
config_help = '''Traffic generator configuration files. Optional configuration
files are:
'''
config_help += '\n'.join(config_list)

parser.add_argument('--rp', metavar='rp', help=rp_help, default='LRURP')
parser.add_argument('--config', metavar='config', help=config_help,
                                                       default='testLRU1.cfg')
args = parser.parse_args()

# create the system we are going to simulate
system = System()

# Set the clock fequency of the system (and all of its children)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()

# Set up the system
system.mem_mode = 'timing'               # Use timing accesses
system.mem_ranges = [AddrRange('512MB')] # Create an address range

# traffic generator configuration file location
if args.config not in config_list:
    fatal('Cannot find {} in traffic generator configuration files.'
                                                       .format(args.config))
config_file_path = os.path.join(dir_path, '../traces/', args.config)

# Create a pair of simple CPUs
traffic_gen = TrafficGen(config_file=config_file_path)
system.generator = traffic_gen

# Create a SimpleMemory memory controller and connect it to the membus
system.mem_ctrl = SimpleMemory()
system.mem_ctrl.range = system.mem_ranges[0]

# Create the Ruby System
system.caches = MyCacheSystem()
system.caches.setup(system, system.generator, [system.mem_ctrl],
                                                    rp_list.get(args.rp)())

# set up the root SimObject and start the simulation
root = Root(full_system = False, system = system)
# instantiate all of the objects we've created above
m5.instantiate()

exit_event = m5.simulate(5000000)
if exit_event.getCause() != 'exiting with last active thread context':
    exit(1)
