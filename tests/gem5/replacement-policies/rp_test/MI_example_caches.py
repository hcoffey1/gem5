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

import math

from m5.defines import buildEnv
from m5.util import fatal, panic

from m5.objects import *

class MyCacheSystem(RubySystem):

    def __init__(self):
        if buildEnv['PROTOCOL'] != 'MI_example':
            fatal("This system assumes MI_example!")

        super(MyCacheSystem, self).__init__()

    def setup(self, system, generator, mem_ctrls, replacement_policy):
        """Set up the Ruby cache subsystem. Note: This can't be done in the
           constructor because many of these items require a pointer to the
           ruby system (self). This causes infinite recursion in initialize()
           if we do this in the __init__.
        """
        # Ruby's global network.
        self.network = MyNetwork(self)

        # MI example uses 5 virtual networks
        self.number_of_virtual_networks = 5
        self.network.number_of_virtual_networks = 5

        # Though we only have one traffic generator here, create a list
        # of controlllers so that we can pass it to network.connectControllers
        self.controllers = [L1Cache(
                               system, self, generator, replacement_policy),
                            DirController(self, system.mem_ranges, mem_ctrls)]

        self.sequencers = [RubySequencer(version = 0,
                                # I/D cache is combined and grab from ctrl
                                icache = self.controllers[0].cacheMemory,
                                dcache = self.controllers[0].cacheMemory,
                                clk_domain = self.controllers[0].clk_domain),]

        self.controllers[0].sequencer = self.sequencers[0]

        self.num_of_sequencers = 1

        # Create the network and connect the controllers.
        self.network.connectControllers(self.controllers)
        self.network.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.sys_port_proxy = RubyPortProxy()
        system.system_port = self.sys_port_proxy.slave

        # Connect the generator's cache ports to Ruby
        generator.port = self.controllers[0].sequencer.slave

class L1Cache(L1Cache_Controller):

    _version = 0
    @classmethod
    def versionCount(cls):
        cls._version += 1 # Use count for this particular type
        return cls._version - 1

    def __init__(self, system, ruby_system, generator, replacement_policy):
        """CPUs are needed to grab the clock domain and system is needed for
           the cache block size.
        """
        super(L1Cache, self).__init__()

        self.version = self.versionCount()
        # This is the cache memory object that stores the cache data and tags
        self.cacheMemory = RubyCache(size = '512B',
                               assoc = 4,
                               # set replacement policies
                               replacement_policy = replacement_policy,
                               start_index_bit = self.getBlockSizeBits(system))
        self.clk_domain = generator.clk_domain
        self.send_evictions = self.sendEvicts(generator)
        self.ruby_system = ruby_system
        self.connectQueues(ruby_system)

    def getBlockSizeBits(self, system):
        bits = int(math.log(system.cache_line_size, 2))
        if 2**bits != system.cache_line_size.value:
            panic("Cache line size not a power of 2!")
        return bits

    def sendEvicts(self, generator):
        """True if the CPU model or ISA requires sending evictions from caches
           to the CPU. Two scenarios warrant forwarding evictions to the CPU:
           1. The O3 model must keep the LSQ coherent with the caches
           2. The x86 mwait instruction is built on top of coherence
           3. The local exclusive monitor in ARM systems
        """
        if type(generator) is DerivO3CPU or \
           buildEnv['TARGET_ISA'] in ('x86', 'arm'):
            return True
        return False

    def connectQueues(self, ruby_system):
        """Connect all of the queues for this controller.
        """
        self.mandatoryQueue = MessageBuffer()
        self.requestFromCache = MessageBuffer(ordered = True)
        self.requestFromCache.master = ruby_system.network.slave
        self.responseFromCache = MessageBuffer(ordered = True)
        self.responseFromCache.master = ruby_system.network.slave
        self.forwardToCache = MessageBuffer(ordered = True)
        self.forwardToCache.slave = ruby_system.network.master
        self.responseToCache = MessageBuffer(ordered = True)
        self.responseToCache.slave = ruby_system.network.master

class DirController(Directory_Controller):

    _version = 0
    @classmethod
    def versionCount(cls):
        cls._version += 1 # Use count for this particular type
        return cls._version - 1

    def __init__(self, ruby_system, ranges, mem_ctrls):
        """ranges are the memory ranges assigned to this controller.
        """
        if len(mem_ctrls) > 1:
            panic("This cache system can only be connected to one mem ctrl")
        super(DirController, self).__init__()
        self.version = self.versionCount()
        self.addr_ranges = ranges
        self.ruby_system = ruby_system
        self.directory = RubyDirectoryMemory()
        # Connect this directory to the memory side.
        self.memory = mem_ctrls[0].port
        self.connectQueues(ruby_system)

    def connectQueues(self, ruby_system):
        self.requestToDir = MessageBuffer(ordered = True)
        self.requestToDir.slave = ruby_system.network.master
        self.dmaRequestToDir = MessageBuffer(ordered = True)
        self.dmaRequestToDir.slave = ruby_system.network.master

        self.responseFromDir = MessageBuffer()
        self.responseFromDir.master = ruby_system.network.slave
        self.dmaResponseFromDir = MessageBuffer(ordered = True)
        self.dmaResponseFromDir.master = ruby_system.network.slave
        self.forwardFromDir = MessageBuffer()
        self.forwardFromDir.master = ruby_system.network.slave
        self.responseFromMemory = MessageBuffer()

class MyNetwork(SimpleNetwork):
    """A simple point-to-point network. This doesn't not use garnet.
    """

    def __init__(self, ruby_system):
        super(MyNetwork, self).__init__()
        self.netifs = []
        self.ruby_system = ruby_system

    def connectControllers(self, controllers):
        """Connect all of the controllers to routers and connect the routers
           together in a point-to-point network.
        """
        # Create one router/switch per controller in the system
        self.routers = [Switch(router_id = i) for i in range(len(controllers))]

        # Make a link from each controller to the router. The link goes
        # externally to the network.
        self.ext_links = [SimpleExtLink(link_id=0, ext_node=c,
                                        int_node=self.routers[i])
                          for i, c in enumerate(controllers)]

        # Make an "internal" link (internal to the network) between every pair
        # of routers.
        link_count = 0
        self.int_links = []
        for ri in self.routers:
            for rj in self.routers:
                if ri == rj: continue # Don't connect a router to itself!
                link_count += 1
                self.int_links.append(SimpleIntLink(link_id = link_count,
                                                    src_node = ri,
                                                    dst_node = rj))
