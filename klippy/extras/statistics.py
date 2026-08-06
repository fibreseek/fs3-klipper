# Support for logging periodic statistics
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, time, logging

class PrinterSysStats:
    def __init__(self, config):
        printer = config.get_printer()
        self.last_process_time = self.total_process_time = 0.
        self.last_load_avg = 0.
        self.last_mem_avail = 0
        self.mem_file = None
        try:
            self.mem_file = open("/proc/meminfo", "r")
        except:
            pass
        printer.register_event_handler("klippy:disconnect", self._disconnect)
    def _disconnect(self):
        if self.mem_file is not None:
            self.mem_file.close()
            self.mem_file = None
    def stats(self, eventtime):
        # Get core usage stats
        ptime = time.process_time()
        pdiff = ptime - self.last_process_time
        self.last_process_time = ptime
        if pdiff > 0.:
            self.total_process_time += pdiff
        self.last_load_avg = os.getloadavg()[0]
        msg = "sysload=%.2f cputime=%.3f" % (self.last_load_avg,
                                             self.total_process_time)
        # Get available system memory
        if self.mem_file is not None:
            try:
                self.mem_file.seek(0)
                data = self.mem_file.read()
                for line in data.split('\n'):
                    if line.startswith("MemAvailable:"):
                        self.last_mem_avail = int(line.split()[1])
                        msg = "%s memavail=%d" % (msg, self.last_mem_avail)
                        break
            except:
                pass
        return (False, msg)
    def get_status(self, eventtime):
        return {'sysload': self.last_load_avg,
                'cputime': self.total_process_time,
                'memavail': self.last_mem_avail}

class PrinterStats:
    STATS_LOG_INTERVAL = 5.
    def __init__(self, config):
        self.printer = config.get_printer()
        reactor = self.printer.get_reactor()
        self.stats_timer = reactor.register_timer(self.generate_stats)
        self.stats_cb = []
        self.print_stats = None
        self.last_stats_log_time = 0.
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
    def handle_ready(self):
        self.stats_cb = [o.stats for n, o in self.printer.lookup_objects()
                         if hasattr(o, 'stats')]
        self.print_stats = self.printer.lookup_object('print_stats', None)
        if self.printer.get_start_args().get('debugoutput') is None:
            reactor = self.printer.get_reactor()
            reactor.update_timer(self.stats_timer, reactor.NOW)
    def generate_stats(self, eventtime):
        stats = [cb(eventtime) for cb in self.stats_cb]
        # 仅打印中每 5s 记录一次，降低 klippy.log 体积
        if (self.print_stats is not None
                and self.print_stats.state == "printing"
                and eventtime - self.last_stats_log_time >= self.STATS_LOG_INTERVAL
                and max([s[0] for s in stats])):
            logging.info("Stats %.1f: %s", eventtime,
                         ' '.join([s[1] for s in stats]))
            self.last_stats_log_time = eventtime
        return eventtime + 1.

def load_config(config):
    config.get_printer().add_object('system_stats', PrinterSysStats(config))
    return PrinterStats(config)
