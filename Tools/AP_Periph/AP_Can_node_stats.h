#pragma once


#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>


class AP_Can_node_stats {
    public:
    AP_Can_node_stats();

    uint32_t runtime; // total wallclock time spent running ArduPilot (seconds)
    uint32_t reset;   // last time AP_Stats parameters were reset (in seconds since AP_Stats Jan 1st 2016)
    
    void init();

    // copy state into underlying parameters:
    void flush();

    // periodic update function (e.g. put our values to permanent storage):
    // call at least 1Hz
    void update();

    void set_flying(bool b);

    // accessor for is_flying
    bool get_is_flying(void) const {
        return _flying_ms != 0;
    }

    // get singleton
    static AP_Can_node_stats *get_singleton(void) {
        return _singleton;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Can_node_stats *_singleton;

    struct {
        AP_Int16 bootcount;
        AP_Int32 runtime;
        AP_Int32 reset;
    } params;

    void copy_variables_from_parameters();

    uint64_t last_flush_ms; // in terms of system uptime
    const uint16_t flush_interval_ms = 30000;

    uint64_t _flying_ms;
    uint64_t _last_runtime_ms;

    void update_runtime();

    HAL_Semaphore sem;
};

namespace AP {
    AP_Can_node_stats *Can_node_stats();
};