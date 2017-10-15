#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER lepp3_trace_provider

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./lepp3/util/lepp3_tracepoint_provider.hpp"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>

#endif

#include <lttng/tracepoint-event.h>

TRACEPOINT_EVENT_CLASS(
  lepp3_trace_provider,
  lepp3_event_start,
  TP_ARGS(
    int, in_size
  ),
  TP_FIELDS(
    ctf_integer(int, in_size, in_size)
  )
)

TRACEPOINT_EVENT_CLASS(
  lepp3_trace_provider,
  lepp3_event_end,
  TP_ARGS(
    int, out_size
  ),
  TP_FIELDS(
    ctf_integer(int, out_size, out_size)
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  gmm_frame_start,
  TP_ARGS(
    int, in_size
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  gmm_frame_end,
  TP_ARGS(
    int, out_size
  )
)

