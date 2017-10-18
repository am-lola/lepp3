
#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER lepp3_trace_provider

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./lepp3/util/lepp3_tracepoint_provider.hpp"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>



/************************************************************************************
 * This file contains tracepoint class and event definitions for use with LTTng-UST.
 * For details, see: http://lttng.org/docs/v2.10/#doc-tracepoint-provider
 ************************************************************************************/


TRACEPOINT_EVENT_CLASS(
  lepp3_trace_provider,
  lepp3_event_start,
  TP_ARGS(
  ),
  TP_FIELDS(
  )
)

TRACEPOINT_EVENT_CLASS(
  lepp3_trace_provider,
  lepp3_event_end,
  TP_ARGS(
  ),
  TP_FIELDS(
  )
)


/***********************
 * General Events
 ***********************/

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  new_depth_frame,
  TP_ARGS()
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  new_rgb_frame,
  TP_ARGS()
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  obstacle_segmenter_start,
  TP_ARGS()
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  obstacle_segmenter_end,
  TP_ARGS()
)

/***********************
 * GMM Segmenter Events
 ***********************/

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  gmm_frame_start,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  gmm_frame_end,
  TP_ARGS(
  )
)


/*****************************
 * Euclidean Segmenter Events
 *****************************/

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  euclidean_frame_start,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  euclidean_frame_end,
  TP_ARGS(
  )
)


/*****************************
 * Surface Detection Events
 *****************************/

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  ransac_start,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  ransac_end,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  surface_tracker_update_start,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  surface_tracker_update_end,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  plane_inlier_update_start,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  plane_inlier_update_end,
  TP_ARGS(
  )
)

/******************************
 * Object Approximation Events
 *****************************/

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_start,
  ssv_approx_start,
  TP_ARGS(
  )
)

TRACEPOINT_EVENT_INSTANCE(
  lepp3_trace_provider,
  lepp3_event_end,
  ssv_approx_end,
  TP_ARGS(
  )
)

#endif

#include <lttng/tracepoint-event.h>
