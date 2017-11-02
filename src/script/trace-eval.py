import sys
from statistics import mean, stdev
import plotly     # pip3 install plotly
import plotly.graph_objs as go
import babeltrace # apt-get install python3-babeltrace

if len(sys.argv) < 3:
    print('USAGE:\n\t', sys.argv[0], ' <trace_location> <chart_filename>')
    print('\t<trace_location> : Directory containing your trace files')
    print('\t<chart_filename> : Filename (without extension) to save visualized data in')
    exit()

trace_path = sys.argv[1]
out_file   = sys.argv[2]

trace_collection = babeltrace.TraceCollection()
trace_collection.add_trace(trace_path, 'ctf')

durations = {}    # list of all recorded durations for each event, indexed by event name
frame_lookup = {} # table to look up which duration entry came from which frame
start_times = {}  # last-seen _start trace for each event
frames = 0

# Collect durations of all events in the log
for event in trace_collection.events:
    provider, eventname = event.name.split(':')
    if provider == 'lepp3_trace_provider':
        if eventname.endswith("_start"):
            # save start time to use when we encounter the _end trace for this event
            start_times[eventname[:-6]] = event.timestamp
        elif eventname.endswith("_end"):
            # get time between end & start for this event
            if eventname[:-4] in durations:
                durations[eventname[:-4]].append(float(event.timestamp - start_times[eventname[:-4]])/1000000.0) # timestamps are in nanoseconds
            else:
                durations[eventname[:-4]] = [float(event.timestamp - start_times[eventname[:-4]])/1000000.0]

            # Store reference to duration & current frame number for plotting
            if eventname[:-4] in frame_lookup:
                frame_lookup[eventname[:-4]].append((frames, len(durations[eventname[:-4]])-1))
            else:
                frame_lookup[eventname[:-4]] = [(frames, len(durations[eventname[:-4]])-1)]
        elif eventname == 'new_depth_frame':
            frames += 1
        else:
            print('malformed event name: ', event.name)

# Print a few stats about each event
print(frames, 'total frames recorded')
for e, d in durations.items():
    print(e, len(d), 'samples')
    print('\tMax duration: ', max(d), '(ms), Min duration: ', min(d), '(ms), Avg duration: ', mean(d), '(ms), std dev: ', stdev(d))

# Organize duration data for plotting
plots = []
for e in frame_lookup:
    x_vals = []
    y_vals = []
    for frame, idx in frame_lookup[e]:
        if frame not in x_vals:
            x_vals.append(frame)
            y_vals.append(durations[e][idx])
        else:
            y_vals[len(y_vals)-1] += durations[e][idx] #add durations for events which occur multiple times in same frame
    plots.append(go.Scatter(
        x=x_vals,
        y=y_vals,
        fill='tozeroy',
        name=e
    ))
        
layout = dict(
        title='Per-frame Runtime contribution of individual pipeline steps',
        xaxis=dict(title='Frame Number'),
        yaxis=dict(title='Time (ms)')
)

# Generate HTML Plot
plotly.offline.plot(dict(data=plots, layout=layout), filename=out_file + '.html')

